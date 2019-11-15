/**
 * File:  flash-drive.h
 * Author: Daniel Carvalho Dias (daniel.dias@gmail.com)
 * Date:   05/08/2019
 *
 * Implements the interface to save and load data to the external flash drive of
 * CC2650 control board. The flash drive will be used to save data if the board
 * loose connection with the MQTT broker, and send data saved as soon as the
 * connection is restored.
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

#include "contiki.h"
#include "sys/etimer.h"
#include "ext-flash.h"
#include "lib/crc16.h"
#include "lib/memb.h"
#include "flash-drive.h"

/******************
 * Global variables
 ******************/

static bool was_flash_initialized = false;

static ct_main_header_t *main_header;
static ct_block_header_t *block_header;

/********************
 * Internal functions
 ********************/

static void set_bit_on_map(bitmap_word_t *bitmap, int n) {
   bitmap[WORD_OFFSET(n)] |= (1 << BIT_OFFSET(n));
}

static void clear_bit_on_map(bitmap_word_t *bitmap, int n) {
   bitmap[WORD_OFFSET(n)] &= ~(1 << BIT_OFFSET(n));
}

static int get_bit_on_map(bitmap_word_t *bitmap, int n) {
   bitmap_word_t bit = bitmap[WORD_OFFSET(n)] & (1 << BIT_OFFSET(n));
   return bit != 0;
}

static ct_main_header_t* malloc_ct_main_header_t() {
   return memb_alloc(&alloc_ct_main_header);
}

static fd_main_header_t* malloc_fd_main_header_t() {
   return memb_alloc(&alloc_fd_main_header);
}

static ct_block_header_t* malloc_ct_block_reader_t() {
   return memb_alloc(&alloc_ct_block_header);
}

static fd_block_header_t* malloc_fd_block_reader_t() {
   return memb_alloc(&alloc_fd_block_header);
}

static ct_data_t* malloc_ct_data_t() {
   return memb_alloc(&alloc_ct_data);
}

static fd_data_t* malloc_fd_data_t() {
   return memb_alloc(&alloc_fd_data);
}

static void free_ct_main_header_t(ct_main_header_t *p) {
   memb_free(&alloc_ct_main_header, p);
}

static void free_fd_main_header_t(fd_main_header_t *p) {
   memb_free(&alloc_fd_main_header, p);
}

static void free_ct_block_reader_t(ct_block_header_t *p) {
   memb_free(&alloc_ct_block_header, p);
}

static void free_fd_block_reader_t(fd_block_header_t *p) {
   memb_free(&alloc_fd_block_header, p);
}

static void free_ct_data_t(ct_data_t *p) {
   memb_free(&alloc_ct_data, p);
}

static void free_fd_data_t(fd_data_t *p) {
   memb_free(&alloc_fd_data, p);
}

static uint32_t get_block_address(uint8_t position) {
   return position * BLOCK_SIZE;
}

static uint32_t get_sector_address(uint8_t position, uint8_t sector) {
   return (position * BLOCK_SIZE) + (sector * SECTOR_SIZE);
}

static uint8_t flash_open() {
   return ext_flash_open() ? FLASH_OPERATION_SUCCESSFUL : FLASH_OPERATION_ERROR_OPEN;
}

static uint8_t flash_close() {
   ext_flash_close();
   return FLASH_OPERATION_SUCCESSFUL;
}

static uint8_t read_main_header(ct_main_header_t *header) {
   static uint8_t result = FLASH_OPERATION_SUCCESSFUL;
   fd_main_header_t *header_container = malloc_fd_main_header_t();
   if (ext_flash_read(MAIN_HEADER_PRIMARY_ADDR, sizeof(fd_main_header_t),
                      (uint8_t *) header_container)) {
      memcpy(header, &(header_container->content), sizeof(ct_main_header_t));
      if (strcmp(header->tag, TAG_MAIN_HEADER) == 0) {
         unsigned short crc = crc16_data((unsigned char*) header,
                                         sizeof(ct_main_header_t), 0);
         if (crc != header_container->crc) {
            result = FLASH_OPERATION_ERROR_CRC;
         }
      } else {
         result = FLASH_OPERATION_NO_DATA;
      }
   } else {
      result = FLASH_OPERATION_ERROR_READING;
   }
   if (result != FLASH_OPERATION_SUCCESSFUL) {
      result = FLASH_OPERATION_SUCCESSFUL_BACKUP;
      if (ext_flash_read(MAIN_HEADER_BACKUP_ADDR, sizeof(fd_main_header_t),
                         (uint8_t *) header_container)) {
         memcpy(header, &(header_container->content), sizeof(ct_main_header_t));
         if (strcmp(header->tag, TAG_MAIN_HEADER) == 0) {
            unsigned short crc = crc16_data((unsigned char*) header,
                                            sizeof(ct_main_header_t), 0);
            if (crc != header_container->crc) {
               result = FLASH_OPERATION_ERROR_CRC;
            }
         } else {
            result = FLASH_OPERATION_NO_DATA;
         }
      } else {
         result = FLASH_OPERATION_ERROR_READING;
      }
   }
   free_fd_main_header_t(header_container);
   return result;
}

static uint8_t read_block_header(uint8_t position, ct_block_header_t *header) {
   if ((position < DATA_BLOCKS_START_POSITION) || (position > DATA_BLOCKS_END_POSITION)) {
      return FLASH_OPERATION_INVALID_BLOCK_POS;
   }
   uint8_t result = FLASH_OPERATION_SUCCESSFUL;
   fd_block_header_t *header_container = malloc_fd_block_reader_t();
   if (ext_flash_read(get_block_address(position),
                      sizeof(fd_block_header_t),
                      (uint8_t *) header_container)) {
      memcpy(header, &(header_container->content), sizeof(ct_block_header_t));
      if (strcmp(header->tag, TAG_BLOCK_HEADER) == 0) {
         unsigned short crc = crc16_data((unsigned char*) header,
                                         sizeof(ct_block_header_t), 0);
         if (crc != header_container->crc) {
            result = FLASH_OPERATION_ERROR_CRC;
         }
      } else {
         result = FLASH_OPERATION_NO_DATA;
      }
   } else {
      result = FLASH_OPERATION_ERROR_READING;
   }
   free_fd_block_reader_t(header_container);
   return result;
}

static uint8_t read_data(uint8_t position, uint8_t sector, ct_data_t *data) {
   if ((position < DATA_BLOCKS_START_POSITION) || (position > DATA_BLOCKS_END_POSITION)) {
      return FLASH_OPERATION_INVALID_BLOCK_POS;
   }
   if ((sector < DATA_SECTORS_START_POSITION) || (sector > DATA_SECTORS_END_POSITION)) {
      return FLASH_OPERATION_INVALID_SECTOR_POS;
   }
   uint8_t result = FLASH_OPERATION_SUCCESSFUL;
   fd_data_t *data_container = malloc_fd_data_t();
   if (ext_flash_read(get_sector_address(position, sector),
                      sizeof(fd_data_t), (uint8_t *) data_container)) {
      memcpy(data, &(data_container->content), sizeof(ct_data_t));
      if (strcmp(data->tag, TAG_STATUS_DATA) == 0) {
         unsigned short crc = crc16_data((unsigned char*) data,
                                         sizeof(ct_data_t), 0);
         if (crc != data_container->crc) {
            result = FLASH_OPERATION_ERROR_CRC;
         }
      } else {
         result = FLASH_OPERATION_NO_DATA;
      }
   } else {
      result = FLASH_OPERATION_ERROR_READING;
   }
   free_fd_data_t(data_container);
   return result;
}

static uint8_t erase_block(uint8_t position) {
   if ((position < MAIN_HEADER_POSITION) || (position > DATA_BLOCKS_END_POSITION)) {
      return FLASH_OPERATION_INVALID_BLOCK_POS;
   }
   uint8_t result = FLASH_OPERATION_SUCCESSFUL;
   if (!ext_flash_erase(get_block_address(position), BLOCK_SIZE)) {
      result = FLASH_OPERATION_ERROR_ERASING;
   }
   return result;
}

static uint8_t write_main_header(ct_main_header_t *header) {
   uint8_t result = FLASH_OPERATION_SUCCESSFUL;
   fd_main_header_t *header_container = malloc_fd_main_header_t();
   strcpy(header->tag, TAG_MAIN_HEADER);
   memcpy(&(header_container->content), header, sizeof(ct_main_header_t));
   header_container->crc = crc16_data((unsigned char*) header,
                                      sizeof(ct_main_header_t), 0);
   if (ext_flash_write(MAIN_HEADER_PRIMARY_ADDR, sizeof(fd_main_header_t),
                       (uint8_t *) header_container)) {
      if (!ext_flash_write(MAIN_HEADER_BACKUP_ADDR, sizeof(fd_main_header_t),
                           (uint8_t *) header_container)) {
         result = FLASH_OPERATION_ERROR_WRITING;
      }
   } else {
      result = FLASH_OPERATION_ERROR_WRITING;
   }
   free_fd_main_header_t(header_container);
   return result;
}

static uint8_t write_block_header(uint16_t position,
         ct_block_header_t *header) {
   if ((position < DATA_BLOCKS_START_POSITION) || (position > DATA_BLOCKS_END_POSITION)) {
      return FLASH_OPERATION_INVALID_BLOCK_POS;
   }
   uint8_t result = FLASH_OPERATION_SUCCESSFUL;
   fd_block_header_t *header_container = malloc_fd_block_reader_t();
   strcpy(header->tag, TAG_BLOCK_HEADER);
   memcpy(&(header_container->content), header, sizeof(ct_block_header_t));
   header_container->crc = crc16_data((unsigned char*) header,
                                      sizeof(ct_block_header_t), 0);
   if (!ext_flash_write(get_block_address(position),
                        sizeof(fd_block_header_t),
                        (uint8_t *) header_container)) {
      result = FLASH_OPERATION_ERROR_WRITING;
   }
   free_fd_block_reader_t(header_container);
   return result;
}

static uint8_t write_data(uint16_t position, uint16_t sector, ct_data_t *data) {
   if ((position < DATA_BLOCKS_START_POSITION) || (position > DATA_BLOCKS_END_POSITION)) {
      return FLASH_OPERATION_INVALID_BLOCK_POS;
   }
   if ((sector < DATA_SECTORS_START_POSITION) || (sector > DATA_SECTORS_END_POSITION)) {
      return FLASH_OPERATION_INVALID_SECTOR_POS;
   }
   uint8_t result = FLASH_OPERATION_SUCCESSFUL;
   fd_data_t *data_container = malloc_fd_data_t();
   strcpy(data->tag, TAG_STATUS_DATA);
   memcpy(&(data_container->content), data, sizeof(ct_data_t));
   data_container->crc = crc16_data((unsigned char*) data, sizeof(ct_data_t),
                                    0);
   if (!ext_flash_write(get_sector_address(position, sector),
                        sizeof(fd_data_t), (uint8_t *) data_container)) {
      result = FLASH_OPERATION_ERROR_WRITING;
   }
   free_fd_data_t(data_container);
   return result;
}

static void reset_main_header() {
   strcpy(main_header->tag, TAG_MAIN_HEADER);
   main_header->init_data_block = 1;
   main_header->next_free_block = 1;
   static uint8_t i, j;
   for (i = 0; i < MAIN_HEADER_MAP_BLOCKS_PAGES; i++) {
      for (j = 0; j < BITMAP_SIZE_IN_BITS; j++) {
         clear_bit_on_map(&(main_header->block_map[i]), j);
      }
   }
}

static void reset_block_header() {
   strcpy(block_header->tag, TAG_BLOCK_HEADER);
   block_header->init_data_sector = 1;
   block_header->next_free_sector = 1;
   static uint8_t i;
   for (i = 0; i < BITMAP_SIZE_IN_BITS; i++) {
      clear_bit_on_map(&(block_header->sector_map), i);
   }
}

static void mark_current_sector_as_bad() {
   set_bit_on_map(&(block_header->sector_map), block_header->next_free_sector);
   static uint8_t i, counter = 0;
   for (i = DATA_SECTORS_START_POSITION; i <= DATA_SECTORS_END_POSITION; i++) {
      if (get_bit_on_map(&(block_header->sector_map), i) == BAD) {
         counter++;
      }
   }
   if (counter == (DATA_SECTORS_END_POSITION - DATA_SECTORS_START_POSITION + 1)) {
      set_bit_on_map(
               &(main_header->block_map[(uint8_t) (main_header->next_free_block
                        / MAIN_HEADER_MAP_BLOCKS_PER_PAGES)]),
               (main_header->next_free_block % MAIN_HEADER_MAP_BLOCKS_PER_PAGES));
   }
}

static bool increase_to_next_sector(bool mark_bad_block) {
   if (mark_bad_block) {
      mark_current_sector_as_bad();
   }
   static uint8_t next_block = main_header->next_free_block;
   static uint8_t next_sector = block_header->next_free_sector + 1;
   static uint8_t i, j;
   bool found_next = false;
   // TODO Terminar
   for (i = next_block; i <= DATA_BLOCKS_END_POSITION; i++) {
      bool block_changed = false;
      for (j = next_sector; j <= DATA_SECTORS_END_POSITION; j++) {
         if (get_bit_on_map(&(block_header->sector_map), j) == GOOD) {
            next_sector = j;
            found_next = true;
            break;
         }
      }
      if (found_next) {
         next_block = i;
         break;
      }
      next_sector = DATA_SECTORS_START_POSITION;
   }
   if (found_next) {
      main_header->next_free_block = next_block ;
      block_header->next_free_sector = next_sector;
   }
   return found_next;
}

static uint8_t save_status(uint8_t status_type, uint8_t topic_index, char status_data[64]) {
   uint8_t result = FLASH_OPERATION_SUCCESSFUL;
   ct_data_t data = malloc_ct_data_t();
   strcpy(data->tag, TAG_STATUS_DATA);
   data->status_type = status_type;
   if (status_type == STATUS_TYPE_BOARD) {
      data->topic_index = 0;
   } else {
      data->topic_index = topic_index;
   }
   strcpy(data->status_data, status_data);
   static uint8_t i;
   for (i = 0; i < SAVING_ATTEMPTS; i++) {
      result = write_data(main_header->next_free_block, block_header->next_free_sector, data);
      if (result == FLASH_OPERATION_SUCCESSFUL) {
         break;
      } else if (result == FLASH_OPERATION_ERROR_WRITING) {
         if (!increase_to_next_sector(true) {
            break;
         }
      }
   }
   if (result != FLASH_OPERATION_SUCCESSFUL) {

   }

   return result;
}

/************************************
 * Interface functions implementation
 ************************************/

static uint8_t initialize_flash_drive() {
   uint8_t result = FLASH_OPERATION_SUCCESSFUL;
   if (!was_flash_initialized) {
      result = flash_open();
      if (result == FLASH_OPERATION_SUCCESSFUL) {
         main_header = malloc_ct_main_header_t();
         result = read_main_header(main_header);
         if (result == FLASH_OPERATION_NO_DATA) {
            reset_main_header();
            result = write_main_header(main_header);
         }
         if (result == FLASH_OPERATION_SUCCESSFUL) {
            block_header = malloc_ct_block_reader_t();
            result = read_block_header(main_header->next_free_block, block_header);
            if (result == FLASH_OPERATION_SUCCESSFUL) {
               result = true;
            } else if (result == FLASH_OPERATION_NO_DATA) {
               reset_block_header();
               result = write_block_header(main_header->next_free_block, block_header);
            }
         }
      }
      if (result == FLASH_OPERATION_SUCCESSFUL) {
         was_flash_initialized = true;
      }
   }
   return result;
}

static uint8_t finalize_flash_drive() {
   uint8_t result = FLASH_OPERATION_SUCCESSFUL;
   if (was_flash_initialized) {
      was_flash_initialized = false;
      free_ct_block_reader_t(block_header);
      free_ct_main_header_t(main_header);
      result = flash_close();
   }
   return result;
}

static uint8_t save_board_status(char status_data[64]) {
   return save_status(STATUS_TYPE_BOARD, 0, status_data);
}

static uint8_t save_sensor_status(uint8_t topic_index, char status_data[64]) {
   return save_status(STATUS_TYPE_SENSORS, topic_index, status_data);
}

static bool load_board_status(ct_data_t *status_data) {
   uint8_t result = FLASH_OPERATION_SUCCESSFUL;
   return result;
}

