/**
 * File:  flash-drive.h
 * Author: Daniel Carvalho Dias (daniel.dias@gmail.com)
 * Date:   05/08/2019
 *
 * Defines an interface to save and load data to the external flash drive of
 * CC2650 control board. The flash drive will be used to save data if the board
 * loose connection with the MQTT broker, and send data saved as soon as the
 * connection is restored.
 */

#include <limits.h>
#include <stdint.h>

#include "contiki.h"
#include "lib/memb.h"

/***********************
 * Structs and constants
 ***********************/

#define FLASH_OPERATION_SUCCESSFUL 0
#define FLASH_OPERATION_SUCCESSFUL_BACKUP 1
#define FLASH_OPERATION_ERROR_OPEN 2
#define FLASH_OPERATION_ERROR_WRITING 3
#define FLASH_OPERATION_ERROR_READING 4
#define FLASH_OPERATION_ERROR_CRC 5
#define FLASH_OPERATION_ERROR_ERASING 6
#define FLASH_OPERATION_NO_DATA 7
#define FLASH_OPERATION_INVALID_BLOCK_POS 8
#define FLASH_OPERATION_INVALID_SECTOR_POS 9

#define BLOCK_SIZE 4096 // 4 KBytes
#define SECTOR_SIZE 256 // Bytes

#define MAIN_HEADER_POSITION 0

#define MAIN_HEADER_PRIMARY_ADDR ((BLOCK_SIZE * MAIN_HEADER_POSITION) + 0)
#define MAIN_HEADER_BACKUP_ADDR ((BLOCK_SIZE * MAIN_HEADER_POSITION) + 256)

#define DATA_BLOCKS_START_POSITION 1
#define DATA_BLOCKS_END_POSITION 255

#define DATA_SECTORS_START_POSITION 1
#define DATA_SECTORS_END_POSITION 15

#define STATUS_TYPE_BOARD 0
#define STATUS_TYPE_SENSORS 1

#define MAIN_HEADER_MAP_BLOCKS_PAGES 8
#define MAIN_HEADER_MAP_BLOCKS_PER_PAGES 32

#define GOOD 0
#define BAD 1

#define SAVING_ATTEMPTS 5

#define TAG_MAIN_HEADER "TMH\0"
#define TAG_BLOCK_HEADER "TSH\0"
#define TAG_STATUS_DATA "TSD\0"

typedef uint32_t bitmap_word_t;

#define BITMAP_SIZE_IN_BITS (sizeof(bitmap_word_t) * CHAR_BIT)

enum { BITS_PER_WORD = BITMAP_SIZE_IN_BITS };

#define WORD_OFFSET(b) ((b) / BITS_PER_WORD)
#define BIT_OFFSET(b)  ((b) % BITS_PER_WORD)

typedef struct _ct_main_header_t {
   char tag[4];
   uint8_t init_data_block;
   uint8_t next_free_block;
   bitmap_word_t block_map[MAIN_HEADER_MAP_BLOCKS_PAGES];
} ct_main_header_t;
MEMB(alloc_ct_main_header, ct_main_header_t, 2);
typedef struct _fd_main_header_t {
   ct_main_header_t content;
   unsigned short crc;
} fd_main_header_t;
MEMB(alloc_fd_main_header, fd_main_header_t, 2);

typedef struct _ct_block_header_t {
   char tag[4];
   uint8_t init_data_sector;
   uint8_t next_free_sector;
   bitmap_word_t sector_map;
} ct_block_header_t;
MEMB(alloc_ct_block_header, ct_block_header_t, 2);
typedef struct _fd_block_header_t {
   ct_block_header_t content;
   unsigned short crc;
} fd_block_header_t;
MEMB(alloc_fd_block_header, fd_block_header_t, 2);

typedef struct _ct_data_t {
   char tag[4];
   uint8_t status_type;
   uint8_t topic_index;
   char status_data[64];
} ct_data_t;
MEMB(alloc_ct_data, ct_data_t, 2);
typedef struct _fd_data_t {
   ct_data_t content;
   unsigned short crc;
} fd_data_t;
MEMB(alloc_fd_data, fd_data_t, 2);

/**************************************
 * Function definitions for flash drive
 **************************************/

static uint8_t initialize_flash_drive();

static uint8_t finalize_flash_drive();

static uint8_t save_board_status(char status_data[64]);

static uint8_t save_sensor_status(uint8_t topic_index, char status_data[64]);

static uint8_t load_board_status(ct_data_t *status_data);
