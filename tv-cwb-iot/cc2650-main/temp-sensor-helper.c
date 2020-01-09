/**
 * File:   temp-sensor-helper.c
 * Author: Daniel Carvalho Dias (daniel.dias@gmail.com)
 * Date:   12/07/2019
 *
 * Implements the temp-sensor-helper.h interface to configure and read from
 * temperature sensors used by the CC2650 board.
 *
 * The temperature sensor DS18b20 uses a GPIO port and the reading will be
 * performed by pooling.
 *
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "contiki.h"
#include "ti-lib.h"
#include "ioc.h"

#include "temp-sensor-helper.h"

/******************************************************************************
 * Internal functions definitions
 ******************************************************************************/

void OW_SET_OUTPUT() {
   ti_lib_rom_ioc_pin_type_gpio_output(DS18B20_PORT);
   ti_lib_gpio_write_dio(DS18B20_PORT, 0);
}

void OW_SET_INPUT() {
   ti_lib_rom_ioc_pin_type_gpio_input(DS18B20_PORT);
   IOCIOPortPullSet(DS18B20_PORT, IOC_IOPULL_UP); //IOC_IOPULL_UP IOC_NO_IOPULL IOC_IOPULL_DOWN
}

void write_bit(uint8_t bit) {
   OW_SET_OUTPUT();
   clock_delay_usec(2);
   if (bit) {
      OW_SET_INPUT();
   }
   clock_delay_usec(60);
   OW_SET_INPUT();
   clock_delay_usec(2);
}

uint8_t read_bit(void) {
   uint8_t bit = 0;

   OW_SET_OUTPUT();

   clock_delay_usec(2);

   OW_SET_INPUT();
   clock_delay_usec(2);
   bit = GPIO_readDio(DS18B20_PORT); //OW_GET_PIN_STATE()
   clock_delay_usec(50);
   OW_SET_INPUT();

   clock_delay_usec(2);

   return bit ? 1 : 0;
}

uint8_t read_byte(void) {
   uint8_t result = 0;
   uint8_t bit;
   int i;

   for (i = 0; i < 8; i++) {
      bit = read_bit();
      result += (bit << i);
   }

   return result;
}

void write_byte(uint8_t byte) {
   int i;
   for (i = 0; i < 8; i++) {
      write_bit((byte >> i) & 1);
   }
}

uint8_t crc8_ds18b20(uint8_t *buf, uint8_t buf_len) {
   uint8_t result = 0;
   uint8_t i, b;

   for (i = 0; i < buf_len; i++) {
      result = result ^ buf[i];
      for (b = 1; b < 9; b++) {
         if (result & 0x1) {
            result = (result >> 1) ^ 0x8C;
         } else {
            result = result >> 1;
         }
      }
   }
   return result;
}

/******************************************************************************
 * Interface functions definitions
 ******************************************************************************/

uint8_t ds18b20_probe(void) {
   uint8_t result = 0;
   OW_SET_OUTPUT();
   clock_delay_usec(480);
   OW_SET_INPUT();
   clock_delay_usec(64);
   result = !GPIO_readDio(DS18B20_PORT); //OW_GET_PIN_STATE()
   if (result) {
      clock_delay_usec(300);
      result = GPIO_readDio(DS18B20_PORT); //OW_GET_PIN_STATE()
   }
   return result;
}

uint8_t ds18b20_get_temp(float *temp1) {
   uint8_t result = 0;
   float temp_c;
   if (ds18b20_probe()) {
      write_byte(DS18B20_COMMAND_SKIP_ROM);
      write_byte(DS18B20_COMMAND_START_CONVERSION);
      OW_SET_INPUT();
      int count = 0;
      while (!GPIO_readDio(DS18B20_PORT)) {
         clock_delay_usec(10000); //clock_delay_msec(10);
         count++;
         if (count > 80) {
            return 0;
         }
      }

      (void) ds18b20_probe();
      write_byte(DS18B20_COMMAND_SKIP_ROM);
      write_byte(DS18B20_COMMAND_READ_SCRATCH_PAD);
      uint8_t i;
      uint8_t sp_arr[9];
      for (i = 0; i < 9; i++) {
         sp_arr[i] = read_byte();
      }

      uint8_t crc_cal = crc8_ds18b20(sp_arr, 8);
      if (crc_cal != sp_arr[8]) {
         return 0;
      }
      uint16_t temp_digital = sp_arr[0] | sp_arr[1] << 8; //make integer
      if (temp_digital >= 0x800) {  //temperture is negative
         temp_c = 0;
         if (temp_digital & 0x0001)
            temp_c += 0.06250;
         if (temp_digital & 0x0002)
            temp_c += 0.12500;
         if (temp_digital & 0x0004)
            temp_c += 0.25000;
         if (temp_digital & 0x0008)
            temp_c += 0.50000;

         temp_digital = (temp_digital >> 4) & 0x00FF;
         temp_digital = temp_digital - 0x0001; //subtract 1
         temp_digital = ~temp_digital; //ones compliment
         temp_c = temp_c - (float) (temp_digital & 0xFF);
      } else { //temperture is positive
         temp_c = 0;
         temp_c = (temp_digital >> 4) & 0x00FF;
         if (temp_digital & 0x0001)
            temp_c = temp_c + 0.06250;
         if (temp_digital & 0x0002)
            temp_c = temp_c + 0.12500;
         if (temp_digital & 0x0004)
            temp_c = temp_c + 0.25000;
         if (temp_digital & 0x0008)
            temp_c = temp_c + 0.50000;
      }
      *temp1 = temp_c;

      result = 1;
   }
   return result;
}
