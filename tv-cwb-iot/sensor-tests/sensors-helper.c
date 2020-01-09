/**
 * File:   sensors-helper.c
 * Author: Daniel Carvalho Dias (daniel.dias@gmail.com)
 * Date:   28/05/2019
 *
 * Implements the sensors-helper.h interface to configure and read from all
 * sensors used by the CC2650 board.
 *
 * The temperature sensor will use a GPIO port and the reading will be performed by
 * pooling.
 *
 * The moisture sensor and the rain on drain sensor will be used as ADC sensors.
 *
 * The pluviometer and the rain optical sensor will use a interruption event, just
 * like a button's implementation for the CC2650 board.
 * The presence which sensor (pluviometer or rain optioncal) will be defined by a
 * jumper. When ON, indicates the pluviometer is installed, otherwise, the rain
 * optical sensor is installed.
 */

#include <stdio.h>
#include <stdint.h>

#include "board.h"
#include "contiki.h"
#include "dev/adc-sensor.h"
#include "gpio-interrupt.h"
#include "lib/sensors.h"
#include "lpm.h"
#include "sys/timer.h"
#include "ti-lib.h"

#include "sensors-helper.h"
#include "temp-sensor-helper.h"

/******************
 * Global variables
 ******************/
static struct sensors_sensor *sensorMoisture;

static struct timer reading_timer_ADS, reading_timer_tmp;

/**
 * sensors-helper.h header functions implementation
 */
void configureGPIOSensors() {
  IOCPinTypeGpioInput(JUMPER_PLUVIOMETER_INSTALLED);
}

uint32_t internalADSReading(u_int32_t port) {
   static uint32_t valueRead;
   sensorMoisture = sensors_find(ADC_SENSOR);
   SENSORS_ACTIVATE(*sensorMoisture);
   sensorMoisture->configure(ADC_SENSOR_SET_CHANNEL, port);
   valueRead = (uint32_t)(sensorMoisture->value(ADC_SENSOR_VALUE));
   SENSORS_DEACTIVATE(*sensorMoisture);
   return valueRead;
}

uint32_t internalADSReadingAttemps(u_int32_t port) {
   uint32_t result = 0;
   uint32_t readings[] = {0, 0, 0};
   uint8_t i, r = 0;
   bool failed = true;
   for (i = 0; i < MAX_READING_ATTEMPTS_ADS; i++) {
      uint32_t aux = internalADSReading(port);
      if ((aux >= MIN_VALUE_ACCEPTED_ADS) && (aux <= MAX_VALUE_ACCEPTED_ADS)) {
         readings[r] = aux;
         r++;
         if (r == 3) {
            failed = false;
            break;
         }
      }
      timer_set(&reading_timer_ADS, CLOCK_SECOND * 0.05);
      while (!timer_expired(&reading_timer_ADS));
   }
   if (failed) {
      result = 0;
   } else {
      if (r > 0) {
         uint32_t readings_sum = 0;
         for (i = 0; i < r; i++) {
            readings_sum += readings[i];
         }
         result = (uint32_t)(readings_sum / r);
      }
   }
   return result;
}

uint32_t readADSSensor(uint32_t port) {
   return internalADSReadingAttemps(port);
}

int readTemperatureSensor() {
   (void) ds18b20_probe();
   int result = 0;
   float temp;
   uint8_t i;
   for (i = 0; i < MAX_READING_ATTEMPTS_TMP; i++) {
      int ret = ds18b20_get_temp(&temp);
      if (ret) {
         result = (uint32_t)(temp * 100);
         break;
      }
      timer_set(&reading_timer_tmp, CLOCK_SECOND * 0.2);
      while (!timer_expired(&reading_timer_tmp));
   }
   return result;
}

u_int32_t readGPIOSensor(u_int32_t dio) {
  return GPIO_readDio(dio);
}
