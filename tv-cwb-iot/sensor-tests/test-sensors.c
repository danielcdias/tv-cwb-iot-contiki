/**
 * File:   test-sensor.c
 * Author: Daniel Carvalho Dias (daniel.dias@gmail.com)
 * Date:   11/05/2019
 *
 * Tests all sensors that can be connected in the CC2650 board, printing results in the serial output.
 *
 */

#include <stdio.h>
#include <stdbool.h>

#include "contiki.h"
#include "sys/etimer.h"
#include "lib/sensors.h"
#include "button-sensor.h"

#include "sensors-helper.h"
#include "temp-sensor-helper.h"
#include "interruption-sensor.h"

/******************
 * Global variables
 ******************/

static struct etimer et_rainCapacitiveDrainSensor, et_rainOpticSensor, et_pluviometer, et_moistureSensor, et_temperatureSensor;

/**********************
 * Processes definition
 **********************/

PROCESS(testRainCapacitiveDrainSensor, "testRainCapacitiveDrainSensor");
PROCESS(testRainOpticalSensor, "testRainOpticalSensor");
PROCESS(testPluviometerSensor, "testPluviometerSensor");
PROCESS(testMoistureSensor, "testMoistureSensor");
PROCESS(testTemperatureSensor, "testTemperatureSensor");
AUTOSTART_PROCESSES(&testRainCapacitiveDrainSensor, &testRainOpticalSensor, &testPluviometerSensor, &testMoistureSensor, &testTemperatureSensor);
//AUTOSTART_PROCESSES(&testRainCapacitiveDrainSensor);
//AUTOSTART_PROCESSES(&testPluviometerSensor);
//AUTOSTART_PROCESSES(&testRainOpticalSensor);
//AUTOSTART_PROCESSES(&testMoistureSensor);
//AUTOSTART_PROCESSES(&testTemperatureSensor);
//AUTOSTART_PROCESSES(&testRainCapacitiveDrainSensor, &testRainOpticalSensor, &testTemperatureSensor);

/**************************
 * Processes implementation
 **************************/

PROCESS_THREAD(testRainCapacitiveDrainSensor, ev, data) {
   PROCESS_BEGIN();

   etimer_set(&et_rainCapacitiveDrainSensor, 4 * CLOCK_SECOND);
   PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && data == &et_rainCapacitiveDrainSensor);

   printf("# %s # ---------- Starting Rain on Drain Sensor test...\n", SENSOR_RAIN_DRAIN);

   static bool is_raining = false, previous = false;
   static uint32_t value_read = 0;

   while (true) {

      value_read = readADSSensor(RAIN_ON_DRAIN_SENSOR);
      //printf(">>> Valeu read: %lu\n", value_read);
      is_raining = (value_read < RAIN_ON_DRAIN_DETECTION_MAX_VALUE);
      if (previous ^ is_raining) {
         previous = is_raining;
         printf((is_raining ? "# %s # Water on drain DETECTED!\n" : "# %s # Water on drain detection STOPPED!\n"), SENSOR_RAIN_DRAIN);
      }

      etimer_set(&et_rainCapacitiveDrainSensor, 1 * CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && data == &et_rainCapacitiveDrainSensor);
   }

   PROCESS_END();
}

PROCESS_THREAD(testRainOpticalSensor, ev, data) {
    PROCESS_BEGIN();

    etimer_set(&et_rainOpticSensor, 4 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && data == &et_rainOpticSensor);

    if (readGPIOSensor(JUMPER_PLUVIOMETER_INSTALLED) != PLUVIOMETER_INSTALLED) {

       printf("# %s # ---------- Starting Optical Rain sensor test...\n", SENSOR_OPTICAL_RAIN);
       SENSORS_ACTIVATE(interruption_sensor);

       static uint32_t counter = 0;

       while (true) {
           PROCESS_YIELD();

           if(ev == sensors_event) {
               if(data == &interruption_sensor) {
                  counter++;
                  printf("# %s # Optical Reain sensor received an event. Total: %lu\n", SENSOR_OPTICAL_RAIN, counter);
               }
           }
       }

       SENSORS_DEACTIVATE(interruption_sensor);

    }
    PROCESS_END();
}

PROCESS_THREAD(testPluviometerSensor, ev, data) {
    PROCESS_BEGIN();

    etimer_set(&et_pluviometer, 4 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && data == &et_pluviometer);

    if (readGPIOSensor(JUMPER_PLUVIOMETER_INSTALLED) == PLUVIOMETER_INSTALLED) {

       printf("# %s # ---------- Pluviometric sensor test...\n", SENSOR_OPTICAL_RAIN);
       SENSORS_ACTIVATE(interruption_sensor);

       static uint32_t counter = 0;

       while (true) {
           PROCESS_YIELD();

           if(ev == sensors_event) {
               if(data == &interruption_sensor) {
                  counter++;
                  printf("# %s # Pluviometric sensor received an event. Total: %lu\n", SENSOR_OPTICAL_RAIN, counter);
               }
           }
       }

       SENSORS_DEACTIVATE(interruption_sensor);

    }

    PROCESS_END();
}

PROCESS_THREAD(testMoistureSensor, ev, data) {
    PROCESS_BEGIN();

    etimer_set(&et_moistureSensor, 4 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && data == &et_moistureSensor);

    printf("# %s # ---------- Soil Moisture sensor test...\n", SENSOR_CAPACITIVE_SOIL_MOISTURE);

    while (true) {

        printf("# %s # Moisture sensor reading - value read: %lu\n", SENSOR_CAPACITIVE_SOIL_MOISTURE, readADSSensor(MOISTURE_SENSOR));

        etimer_set(&et_moistureSensor, MOISTURE_TEMP_READ_INTERVAL * CLOCK_SECOND);
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && data == &et_moistureSensor);
    }

    PROCESS_END();
}

PROCESS_THREAD(testTemperatureSensor, ev, data) {
    PROCESS_BEGIN();

    etimer_set(&et_temperatureSensor, 4 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && data == &et_temperatureSensor);

    printf("# %s # ---------- Temperature sensor test...\n", SENSOR_TEMPERATURE);

    while (true) {

        printf("# %s # Temperature - value read: %i\n", SENSOR_TEMPERATURE, readTemperatureSensor(TEMPERATURE_SENSOR));

        etimer_set(&et_temperatureSensor, MOISTURE_TEMP_READ_INTERVAL * CLOCK_SECOND);
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && data == &et_temperatureSensor);
    }

    PROCESS_END();
}
