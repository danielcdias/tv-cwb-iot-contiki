/**
 * File:   test-sensor.c
 * Author: Daniel Carvalho Dias (daniel.dias@gmail.com)
 * Date:   11/05/2019
 *
 * Tests all sensors that can be connected in the CC2650 board, printing results in the serial output.
 *
 */

#include <stdio.h>

#include "contiki.h"
#include "sys/etimer.h"
#include "lib/sensors.h"
#include "button-sensor.h"

#include "util.h"
#include "pluv-sensor.h"

/******************
 * Global variables
 ******************/

static struct etimer et_rainSensors, et_pluviometer, et_moistureSensor;

/**********************
 * Processes definition
 **********************/

PROCESS(testRainSensors, "testRainSensors");
PROCESS(testPluviometerSensor, "testPluviometerSensor");
PROCESS(testMoistureSensor, "testMoistureSensor");
AUTOSTART_PROCESSES(&testRainSensors, &testPluviometerSensor, &testMoistureSensor);
//AUTOSTART_PROCESSES(&testRainSensors);
//AUTOSTART_PROCESSES(&testPluviometerSensor);
//AUTOSTART_PROCESSES(&testMoistureSensor);
//AUTOSTART_PROCESSES(&testRainSensors,&testPluviometerSensor);

/**************************
 * Processes implementation
 **************************/

PROCESS_THREAD(testRainSensors, ev, data) {
    PROCESS_BEGIN();

    etimer_set(&et_rainSensors, 4 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && data == &et_rainSensors);

    printf("#1# ---------- Starting Rain Sensors test...\n");

    static int lastValueRead1, lastValueRead2, lastValueRead3, lastValueRead4, lastValueRead5;

    configureGPIOSensors();
    while (1) {
        static int valueRead;
        valueRead = readGPIOSensor(RAIN_SENSOR_SURFACE_1);
        if (valueRead != lastValueRead1) {
            printf("#1# Rain sensor surface 1 - Value changed = %d\n", valueRead);
            lastValueRead1 = valueRead;
        }
        valueRead = readGPIOSensor(RAIN_SENSOR_SURFACE_2);
        if (valueRead != lastValueRead2) {
            printf("#1# Rain sensor surface 2 - Value changed = %d\n", valueRead);
            lastValueRead2 = valueRead;
        }
        valueRead = readGPIOSensor(RAIN_SENSOR_SURFACE_3);
        if (valueRead != lastValueRead3) {
            printf("#1# Rain sensor surface 3 - Value changed = %d\n", valueRead);
            lastValueRead3 = valueRead;
        }
        valueRead = readGPIOSensor(RAIN_SENSOR_SURFACE_4);
        if (valueRead != lastValueRead4) {
            printf("#1# Rain sensor surface 4 - Value changed = %d\n", valueRead);
            lastValueRead4 = valueRead;
        }
        valueRead = readGPIOSensor(RAIN_SENSOR_DRAIN);
        if (valueRead != lastValueRead5) {
            printf("#1# Rain sensor drain - Value changed = %d\n", valueRead);
            lastValueRead5 = valueRead;
        }
        etimer_set(&et_rainSensors, RAIN_SENSORS_READ_INTERVAL * CLOCK_SECOND);
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && data == &et_rainSensors);
    }

    PROCESS_END();
}

PROCESS_THREAD(testPluviometerSensor, ev, data) {
    PROCESS_BEGIN();

    etimer_set(&et_pluviometer, 4 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && data == &et_pluviometer);

    printf("#2# ---------- Starting pluviometer sensor test...\n");
    SENSORS_ACTIVATE(pluviometer_sensor);

    while(1) {
        PROCESS_YIELD();

        if(ev == sensors_event) {
            if(data == &pluviometer_sensor) {
                printf("#2# Pluviometer Sensor event received!\n");
            }
        }
    }

    SENSORS_DEACTIVATE(pluviometer_sensor);
    PROCESS_END();
}

PROCESS_THREAD(testMoistureSensor, ev, data) {
    PROCESS_BEGIN();

    etimer_set(&et_moistureSensor, 4 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && data == &et_moistureSensor);

    printf("#3# ---------- Starting Moisture Sensors (ADC) test...\n");

    while (1) {

        printf("#3# Moisture sensor reading at pin %i - value read: %i\n", MOISTURE_SENSOR, readADSMoistureSensor());

        etimer_set(&et_moistureSensor, MOISTURE_SENSOR_READ_INTERVAL * CLOCK_SECOND);
        PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && data == &et_moistureSensor);
    }

    PROCESS_END();
}
