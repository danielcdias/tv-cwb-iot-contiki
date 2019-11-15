/**
 * File:   util.c
 * Author: Daniel Carvalho Dias (daniel.dias@gmail.com)
 * Date:   11/05/2019
 *
 * Implements the util.h interface to configure and read from all sensors used by the CC2650 board.
 *
 * All 5 rain sensors will use a GPIO port and the reading will be performed by pooling.
 * The pluviometer will use a interruption event, just like the button implementation
 * for the CC2650 board.
 * The moisture sensor will be used as an ADC sensor.
 *
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

#include "util.h"

/******************
 * Global variables
 ******************/
static struct sensors_sensor *sensor;

/**
 * util.h header functions implementation
 */
void configureGPIOSensors() {
  IOCPinTypeGpioInput(RAIN_SENSOR_SURFACE_1);
  IOCPinTypeGpioInput(RAIN_SENSOR_SURFACE_2);
  IOCPinTypeGpioInput(RAIN_SENSOR_SURFACE_3);
  IOCPinTypeGpioInput(RAIN_SENSOR_SURFACE_4);
  IOCPinTypeGpioInput(RAIN_SENSOR_DRAIN);
}

int readADSMoistureSensor() {
  sensor = sensors_find(ADC_SENSOR);
  static int valueRead;
  SENSORS_ACTIVATE(*sensor);
  sensor->configure(ADC_SENSOR_SET_CHANNEL, MOISTURE_SENSOR);
  valueRead = (sensor->value(ADC_SENSOR_VALUE));
  SENSORS_DEACTIVATE(*sensor);
  return valueRead;
}

u_int32_t readGPIOSensor(u_int32_t dio) {
  return GPIO_readDio(dio);
}
