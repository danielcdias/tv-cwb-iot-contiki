/**
 * File:   util.h
 * Author: Daniel Carvalho Dias (daniel.dias@gmail.com)
 * Date:   11/05/2019
 *
 * Defines interface to configure and read from all sensors used by the CC2650 board.
 *
 * All 5 rain sensors will use a GPIO port and the reading will be performed by pooling.
 * The pluviometer will use a interruption event, just like the button implementation
 * for the CC2650 board.
 * The moisture sensor will be used as an ADC sensor.
 *
 */

#include "board.h"
#include "dev/adc-sensor.h"
#include "lib/sensors.h"

#ifndef UTIL_H_
#define UTIL_H_

/**************************************************************************
 * Definitions of port numbers and reading intervals
 **************************************************************************

Constant for ports are:

> Digital IO ports:
RAIN_SENSOR_           Rain Sensor
  RAIN_SENSOR_SURFACE_ Rain Sensor placed in model surface
  RAIN_SENSOR_DRAIN    Rain Sensor placed in model drain
PLUVIOMETER_SENSOR     PLuviometer

> ADC ports:
MOISTURE_SENSOR        soil Moisture Sensor

*/
#define RAIN_SENSOR_SURFACE_1 IOID_25
#define RAIN_SENSOR_SURFACE_2 IOID_26
#define RAIN_SENSOR_SURFACE_3 IOID_27
#define RAIN_SENSOR_SURFACE_4 IOID_28

#define RAIN_SENSOR_DRAIN IOID_29

#define PLUVIOMETER_SENSOR IOID_23

#define MOISTURE_SENSOR ADC_COMPB_IN_AUXIO0 // DIO 30

#define RAIN_SENSORS_READ_INTERVAL 0.1 // seconds

#define MOISTURE_SENSOR_READ_INTERVAL 1 // seconds


/***********************
 * Functions definitions
 ***********************/
void configureGPIOSensors();

int readADSMoistureSensor();

u_int32_t readGPIOSensor(u_int32_t dio);

#endif
