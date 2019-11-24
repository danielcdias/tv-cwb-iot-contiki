/**
 * File:   sensors-helper.h
 * Author: Daniel Carvalho Dias (daniel.dias@gmail.com)
 * Date:   28/05/2019
 *
 * Defines interface to configure and read from all sensors used by the CC2650 board.
 *
 * All 5 rain sensors and the temperature sensor will use a GPIO port and the
 * reading will be performed by pooling.
 *
 * The moisture sensor will be used as ADC sensor.
 *
 * The pluviometer will use a interruption event, just like a button's
 * implementation for the CC2650 board.
 * The presence of this sensor will be defined by a jumper. When ON, indicates
 * the pluviometer is installed.
 */

#include "board.h"
#include "dev/adc-sensor.h"
#include "lib/sensors.h"

#ifndef SENSORS_HELP_H_
#define SENSORS_HELP_H_

/******************************************************************************
 * Definitions of port numbers and reading intervals
 ******************************************************************************

Constant for ports are:

> Digital IO ports:
RAIN_SENSOR_           Rain Sensor
  RAIN_SENSOR_SURFACE_ Rain Sensor placed in model surface
  RAIN_SENSOR_DRAIN    Rain Sensor placed in model drain
PLUVIOMETER_SENSOR     Pluviometer

> ADC ports:
MOISTURE_SENSOR        Soil Moisture Sensor
TEMPERATURE_SENSOR     Temperature Sensor

*/

// Surface rain sensors ports (GPIO)
#define RAIN_SENSOR_SURFACE_1 IOID_25
#define RAIN_SENSOR_SURFACE_2 IOID_26
#define RAIN_SENSOR_SURFACE_3 IOID_27
#define RAIN_SENSOR_SURFACE_4 IOID_28

// Drain rain sensors ports  (GPIO)
#define RAIN_SENSOR_DRAIN IOID_29

// Pluviometer port (GPIO)
#define PLUVIOMETER_SENSOR IOID_23

// Moisture sensor port (ADC)
#define MOISTURE_SENSOR ADC_COMPB_IN_AUXIO0 // DIO 30

// Temperature sensor port (GPIO)
#define TEMPERATURE_SENSOR IOID_24

// Pluviometer installed jumper indicator port (GPIO)
#define JUMPER_PLUVIOMETER_INSTALLED IOID_21

// Pluviometer wait interval to reset
#define PLUVIOMETER_WAIT_INTERVAL_TO_RESET 900 // seconds

// Reading interval for rain sensors
#define RAIN_SENSORS_READ_INTERVAL 0.1 // seconds

// Reading intervals for moisture sensor
#define MOISTURE_SENSOR_READ_INTERVAL_NO_RAIN 3600 // seconds
#define MOISTURE_SENSOR_READ_INTERVAL_RAIN 300 // seconds

// Capacitive sensor reading definitions
#define MIN_VALUE_ACCEPTED_ADS 400000
#define MAX_VALUE_ACCEPTED_ADS 1500000
#define MAX_READING_ATTEMPTS_ADS 10

// Temperature sensor reading definitions
#define MAX_READING_ATTEMPTS_TMP 5

// Interval to report rain sensor array information
#define REPORT_RAIN_SENSORS_ARRAY_INTERVAL 2980 // 298 seconds

// Rain sensor possible values
#define RAIN_SENSOR_NOT_RAINING 0
#define RAIN_SENSOR_RAINING 1

// Timeout after sensor stop to alternate values to assume rain stopped
#define RAIN_SENSOR_TIMEOUT_RAIN 300 // 300 seconds

// Minimum number of sensores to detect rain started/ended
#define RAIN_MIN_SENSORS_DETECT_RAIN 2

// Possible valued for pluviometer installed jumper indicator
#define PLUVIOMETER_INSTALLED 1
#define PLUVIOMETER_NOT_INSTALLED 0

// Indicates if board is running in development environment
#define DEV_ENVIRONMENT_JUMPER IOID_22

// Possible valued for dev environment jumper
#define DEV_ENVIRONMENT 1
#define PROD_ENVIRONMENT 0

/******************************** Sensor IDs **********************************/

/* Rain Sensors */
#define SENSOR_RAIN_SURFACE_1 "CS1"
#define SENSOR_RAIN_SURFACE_2 "CS2"
#define SENSOR_RAIN_SURFACE_3 "CS3"
#define SENSOR_RAIN_SURFACE_4 "CS4"
#define SENSOR_RAIN_DRAIN_1 "CRL"

/* Capacitive soil moisture sensor */
#define SENSOR_CAPACITIVE_SOIL_MOISTURE "SCU"

/* Temperature sensor */
#define SENSOR_TEMPERATURE "TMP"

/* Pluviometer sensor */
#define SENSOR_PLUVIOMETER "PLV"

/******************************************************************************
 * Functions definitions
 ******************************************************************************/
void configureGPIOSensors();

uint32_t readADSMoistureSensor();

int readTemperatureSensor();

u_int32_t readGPIOSensor(u_int32_t dio);

#endif
