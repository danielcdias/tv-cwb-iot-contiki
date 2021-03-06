/**
 * File:   sensors-helper.h
 * Author: Daniel Carvalho Dias (daniel.dias@gmail.com)
 * Date:   28/05/2019
 *
 * Defines interface to configure and read from all sensors used by the CC2650 board.
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

#include "board.h"
#include "dev/adc-sensor.h"
#include "lib/sensors.h"
#include "interruption-sensor.h"
#include "temp-sensor-helper.h"

#ifndef SENSORS_HELP_H_
#define SENSORS_HELP_H_

/******************************************************************************
 * Definitions of port numbers and reading intervals
 ******************************************************************************

Constant for ports are:

> Digital IO ports:

OPTICAL_RAIN_SENSOR    Optical Rain Sensor
PLUVIOMETER_SENSOR     Pluviometric Sensor
TEMPERATURE_SENSOR     Temperature Sensor

> ADC ports:
MOISTURE_SENSOR        Capacitive Soil Moisture Sensor
RAIN_ON_DRAIN_SENSOR   Capacitive Soil Moisture Sensor

*/

// Pluviometer or Rain Optical Sensor port (GPIO)
#define PLV_SOC_SENSOR INTERRUPTION_SENSOR_PORT

// Moisture sensor port (ADC)
#define MOISTURE_SENSOR ADC_COMPB_IN_AUXIO6 // DIO 24

// Rain sensor or drain port (ADC)
#define RAIN_ON_DRAIN_SENSOR ADC_COMPB_IN_AUXIO7 // DIO 23

// Temperature sensor port (GPIO)
#define TEMPERATURE_SENSOR DS18B20_PORT

// Pluviometer installed jumper indicator port (GPIO)
#define JUMPER_PLUVIOMETER_INSTALLED IOID_0

// Interruption sensor wait interval to reset
// Pluviometer or Optiocal Rain Sensor
#define INTERRUPTION_SENSOR_WAIT_INTERVAL_TO_RESET 40 // 900 seconds

// Reading interval for rain on drain sensor
#define RAIN_ON_DRAIN_READ_INTERVAL 0.1 // seconds

// Max value read to detect rain. Any value below that indicates rain
// TODO Checar se este valor � correto para qualquer sensor!
#define RAIN_ON_DRAIN_DETECTION_MAX_VALUE 1000000

// Minimum interval to report water detected again for rain on drain
#define MIN_INTERVAL_REPORT_WATER_ON_DRAIN 30 // seconds

// Reading interval for rain on drain sensor
#define READING_INTERVAL_RAIN_ON_DRAIN_SENSOR 0.5 // seconds

// Rain on drain sensor possible values
#define NO_WATER_ON_DRAIN 0
#define WATER_ON_DRAIN 1

// Reading interval for moisture and temperature sensors
#define MOISTURE_TEMP_READ_INTERVAL 30 // 3600 seconds

// Capacitive sensor reading definitions
#define MIN_VALUE_ACCEPTED_ADS 400000
#define MAX_VALUE_ACCEPTED_ADS 1500000
#define MAX_READING_ATTEMPTS_ADS 10

// Temperature sensor reading definitions
#define MAX_READING_ATTEMPTS_TMP 5

// Possible valued for pluviometer installed jumper indicator
#define PLUVIOMETER_INSTALLED 1
#define PLUVIOMETER_NOT_INSTALLED 0

/******************************** Sensor IDs **********************************/

/* Rain on Drain Sensor */
#define SENSOR_RAIN_DRAIN "SCR"

/* Optical Rain Sensor */
#define SENSOR_OPTICAL_RAIN "SOC"

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

uint32_t readADSSensor(uint32_t port);

int readTemperatureSensor();

u_int32_t readGPIOSensor(u_int32_t dio);

#endif
