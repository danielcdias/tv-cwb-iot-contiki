/**
 * File:   pluv-interruption.h
 * Author: Daniel Carvalho Dias (daniel.dias@gmail.com)
 * Date:   28/05/2019
 *
 * Interface to configure and the pluviometer to work with interruption
 * (as a button).
 *
 */

#include "board.h"
#include "dev/adc-sensor.h"
#include "lib/sensors.h"

#ifndef PLUV_SENSOR_H_
#define PLUV_SENSOR_H_

/**************************************************************************
 * Definitions for pluviometer sensor works with interruption (as a button)
 **************************************************************************/

#define INTERRUPTION_SENSOR "Interruption_Event_Sensor"

#define INTERRUPTION_SENSOR_VALUE_STATE    0
#define INTERRUPTION_SENSOR_VALUE_DURATION 1

#define INTERRUPTION_SENSOR_VALUE_RELEASED 0
#define INTERRUPTION_SENSOR_VALUE_PRESSED  1

extern const struct sensors_sensor pluviometer_sensor;

#endif
