/**
 * File:   temp-sensor-helper.h
 * Author: Daniel Carvalho Dias (daniel.dias@gmail.com)
 * Date:   12/07/2019
 *
 * Defines interface to configure and read from temperature sensors used by
 * the CC2650 board.
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

#include "sensors-helper.h"

/******************************************************************************
 * Definitions of constants for port used and internal commands
 ******************************************************************************/
#define DS18B20_COMMAND_READ_SCRATCH_PAD 0xBE
#define DS18B20_COMMAND_START_CONVERSION 0x44
#define DS18B20_COMMAND_SKIP_ROM 0xCC
#define DS18B20_PORT TEMPERATURE_SENSOR

/******************************************************************************
 * Functions definitions
 ******************************************************************************/

uint8_t ds18b20_probe(void);

uint8_t ds18b20_get_temp(float *temp1);
