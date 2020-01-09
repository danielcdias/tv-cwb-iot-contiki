/**
 * File:   interruption-sensor.c
 * Author: Daniel Carvalho Dias (daniel.dias@gmail.com)
 * Date:   28/05/2019
 *
 * Implementation a sensor that works via interruptions (as a button).
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

#include "launchpad/button-sensor.h"
#include "launchpad/bmp-280-sensor.h"

#include "interruption-sensor.h"

/*******************************************************************
 * Definitions for sensor that works via  interruption (as a button)
 *******************************************************************/

#define INTERRUPTION_GPIO_CFG   (IOC_CURRENT_2MA  | IOC_STRENGTH_AUTO | \
                                 IOC_IOPULL_UP    | IOC_SLEW_DISABLE  | \
                                 IOC_HYST_DISABLE | IOC_BOTH_EDGES    | \
                                 IOC_INT_ENABLE   | IOC_IOMODE_NORMAL | \
                                 IOC_NO_WAKE_UP   | IOC_INPUT_ENABLE)

#define INTERRUPTION_DEBOUNCE_DURATION (CLOCK_SECOND >> 5)

/******************
 * Global variables
 ******************/
struct interruption_timer {
  struct timer debounce;
  clock_time_t start;
  clock_time_t duration;
};

static struct interruption_timer irq_timer;

/***************************************************
 * Functions for interruption (event) implementation
 ***************************************************/

static void interruption_event_handler(uint8_t ioid) {

  if(ioid == INTERRUPTION_SENSOR_PORT) {
      if (!timer_expired(&irq_timer.debounce)) {
        return;
      }

      timer_set(&irq_timer.debounce, INTERRUPTION_DEBOUNCE_DURATION);

      /*
       * Start press duration counter on press (falling), notify on release
       * (rising)
       */
      if (ti_lib_gpio_read_dio(INTERRUPTION_SENSOR_PORT) == 0) {
        irq_timer.start = clock_time();
        irq_timer.duration = 0;
      } else {
        irq_timer.duration = clock_time() - irq_timer.start;
        sensors_changed(&interruption_sensor);
      }
  }

}

static void config_interuptions(int type, int c, uint32_t key) {
  switch (type) {
  case SENSORS_HW_INIT:
    ti_lib_gpio_clear_event_dio(key);
    ti_lib_rom_ioc_pin_type_gpio_input(key);
    ti_lib_rom_ioc_port_configure_set(key, IOC_PORT_GPIO, INTERRUPTION_GPIO_CFG);
    gpio_interrupt_register_handler(key, interruption_event_handler);
    break;
  case SENSORS_ACTIVE:
    if (c) {
      ti_lib_gpio_clear_event_dio(key);
      ti_lib_rom_ioc_pin_type_gpio_input(key);
      ti_lib_rom_ioc_port_configure_set(key, IOC_PORT_GPIO, INTERRUPTION_GPIO_CFG);
      ti_lib_rom_ioc_int_enable(key);
    } else {
      ti_lib_rom_ioc_int_disable(key);
    }
    break;
  default:
    break;
  }
}

static int config_sensor(int type, int value) {
  config_interuptions(type, value, INTERRUPTION_SENSOR_PORT);

  return 1;
}

static int status(int type, uint32_t key_io_id) {
  switch (type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    if (ti_lib_rom_ioc_port_configure_get(key_io_id) & IOC_INT_ENABLE) {
      return 1;
    }
    break;
  default:
    break;
  }
  return 0;
}

static int value_interruption(int type) {
  if (type == INTERRUPTION_SENSOR_VALUE_STATE) {
    return
    ti_lib_gpio_read_dio(INTERRUPTION_SENSOR_PORT) == 0 ?
        INTERRUPTION_SENSOR_VALUE_PRESSED : INTERRUPTION_SENSOR_VALUE_RELEASED;
  } else if (type == INTERRUPTION_SENSOR_VALUE_DURATION) {
    return (int) irq_timer.duration;
  }
  return 0;
}

static int status_interruption(int type) {
  return status(type, INTERRUPTION_SENSOR_PORT);
}

/**
 * Declaring the interruption sensor.
 */
SENSORS_SENSOR(interruption_sensor, INTERRUPTION_SENSOR, value_interruption, config_sensor,
   status_interruption);

SENSORS(&button_left_sensor, &button_right_sensor, &bmp_280_sensor, &adc_sensor, &interruption_sensor);
