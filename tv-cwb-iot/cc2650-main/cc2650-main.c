/**
 * File:  projeto-final-cc2650.c
 * Author: Daniel Carvalho Dias (daniel.dias@gmail.com)
 * Date:   25/05/2019
 *
 * Main C file for the Terraço Verde CWB project. Implements a set of sensor to collect
 * data regarding the influence of green terraces in rain absorption.
 *
 * Each CC2650 board will control 1 optical rain sensor or a pluviometer, 1
 * capacitive rain sensor on drain, 1 capacitive soil moisture sensor and 1
 * temperature sensor.
 */

/******************************************************************************
 * Constants and includes
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "lib/random.h"
#include "sys/timer.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-nameserver.h"
#include "mqtt-sn.h"
#include "rpl.h"
#include "net/ip/resolv.h"
#include "net/rime/rime.h"
#include "simple-udp.h"
#include "ti-lib.h"
#include "dev/leds.h"
#include "button-sensor.h"

#define FIRMWARE_VERSION "01.02.03"

#define DEBUG 1

#include "net/ip/uip-debug.h"

#include "sensors-helper.h"
#include "temp-sensor-helper.h"
#include "interruption-sensor.h"

#define MDNS 1

#define UDP_PORT 1883

#define REQUEST_RETRIES 10
#define DEFAULT_SEND_INTERVAL (10 * CLOCK_SECOND)
#define REPLY_TIMEOUT (3 * CLOCK_SECOND)
#define INACTIVITY_TIMEOUT (300 * CLOCK_SECOND)

#define TOPIC_STA_SENSOR "/tvcwb1299/mmm/sta/%02X%02X/%s"
#define TOPIC_STA_GENERAL "/tvcwb1299/mmm/sta/%02X%02X"
#define TOPIC_CMD "/tvcwb1299/mmm/cmd/%02X%02X"

// Send board and sensores status
// Format S000P000P00 or S000O000P00, where:
// S = sensors, followed by an array of status:
//    Pos 0 = SCR
//    Pos 0 = SCU
//    Pos 0 = TMP
//    Values: 0 for OK, 9 for error
// P (PLV) or O (SOC), followed by number of ticks detected at the moment
// P (processes running in board), followed by the number of process
#define BOARD_STATUS_ARRAY_PLV "%sS%u%u%uP%uP%uA%u"
#define BOARD_STATUS_ARRAY_SOC "%sS%u%u%uO%uP%uA%u"
#define SENSOR_READING_OK 0
#define SENSOR_READING_ERROR 9

#define FIRMWARE_VERSION_STATUS_ARRAY "FWV%s"

#define MESSAGE_STATUS_FORMAT "%iT%li"
#define MESSAGE_BOARD_FORMAT "%sT%li"

#define UDP_CONNECTION_ADDR       tv-cwb-iot.mooo.com
#define UDP_CONNECTION_ADDR_DEV   danieldias.mooo.com

#define _QUOTEME(x) #x
#define QUOTEME(x) _QUOTEME(x)

#define GREEN_LED_SENDING_MESSAGE 1
#define GREEN_LED_NO_MESSAGE 2
#define GREEN_LED_OFF 3
#define GREEN_LED_OFF_REBOOTING 4

#define RED_LED_CONNECTING 1
#define RED_LED_CONNECTED 2
#define RED_LED_OFF 3
#define RED_LED_OFF_REBOOTING 4

#define TOPIC_CONTROL 0
#define TOPIC_STATUS_GENERAL 1
#define TOPIC_OPTICAL_RAIN_SENSOR 2
#define TOPIC_RAIN_SENSOR_DRAIN 3
#define TOPIC_CAPACITIVE_SOIL_MOISTURE_SENSOR 4
#define TOPIC_TEMPERATURE_SENSOR 5
#define TOPIC_PLUVIOMETER 6

#define TOTAL_TOPICS 7

#define BOARD_STATUS_STARTED "STT"
#define BOARD_STATUS_TIMESTAMP_UPDATE_REQUEST "TUR"
#define BOARD_COMMAND_RESET "RST"

#define TIMESTAMP_UPDATE_REQUEST_INTERVAL 86400 // 1 day in seconds

#define PUBLISH_MESSAGES_DELAY 0.2

// Indicates if board is running in development environment
#define DEV_ENVIRONMENT_JUMPER IOID_1

// Possible valued for dev environment jumper
#define DEV_ENVIRONMENT 1
#define PROD_ENVIRONMENT 0

// Interval to report board general status
#define REPORT_BOARD_GENERAL_STATUS_INTERVAL 295 // seconds

// Size of the array of process that has all processes status (running ot stopped)
#define PROCESS_ARRAY_SIZE 14

// Processes status
#define PROCESS_RUNNING 1
#define PROCESS_STOPPED 0

// Process index name
#define MQTTSN_PROCESS 0
#define PUBLISH_PROCESS 1
#define CTRL_SUBSCRIPTION_PROCESS 2
#define WATCHDOG_PROCESS 3
#define REBOOT_PROCESS 4
#define GREEN_LED_PROCESS 5
#define RED_LED_PROCESS 6
#define REQUEST_TIMESTAMP_UPDATE_PROCESS 7
#define INTERRUPTION_SENSOR_RESET_INTERVAL_PROCESS 8
#define RAIN_SENSOR_DRAIN_PROCESS 9
#define MOISTURE_SENSOR_PROCESS 10
#define INTERRUPTION_SENSOR_PROCESS 11
#define REPORT_BOARD_GENERAL_STATUS_PROCESS 12
#define DETECT_TEST_MODE_PROCESS 13

// Constant array with the expected processes status after controller is connected
// Following the process index defined abover
#define PROCESSES_AFTER_CONNECTION \
   { \
      PROCESS_RUNNING, \
      PROCESS_STOPPED, \
      PROCESS_STOPPED, \
      PROCESS_RUNNING, \
      PROCESS_STOPPED, \
      PROCESS_RUNNING, \
      PROCESS_RUNNING, \
      PROCESS_RUNNING, \
      PROCESS_RUNNING, \
      PROCESS_RUNNING, \
      PROCESS_RUNNING, \
      PROCESS_RUNNING, \
      PROCESS_RUNNING, \
      PROCESS_STOPPED  \
   }

// Process array typedef
typedef uint8_t bit_array_t[PROCESS_ARRAY_SIZE];

// Reason sent to board status message before watchdog reboot
#define RESET_BY_COMMAND "C"
#define RESET_BY_DISCONNECTION "D"
#define RESET_BY_KEEP_ALIVE "K"
#define RESET_BY_CANT_CONNECT "X"
#define RESET_BY_TIMEOUT_REGISTER "T"
#define RESET_BY_TIMEOUT_TIMESTAMP "S"
#define RESET_BY_UNABLE_CONNECT "U"
#define RESET_BY_INACTIVITY "I"
#define RESET_BY_PROCESS_STOPPED "P"
// Normal board status without reset
#define NORMAL_BOARD_STATUS "N"

/******************************************************************************
 * Global variables and structs definitions
 ******************************************************************************/

/**
 * Topic sensor struct to contain all topics, topic ids and message ids
 * information
 *
 * Positions inside array:
 * 0 - control topic (TOPIC_CONTROL)
 * 1 - general status topic (TOPIC_STATUS_GENERAL)
 * 2 - optical rain sensor topic (TOPIC_OPTICAL_RAIN_SENSOR)
 * 6 - rain sensor for drain topic (TOPIC_RAIN_SENSOR_DRAIN)
 * 7 - capacitive soil moisture sensor topic (TOPIC_CAPACITIVE_SOIL_MOISTURE_SENSOR)
 * 8 - temperature sensor (TOPIC_TEMPERATURE_SENSOR)
 * 9 - pluviometer sensor (TOPIC_PLUVIOMETER)
 *
 * 'sensor_id' only has value for topics of sensors (index from 2 to 9)
 *
 */
struct topic_sensor_t {
      uint16_t topic_id;
      char topic[28];
      uint16_t message_id;
      char sensor_id[3];
};

static struct mqtt_sn_connection mqtt_sn_c;
static char mqtt_client_id[17];
static struct topic_sensor_t pub_sensors_topic[TOTAL_TOPICS];
static publish_packet_t incoming_packet;
static uint16_t mqtt_keep_alive=10;
static int8_t qos = 1;
static uint8_t retain = FALSE;
static clock_time_t send_interval;
static mqtt_sn_subscribe_request subreq;
static mqtt_sn_register_request regreq;

static bool is_rebooting = false;
static bool are_sta_topics_registered = false;
static bool is_cmd_topic_registered = false;
static bool is_connected = false;

static uint16_t interruption_counter = 0;

static uint32_t base_timestamp_from_server = 0;
static uint32_t base_clock_seconds = 0;

static uint8_t green_led_state = GREEN_LED_OFF;
static uint8_t red_led_state = RED_LED_CONNECTING;

static enum mqttsn_connection_status connection_state = MQTTSN_DISCONNECTED;

static struct ctimer connection_timer;
static process_event_t connection_timeout_event;

static process_event_t mqttsn_connack_event, network_inactivity_timeout_reset,
         interruption_sensor_tic_event, test_mode_activation_ended;

static uint8_t processes_running = 0;

static bool is_pluviometer_installed = false;

static bool is_dev_environment = false;

static bool test_mode_on = false;
static bool test_mode_available = true;
static bool ready_to_work = false;

static uint8_t last_reading_ok[3];

static bit_array_t process_status_array =
   {
      PROCESS_STOPPED,
      PROCESS_STOPPED,
      PROCESS_STOPPED,
      PROCESS_STOPPED,
      PROCESS_STOPPED,
      PROCESS_STOPPED,
      PROCESS_STOPPED,
      PROCESS_STOPPED,
      PROCESS_STOPPED,
      PROCESS_STOPPED,
      PROCESS_STOPPED,
      PROCESS_STOPPED,
      PROCESS_STOPPED,
      PROCESS_STOPPED
   };

/******************************************************************************
 * Processes definition
 ******************************************************************************/

PROCESS(mqttsn_process, "Configure Connection and Topic Registration");
PROCESS(publish_process, "Register topic and publish data");
PROCESS(ctrl_subscription_process, "Subscribe to a device control channel");
PROCESS(watchdog_process, "Watchdog for processes and network inactivity");

PROCESS(reboot_process, "Reboots the board");

PROCESS(green_led_process, "Controls green led indicator");
PROCESS(red_led_process, "Controls ref led indicator");

PROCESS(request_timestamp_update_process, "Send command requesting timestamp update");

PROCESS(interruption_sensor_reset_interval_process, "Resets interruption sensor counter when time interval is reached");

PROCESS(rain_sensor_drain_process, "Reads rain sensor on drain");
PROCESS(moisture_sensor_process, "Reads from capacitive soil moisture and temperature sensors");
PROCESS(interruption_sensor_process, "Receives events from interruption sensor");

PROCESS(report_board_general_status_process, "Reports board general status");

PROCESS(detect_test_mode_process, "Detects if test mode was requested");

AUTOSTART_PROCESSES(&mqttsn_process, &detect_test_mode_process);

/******************************************************************************
 * Functions definition
 ******************************************************************************/

void reboot_board(char reason[2]);
void report_board_status(char reason[2]);

/******************************************************************************
 * Functions implementation
 ******************************************************************************/

/*********************************** General **********************************/

void reboot_board(char reason[2]) {
   is_rebooting = true;
   report_board_status(reason);
   process_start(&reboot_process, NULL);
}

void loop_forever() {
   while (1) {
      static uint32_t i;
      for (i = 0; i < 100000; i++);
      leds_toggle(LEDS_ALL);
   }
}

void set_green_led(uint8_t value ) {
   green_led_state = value;
}

void set_red_led(uint8_t value) {
   red_led_state = value;
}

uint16_t get_process_status() {
    uint16_t result = 0, power = 1;
    for (uint8_t i = 0; i < PROCESS_ARRAY_SIZE; i++ ) {
        result += process_status_array[(PROCESS_ARRAY_SIZE -1) -i] * power;
        power *= 2;
    }
    return result;
}

bool is_all_processes_running() {
   static bool result = true;
   static uint8_t i;
   const uint8_t processes_after_connection[PROCESS_ARRAY_SIZE] = PROCESSES_AFTER_CONNECTION;
   for (i = 0; i < PROCESS_ARRAY_SIZE; i++) {
      if ((processes_after_connection[i] == PROCESS_RUNNING) &&
               (process_status_array[i] == PROCESS_STOPPED)) {
         result = false;
         break;
      }
   }
   return result;
}


/************************************ MQTT ************************************/

static void init_sensor_topics_array() {
   strcpy(pub_sensors_topic[TOPIC_CONTROL].sensor_id, "\0");
   strcpy(pub_sensors_topic[TOPIC_STATUS_GENERAL].sensor_id, "\0");
   strcpy(pub_sensors_topic[TOPIC_OPTICAL_RAIN_SENSOR].sensor_id, SENSOR_OPTICAL_RAIN);
   strcpy(pub_sensors_topic[TOPIC_RAIN_SENSOR_DRAIN].sensor_id, SENSOR_RAIN_DRAIN);
   strcpy(pub_sensors_topic[TOPIC_CAPACITIVE_SOIL_MOISTURE_SENSOR].sensor_id, SENSOR_CAPACITIVE_SOIL_MOISTURE);
   strcpy(pub_sensors_topic[TOPIC_TEMPERATURE_SENSOR].sensor_id, SENSOR_TEMPERATURE);
   strcpy(pub_sensors_topic[TOPIC_PLUVIOMETER].sensor_id, SENSOR_PLUVIOMETER);
   sprintf(pub_sensors_topic[TOPIC_CONTROL].topic, TOPIC_CMD, linkaddr_node_addr.u8[6],
           linkaddr_node_addr.u8[7]);
   sprintf(pub_sensors_topic[TOPIC_STATUS_GENERAL].topic, TOPIC_STA_GENERAL, linkaddr_node_addr.u8[6],
           linkaddr_node_addr.u8[7]);
   static uint8_t i;
   for (i = 2; i < TOTAL_TOPICS; i++) {
      sprintf(pub_sensors_topic[i].topic, TOPIC_STA_SENSOR, linkaddr_node_addr.u8[6],
              linkaddr_node_addr.u8[7], pub_sensors_topic[i].sensor_id);
   }
}

static void
puback_receiver(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen) {
   PRINTF("[E] Puback received\n");
   set_green_led(GREEN_LED_NO_MESSAGE);
}

static void connack_receiver(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen) {
   uint8_t connack_return_code;
   connack_return_code = *(data + 3);
   PRINTF("[E] Connack received\n");
   if (connack_return_code == ACCEPTED) {
      process_post(&mqttsn_process, mqttsn_connack_event, NULL);
   } else {
     PRINTF("[E] Connack error: %s\n", mqtt_sn_return_code_string(connack_return_code));
   }
}

static void regack_receiver(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen) {
   regack_packet_t incoming_regack;
   memcpy(&incoming_regack, data, datalen);
   PRINTF("[E] Regack received\n");
   static bool found = false;
   static uint8_t i;
   for (i = 1; i < TOTAL_TOPICS; i++) {
      if (incoming_regack.message_id == pub_sensors_topic[i].message_id) {
         if (incoming_regack.return_code == ACCEPTED) {
            pub_sensors_topic[i].topic_id = uip_htons(incoming_regack.topic_id);
            found = true;
            break;
         }
      }
   }
   if (!found) {
      PRINTF("[E] Regack error: %s\n", mqtt_sn_return_code_string(incoming_regack.return_code));
   }
}

static void suback_receiver(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen) {
   suback_packet_t incoming_suback;
   memcpy(&incoming_suback, data, datalen);
   PRINTF("[E] Suback received\n");
   if (incoming_suback.message_id == pub_sensors_topic[TOPIC_CONTROL].message_id) {
      if (incoming_suback.return_code == ACCEPTED) {
         pub_sensors_topic[TOPIC_CONTROL].topic_id = uip_htons(incoming_suback.topic_id);
      } else {
        PRINTF("[E] Suback error: %s\n", mqtt_sn_return_code_string(incoming_suback.return_code));
      }
   }
}

static void publish_receiver(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen) {
   memcpy(&incoming_packet, data, datalen);
   incoming_packet.data[datalen-7] = 0x00;
   PRINTF("[E] Published message received: %s\n", incoming_packet.data);
   if (uip_htons(incoming_packet.topic_id) == pub_sensors_topic[TOPIC_CONTROL].topic_id) {
      if (incoming_packet.data[0] == 'T') {
         PRINTF("}}}}} Timestamp from SERVER received!\n");
         char timestamp_str[11] = "\0";
         memcpy(timestamp_str, &incoming_packet.data[1], 10);
         base_clock_seconds = clock_seconds();
         base_timestamp_from_server = atoi(timestamp_str);
      } else  if (strcmp(BOARD_COMMAND_RESET, incoming_packet.data) == 0) {
         PRINTF("\n*}*}*} Reset command received! {*{*{*\n\n");
         reboot_board(RESET_BY_COMMAND);
      } else {
         PRINTF("}}}}} Command not identified: '%s'.\n", incoming_packet.data);
      }
   } else {
      PRINTF("[E] Unknown publication received.\n");
   }
}

static void pingreq_receiver(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen) {
   PRINTF("[E] PingReq received\n");
}

static void disconnect_receiver(struct mqtt_sn_connection *mqc, const uip_ipaddr_t *source_addr, const uint8_t *data, uint16_t datalen) {
   PRINTF("[E] Disconnection received\n");
   is_connected = false;
   reboot_board(RESET_BY_DISCONNECTION);
}

static void keepalive_timeout_receiver(struct mqtt_sn_connection *mqc) {
   PRINTF("[E] Keepalive timeout received\n");
   is_connected = false;
   reboot_board(RESET_BY_KEEP_ALIVE);
}

static const struct mqtt_sn_callbacks mqtt_sn_call = {
   publish_receiver,
   pingreq_receiver,
   NULL,
   connack_receiver,
   regack_receiver,
   puback_receiver,
   suback_receiver,
   disconnect_receiver,
   keepalive_timeout_receiver
};

static uint32_t get_current_timestamp() {
   return base_timestamp_from_server + (clock_seconds() - base_clock_seconds);
}

static void publish_board_status(char data[50]) {
   if ((test_mode_on) && (strcmp(BOARD_STATUS_STARTED, data))) {
      process_post(&watchdog_process, network_inactivity_timeout_reset, NULL);
      return;
   }
   set_green_led(GREEN_LED_SENDING_MESSAGE);
   static uint8_t buf_len;
   static char buf[64] = "\0";
   if ((strcmp(BOARD_STATUS_STARTED, data)) && (strcmp(BOARD_STATUS_TIMESTAMP_UPDATE_REQUEST, data))) {
      sprintf(buf, MESSAGE_BOARD_FORMAT, data, get_current_timestamp());
   } else {
      strcpy(buf, data);
   }
   buf_len = strlen(buf);
   PRINTF("Publishing at topic: %s -> msg: %s\n", pub_sensors_topic[TOPIC_STATUS_GENERAL].topic, buf);
   uint16_t result = mqtt_sn_send_publish(&mqtt_sn_c, pub_sensors_topic[TOPIC_STATUS_GENERAL].topic_id,
                       MQTT_SN_TOPIC_TYPE_NORMAL, buf, buf_len, qos, retain);
   process_post(&watchdog_process, network_inactivity_timeout_reset, NULL);
   if (result == 0) {
      PRINTF("** Error publishing message **\n");
   }
}


static void publish_sensor_status(const uint8_t topic_index, int data) {
   if (test_mode_on) {
      process_post(&watchdog_process, network_inactivity_timeout_reset, NULL);
      return;
   }
   set_green_led(GREEN_LED_SENDING_MESSAGE);
   static char buf[64] = "\0";
   static uint8_t buf_len;
   sprintf(buf, MESSAGE_STATUS_FORMAT, data, get_current_timestamp());
   PRINTF("Publishing at topic: %s -> msg: %s\n", pub_sensors_topic[topic_index].topic, buf);
   buf_len = strlen(buf);
   uint16_t result = mqtt_sn_send_publish(&mqtt_sn_c, pub_sensors_topic[topic_index].topic_id,
                        MQTT_SN_TOPIC_TYPE_NORMAL, buf, buf_len, qos, retain);
   process_post(&watchdog_process, network_inactivity_timeout_reset, NULL);
   if (result == 0) {
      PRINTF("** Error publishing message **\n");
   }
}

static void publish_firmware_version() {
   char buf[12] = "\0";
   sprintf(buf, FIRMWARE_VERSION_STATUS_ARRAY, FIRMWARE_VERSION);
   publish_board_status(buf);
}

void report_board_status(char reason[2]) {
   char rsa_values[20] = "\0";
   sprintf(
      rsa_values,
      (is_pluviometer_installed ? BOARD_STATUS_ARRAY_PLV : BOARD_STATUS_ARRAY_SOC), reason,
      last_reading_ok[0], last_reading_ok[1], last_reading_ok[2],
      interruption_counter, processes_running, get_process_status());
   PRINTF("§§§§§ Board status: %s.\n", rsa_values);
   publish_board_status(rsa_values);
}

static void connection_timer_callback(void *mqc) {
   process_post(&mqttsn_process, connection_timeout_event, NULL);
}

static void print_local_addresses(void) {
   static uint8_t i;
   uint8_t state;

   PRINTF("Client IPv6 addresses: ");
   for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
      state = uip_ds6_if.addr_list[i].state;
      if(uip_ds6_if.addr_list[i].isused &&
         (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
        PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
        PRINTF("\n");
      }
   }
}

static resolv_status_t set_connection_address(uip_ipaddr_t *ipaddr) {

   resolv_status_t status = RESOLV_STATUS_ERROR;

   static char mqtt_host[100] = QUOTEME(UDP_CONNECTION_ADDR);
   if (is_dev_environment) {
      strcpy(mqtt_host, QUOTEME(UDP_CONNECTION_ADDR_DEV));
   }

   if(uiplib_ipaddrconv(mqtt_host, ipaddr) == 0) {
      uip_ipaddr_t *resolved_addr = NULL;
      status = resolv_lookup(mqtt_host, &resolved_addr);
      if(status == RESOLV_STATUS_UNCACHED || status == RESOLV_STATUS_EXPIRED) {
         PRINTF("Attempting to look up %s\n", mqtt_host);
         resolv_query(mqtt_host);
         status = RESOLV_STATUS_RESOLVING;
      } else if(status == RESOLV_STATUS_CACHED && resolved_addr != NULL) {
         PRINTF("Lookup of \"%s\" succeded!\n", mqtt_host);
      } else if(status == RESOLV_STATUS_RESOLVING) {
         PRINTF("Still looking up \"%s\"...\n", mqtt_host);
      } else {
         PRINTF("Lookup of \"%s\" failed. status = %d\n", mqtt_host, status);
      }
      if(resolved_addr) {
         uip_ipaddr_copy(ipaddr, resolved_addr);
      }
   } else {
    status = RESOLV_STATUS_CACHED;
   }

   return status;
}

/******************************************************************************
 * Processes implementation
 ******************************************************************************/

/************************************ MQTT ************************************/

/**
 * Registers the topics used to report statuses from sensors.
 */
PROCESS_THREAD(publish_process, ev, data) {
   PROCESS_BEGIN();

   process_status_array[PUBLISH_PROCESS] = PROCESS_RUNNING;
   processes_running++;
   PRINTF("--->>> Starting process thread 'publish_process'\n");

   static uint8_t registration_tries, rreq_result, rreq_results = 0;
   static struct etimer send_timer;


   send_interval = DEFAULT_SEND_INTERVAL;
   static uint8_t i;
   for (i = 1; i < TOTAL_TOPICS; i++) {
      PRINTF("Registering topic %s\n", pub_sensors_topic[i].topic);
      static mqtt_sn_register_request *rreq = &regreq;
      registration_tries = 0;
      rreq_result = 0;
      while ((!is_rebooting) && (registration_tries < REQUEST_RETRIES)) {
         pub_sensors_topic[i].message_id = mqtt_sn_register_try(rreq, &mqtt_sn_c, pub_sensors_topic[i].topic, REPLY_TIMEOUT);
         etimer_set(&send_timer, 1 * CLOCK_SECOND);
         PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && data == &send_timer);
         if (mqtt_sn_request_success(rreq)) {
            registration_tries = REQUEST_RETRIES;
            PRINTF("Registration acked.\n");
         } else {
            registration_tries++;
            if (rreq->state == MQTTSN_REQUEST_FAILED) {
               PRINTF("Regack error: %s\n", mqtt_sn_return_code_string(rreq->return_code));
            }
         }
      }
      rreq_result = mqtt_sn_request_success(rreq);
      rreq_results += rreq_result;
      if (rreq_result == 0) {
         PRINTF("Unable to register to topic %s.\n", pub_sensors_topic[i].topic);
         break;
      }
   }
   if (rreq_results == (TOTAL_TOPICS - 1)) {
      are_sta_topics_registered = true;
      PRINTF("All topics registered.\n");
   }

   PRINTF("<<<--- Ending process thread 'publish_process'\n");
   processes_running--;
   process_status_array[PUBLISH_PROCESS] = PROCESS_STOPPED;

   PROCESS_END();
}

/**
 * Registers the topic used as command receiver (control).
 */
PROCESS_THREAD(ctrl_subscription_process, ev, data) {
   PROCESS_BEGIN();

   process_status_array[CTRL_SUBSCRIPTION_PROCESS] = PROCESS_RUNNING;
   processes_running++;
   PRINTF("--->>> Starting process thread 'ctrl_subscription_process'\n");

   static uint8_t subscription_tries;
   static mqtt_sn_subscribe_request *sreq = &subreq;
   static struct etimer periodic_timer;
   subscription_tries = 0;
   PRINTF("Requesting subscription\n");
   while ((!is_rebooting) && (subscription_tries < REQUEST_RETRIES)) {
      PRINTF("Subscribing... topic: %s\n", pub_sensors_topic[TOPIC_CONTROL].topic);
      pub_sensors_topic[TOPIC_CONTROL].message_id = mqtt_sn_subscribe_try(sreq,
         &mqtt_sn_c, pub_sensors_topic[TOPIC_CONTROL].topic, 0, REPLY_TIMEOUT);
      etimer_set(&periodic_timer, 1 * CLOCK_SECOND);
      PROCESS_WAIT_EVENT();
      if (mqtt_sn_request_success(sreq)) {
          subscription_tries = REQUEST_RETRIES;
          PRINTF("Subscription acked\n");
          is_cmd_topic_registered = true;
      }
      else {
          subscription_tries++;
          if (sreq->state == MQTTSN_REQUEST_FAILED) {
              PRINTF("Suback error: %s\n", mqtt_sn_return_code_string(sreq->return_code));
          }
      }
   }

   PRINTF("<<<--- Ending process thread 'ctrl_subscription_process'\n");
   processes_running--;
   process_status_array[CTRL_SUBSCRIPTION_PROCESS] = PROCESS_STOPPED;

   PROCESS_END();
}

/**
 * Starts all MQTT communication and all other processes.
 */
PROCESS_THREAD(mqttsn_process, ev, data) {
   PROCESS_BEGIN();

   process_status_array[MQTTSN_PROCESS] = PROCESS_RUNNING;
   processes_running++;
   PRINTF("--->>> Starting process thread 'mqttsn_process'\n");

   static struct etimer periodic_timer;
   static struct etimer et;
   static uip_ipaddr_t broker_addr,google_dns;
   static uint8_t connection_retries = 0;
   static resolv_status_t status;
   char contiki_hostname[16];

   PRINTF("TV-CWB-IOT - Starting with firmware version %s\n", FIRMWARE_VERSION);

   process_start(&green_led_process, NULL);
   process_start(&red_led_process, NULL);

   etimer_set(&et, 1 * CLOCK_SECOND);
   PROCESS_WAIT_EVENT_UNTIL(ev = PROCESS_EVENT_TIMER);

   sprintf(contiki_hostname, "node%02X%02X", linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[7]);
   resolv_set_hostname(contiki_hostname);
   PRINTF("====== Setting hostname to %s\n", contiki_hostname);

   mqttsn_connack_event = process_alloc_event();

   mqtt_sn_set_debug(1);
   uip_ip6addr(&google_dns, 0x2001, 0x4860, 0x4860, 0x0, 0x0, 0x0, 0x0, 0x8888);

   set_red_led(RED_LED_CONNECTING);

   etimer_set(&periodic_timer, 2 * CLOCK_SECOND);
   while ((!is_rebooting) && (uip_ds6_get_global(ADDR_PREFERRED) == NULL)) {
      PROCESS_WAIT_EVENT();
      if(etimer_expired(&periodic_timer)) {
         PRINTF("Waiting for IP auto configuration...\n");
         etimer_set(&periodic_timer, 2 * CLOCK_SECOND);
      }
   }

   print_local_addresses();

   rpl_dag_t *dag = rpl_get_any_dag();
   if(dag) {
      uip_nameserver_update(&google_dns, UIP_NAMESERVER_INFINITE_LIFETIME);
   }

   is_dev_environment = (readGPIOSensor(DEV_ENVIRONMENT_JUMPER) == DEV_ENVIRONMENT);

   status = RESOLV_STATUS_UNCACHED;
   connection_retries = 0;
   while ((!is_rebooting) && (status != RESOLV_STATUS_CACHED)) {
      status = set_connection_address(&broker_addr);

      if(status == RESOLV_STATUS_RESOLVING) {
         PROCESS_WAIT_EVENT();
      } else if(status != RESOLV_STATUS_CACHED) {
         PRINTF("Can't get connection address.\n");
         etimer_set(&periodic_timer, 2 * CLOCK_SECOND);
         PROCESS_WAIT_EVENT();
         connection_retries++;
         if (connection_retries >= 15) {
            reboot_board(RESET_BY_CANT_CONNECT);
            break;
         }
      }
   }

   if (!is_rebooting) {
      mqtt_sn_create_socket(&mqtt_sn_c,UDP_PORT, &broker_addr, UDP_PORT);
      (&mqtt_sn_c)->mc = &mqtt_sn_call;

      sprintf(mqtt_client_id,"sens%02X%02X%02X%02X",linkaddr_node_addr.u8[4],linkaddr_node_addr.u8[5],linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[7]);

      PRINTF("Requesting connection...\n");
      connection_timeout_event = process_alloc_event();
      connection_retries = 0;
      ctimer_set(&connection_timer, REPLY_TIMEOUT, connection_timer_callback, NULL);
      mqtt_sn_send_connect(&mqtt_sn_c,mqtt_client_id,mqtt_keep_alive);
      connection_state = MQTTSN_WAITING_CONNACK;
      while ((!is_rebooting) && (connection_retries < 11)) {
        PROCESS_WAIT_EVENT();
        if (ev == mqttsn_connack_event) {
           PRINTF("Connection acked.\n");
           ctimer_stop(&connection_timer);
           connection_state = MQTTSN_CONNECTED;
           connection_retries = 15;  //using break here may mess up switch statement of process
         }
         if (ev == connection_timeout_event) {
            connection_state = MQTTSN_CONNECTION_FAILED;
            connection_retries++;
            PRINTF("Connection timeout (%i).\n", connection_retries);
            ctimer_restart(&connection_timer);
            if (connection_retries < 15) {
               mqtt_sn_send_connect(&mqtt_sn_c,mqtt_client_id,mqtt_keep_alive);
               connection_state = MQTTSN_WAITING_CONNACK;
            }
         }
      }
      ctimer_stop(&connection_timer);
      if (connection_state == MQTTSN_CONNECTED){
         is_connected = true;
         init_sensor_topics_array();
         process_start(&ctrl_subscription_process, NULL);
         process_start(&publish_process, NULL);
         process_start(&watchdog_process, NULL);
         static uint8_t i = 0;
         while ((!is_rebooting) && ((!are_sta_topics_registered) || (!is_cmd_topic_registered))) {
            etimer_set(&et, 0.1 * CLOCK_SECOND);
            PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && data == &et);
            i++;
            if (i >= 200) {
               PRINTF("***** Taking too long to register topics...\n");
               reboot_board(RESET_BY_TIMEOUT_REGISTER);
               break;
            }
         }
         if (!is_rebooting) {
            set_red_led(RED_LED_CONNECTED);
            set_green_led(GREEN_LED_NO_MESSAGE);
            publish_board_status(BOARD_STATUS_STARTED);
            i = 0;
            while (base_timestamp_from_server == 0) {
               etimer_set(&et, CLOCK_SECOND);
               PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && data == &et);
               i++;
               if (i >= 60) {
                  PRINTF("***** Taking too long to received base timestamp...\n");
                  reboot_board(RESET_BY_TIMEOUT_TIMESTAMP);
                  break;
               }
            }
            if (!is_rebooting) {
               test_mode_available = false;
               process_post(&detect_test_mode_process, test_mode_activation_ended, NULL);
               publish_firmware_version();
               configureGPIOSensors();
               process_start(&report_board_general_status_process, NULL);
               process_start(&request_timestamp_update_process, NULL);
               process_start(&rain_sensor_drain_process, NULL);
               process_start(&moisture_sensor_process, NULL);
               is_pluviometer_installed = (readGPIOSensor(JUMPER_PLUVIOMETER_INSTALLED) == PLUVIOMETER_INSTALLED);
               if (is_pluviometer_installed)  {
                  PRINTF("### Pluviometer is installed in this control board.\n");
               } else {
                  PRINTF("### Optioncal rain sensor is installed in this control board.\n");
               }
               process_start(&interruption_sensor_process, NULL);
               process_start(&interruption_sensor_reset_interval_process, NULL);
               ready_to_work = true;
               etimer_set(&et, 2 * CLOCK_SECOND);
               while(!is_rebooting) {
                  PROCESS_WAIT_EVENT();
                  if(etimer_expired(&et)) {
                     etimer_restart(&et);
                  }
               }
            }
         }
      } else {
         PRINTF("Unable to connect!\n");
         reboot_board(RESET_BY_UNABLE_CONNECT);
      }
   }

   PRINTF("<<<--- Ending process thread 'mqttsn_process'\n");
   processes_running--;
   process_status_array[MQTTSN_PROCESS] = PROCESS_STOPPED;

   PROCESS_END();
}

/********************************** WATCHDOG **********************************/

/**
 * Detects network inactivity and process crash, calling reboot_board()
 */
PROCESS_THREAD(watchdog_process, ev, data) {
   PROCESS_BEGIN();

   process_status_array[WATCHDOG_PROCESS] = PROCESS_RUNNING;
   processes_running++;
   PRINTF("--->>> Starting process thread 'watchdog_process'\n");

   static struct etimer et;

   static uint32_t counter = 0;

   network_inactivity_timeout_reset = process_alloc_event();
   etimer_set(&et, CLOCK_SECOND);

   while (!is_rebooting) {
      PROCESS_WAIT_EVENT();
      if ((ev == PROCESS_EVENT_TIMER) && (counter >= INACTIVITY_TIMEOUT)) {
         if (etimer_expired(&et)) {
            PRINTF("\n***** INACTIVITY DETECTED *****\n\n");
            reboot_board(RESET_BY_INACTIVITY);
            break;
         }
      }
      if (ev == network_inactivity_timeout_reset) {
         etimer_restart(&et);
         counter = 0;
      }
      if ((ready_to_work) && (!test_mode_on) && (!is_all_processes_running())) {
         etimer_stop(&et);
         PRINTF("\n***** PROCESS STOPPED DETECTED *****\n\n");
         reboot_board(RESET_BY_PROCESS_STOPPED);
         break;
      }
      counter++;
   }

   PRINTF("<<<--- Ending process thread 'watchdog_process'\n");
   processes_running--;
   process_status_array[WATCHDOG_PROCESS] = PROCESS_STOPPED;

   PROCESS_END();
}

/**
 * Set leds to notify board will reset and forces watchdor to reset
 */
PROCESS_THREAD(reboot_process, ev, data) {
   PROCESS_BEGIN();

   process_status_array[REBOOT_PROCESS] = PROCESS_RUNNING;
   processes_running++;
   PRINTF("--->>> Starting process thread 'reboot_process'\n");

   static struct etimer et;

   set_red_led(RED_LED_OFF_REBOOTING);
   set_green_led(GREEN_LED_OFF_REBOOTING);
   while (processes_running > 1) {
      etimer_set(&et, CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(ev = PROCESS_EVENT_TIMER);
   }
   leds_off(LEDS_ALL);

   PRINTF("\n***** Watchdog detected network inactivity/disconnection. REBOOTING board...\n\n\n");
   static uint8_t i;
   for (i = 0; i < 7; i++) {
      leds_toggle(LEDS_ALL);
      etimer_set(&et, 0.8 * CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(ev = PROCESS_EVENT_TIMER);
      leds_toggle(LEDS_ALL);
      etimer_set(&et, 0.5 * CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(ev = PROCESS_EVENT_TIMER);
   }
   loop_forever();

   PRINTF("<<<--- Ending process thread 'reboot_process'\n");
   processes_running--;
   process_status_array[REBOOT_PROCESS] = PROCESS_STOPPED;

   PROCESS_END();
}

/************************************ LEDS ************************************/

/**
 * Controls the green led (used to reporting MQTT messaging exchange).
 */
PROCESS_THREAD(green_led_process, ev, data) {
   PROCESS_BEGIN();

   process_status_array[GREEN_LED_PROCESS] = PROCESS_RUNNING;
   processes_running++;
   PRINTF("--->>> Starting process thread 'green_led_process'\n");


   static struct etimer et;
   leds_off(LEDS_GREEN);
   while ((!is_rebooting) && (!test_mode_on)) {
      if (green_led_state == GREEN_LED_SENDING_MESSAGE) {
         leds_off(LEDS_GREEN);
      } else if (green_led_state == GREEN_LED_NO_MESSAGE) {
         leds_on(LEDS_GREEN);
      } else if(green_led_state == GREEN_LED_OFF) {
         leds_off(LEDS_GREEN);
      } else if (green_led_state == GREEN_LED_OFF_REBOOTING) {
         leds_off(LEDS_GREEN);
         break;
      }
      etimer_set(&et, 0.1 * CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(ev = PROCESS_EVENT_TIMER);
   }

   PRINTF("<<<--- Ending process thread 'green_led_process'\n");
   processes_running--;
   process_status_array[GREEN_LED_PROCESS] = PROCESS_STOPPED;

   PROCESS_END();
}

/**
 * Controls the red led (used to reporting MQTT connection status).
 */
PROCESS_THREAD(red_led_process, ev, data) {
   PROCESS_BEGIN();

   process_status_array[RED_LED_PROCESS] = PROCESS_RUNNING;
   processes_running++;
   PRINTF("--->>> Starting process thread 'red_led_process'\n");

   static struct etimer et;
   leds_off(LEDS_RED);
   while ((!is_rebooting) && (!test_mode_on)) {
      if (red_led_state == RED_LED_CONNECTING) {
         leds_toggle(LEDS_RED);
      } else if (red_led_state == RED_LED_CONNECTED) {
         leds_off(LEDS_RED);
      } else if (red_led_state == RED_LED_OFF) {
         leds_off(LEDS_RED);
      } else if (red_led_state == RED_LED_OFF_REBOOTING) {
         leds_off(LEDS_RED);
         break;
      }
      etimer_set(&et, 0.6 * CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(ev = PROCESS_EVENT_TIMER);
   }

   PRINTF("<<<--- Ending process thread 'red_led_process'\n");
   processes_running--;
   process_status_array[RED_LED_PROCESS] = PROCESS_STOPPED;

   PROCESS_END();
}

/********************************** SENSORS ***********************************/

/**
 * Controls the reading status of rain sensor of drain
 */
PROCESS_THREAD(rain_sensor_drain_process, ev, data) {
   PROCESS_BEGIN();

   process_status_array[RAIN_SENSOR_DRAIN_PROCESS] = PROCESS_RUNNING;
   processes_running++;
   PRINTF("--->>> Starting process thread 'rain_sensor_drain_process'\n");

   static struct etimer et;

   static bool water_detected = false, reported_last_rain = false, report_water_on_drain = false;
   static uint32_t counter = 0;
   static uint16_t errors_counter = 0;

   while ((!is_rebooting) && (is_connected)) {

      uint32_t value_read = readADSSensor(RAIN_ON_DRAIN_SENSOR);
      if (value_read == 0) {
         errors_counter++;
      }
      if (errors_counter >= ((
            test_mode_on ?
            MIN_INTERVAL_REPORT_WATER_ON_DRAIN_ON_TEST : MIN_INTERVAL_REPORT_WATER_ON_DRAIN)
            / READING_INTERVAL_RAIN_ON_DRAIN_SENSOR)) {
         errors_counter = 0;
         publish_sensor_status(TOPIC_RAIN_SENSOR_DRAIN, READING_ERROR_STATUS);
         last_reading_ok[0] = SENSOR_READING_ERROR;
         PRINTF("##### Rain Sensor on Drain - ERROR detected!\n");
      } else if (value_read >= MIN_VALUE_ACCEPTED_ADS) {
         last_reading_ok[0] = SENSOR_READING_OK;
         water_detected = (value_read < RAIN_ON_DRAIN_DETECTION_MAX_VALUE);
         if (water_detected) {
            if ((interruption_counter > 0) && (!reported_last_rain)) {
               reported_last_rain = true;
               report_water_on_drain = true;
            } else if (interruption_counter == 0) {
               reported_last_rain = false;
               if (counter > ((
                     test_mode_on ?
                     MIN_INTERVAL_REPORT_WATER_ON_DRAIN_ON_TEST : MIN_INTERVAL_REPORT_WATER_ON_DRAIN)
                     / READING_INTERVAL_RAIN_ON_DRAIN_SENSOR)) {
                  report_water_on_drain = true;
               }
            }
            if (report_water_on_drain) {
               PRINTF("##### Rain Sensor on Drain - Water detected!\n");
               report_water_on_drain = false;
               publish_sensor_status(TOPIC_RAIN_SENSOR_DRAIN, WATER_ON_DRAIN);
               counter = 0;
            }
         }
      }

      etimer_set(&et, READING_INTERVAL_RAIN_ON_DRAIN_SENSOR * CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER && data == &et);
      counter++;
   }

   PRINTF("<<<--- Ending process thread 'rain_sensor_drain_process'\n");
   processes_running--;
   process_status_array[RAIN_SENSOR_DRAIN_PROCESS] = PROCESS_STOPPED;

   PROCESS_END();
}

/**
 * Controls the reading status of the capacitive soil moisture and temperature sensors.
 */
PROCESS_THREAD(moisture_sensor_process, ev, data) {
   PROCESS_BEGIN();

   process_status_array[MOISTURE_SENSOR_PROCESS] = PROCESS_RUNNING;
   processes_running++;
   PRINTF("--->>> Starting process thread 'moisture_sensor_process'\n");

   static struct etimer et;

   static bool first_temp_reading = true;
   static uint32_t counter = 0;

   while ((!is_rebooting) && (is_connected)) {
      if (counter >= (test_mode_on ? TEST_MODE_READING_INTERVAL : MOISTURE_TEMP_READ_INTERVAL)) {
         counter = 0;
         uint32_t moistureSensorRead = readADSSensor(MOISTURE_SENSOR);
         int temperatureSensorRead = readTemperatureSensor();
         if (moistureSensorRead == 0) {
            last_reading_ok[1] = SENSOR_READING_ERROR;
         } else {
            last_reading_ok[1] = SENSOR_READING_OK;
         }
         if (temperatureSensorRead == 0) {
            last_reading_ok[2] = SENSOR_READING_ERROR;
         } else {
            last_reading_ok[2] = SENSOR_READING_OK;
         }
         if ((first_temp_reading) && (temperatureSensorRead == 8500)) {
            first_temp_reading = false;
         } else {
            PRINTF("##### Moisture sensor - value read: %li ##### Temperature sensor - value read: %i\n", moistureSensorRead, temperatureSensorRead);
            publish_sensor_status(TOPIC_CAPACITIVE_SOIL_MOISTURE_SENSOR, moistureSensorRead);
            publish_sensor_status(TOPIC_TEMPERATURE_SENSOR, temperatureSensorRead);
         }
      }
      etimer_set(&et, CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
      counter++;
   }

   PRINTF("<<<--- Ending process thread 'moisture_sensor_process'\n");
   processes_running--;
   process_status_array[MOISTURE_SENSOR_PROCESS] = PROCESS_STOPPED;

   PROCESS_END();
}

/**
 * Controls the reading status of the interruption sensor (pluviometer or optical rain sensor).
 */
PROCESS_THREAD(interruption_sensor_process, ev, data) {
   PROCESS_BEGIN();

   process_status_array[INTERRUPTION_SENSOR_PROCESS] = PROCESS_RUNNING;
   PRINTF("--->>> Starting process thread 'interruption_sensor_process'\n");
   processes_running++;

   SENSORS_ACTIVATE(interruption_sensor);

   while ((!is_rebooting) && (is_connected)) {
      PROCESS_YIELD();
      if(ev == sensors_event) {
          if(data == &interruption_sensor) {
             interruption_counter++;
             if (is_pluviometer_installed) {
                PRINTF("##### Pluviometric Sensor event received! Counter: %u.\n", interruption_counter);
                publish_sensor_status(TOPIC_PLUVIOMETER, interruption_counter);
             } else {
                PRINTF("##### Optical Rain Sensor event received! Counter: %u.\n", interruption_counter);
                publish_sensor_status(TOPIC_OPTICAL_RAIN_SENSOR, interruption_counter);
             }
             process_post(&interruption_sensor_reset_interval_process, interruption_sensor_tic_event, NULL);
          }
      }
   }

   SENSORS_DEACTIVATE(interruption_sensor);

   PRINTF("<<<--- Ending process thread 'interruption_sensor_process'\n");
   processes_running--;
   process_status_array[INTERRUPTION_SENSOR_PROCESS] = PROCESS_STOPPED;

   PROCESS_END();
}

/**
 * Resets interruption sensor counter when time is reached
 */
PROCESS_THREAD(interruption_sensor_reset_interval_process, ev, data) {
   PROCESS_BEGIN();

   process_status_array[INTERRUPTION_SENSOR_RESET_INTERVAL_PROCESS] = PROCESS_RUNNING;
   processes_running++;
   PRINTF("--->>> Starting process thread 'interruption_sensor_reset_interval_process'\n");

   static struct etimer et;
   static uint32_t counter = 0;
   interruption_sensor_tic_event = process_alloc_event();

   etimer_set(&et, CLOCK_SECOND);
   while ((!is_rebooting) && (is_connected)) {
      counter++;
      PROCESS_WAIT_EVENT();
      if (ev == interruption_sensor_tic_event) {
         counter = 0;
      }
      if (ev == PROCESS_EVENT_TIMER) {
         if (counter >= (test_mode_on ?
                  INTERRUPTION_SENSOR_WAIT_INTERVAL_TO_RESET_ON_TEST :
                  INTERRUPTION_SENSOR_WAIT_INTERVAL_TO_RESET)) {
            if (interruption_counter > 0) {
               interruption_counter = 0;
               if (is_pluviometer_installed) {
                  PRINTF("##### Pluviometric Sensor - value reset.\n");
                  publish_sensor_status(TOPIC_PLUVIOMETER, interruption_counter);
               } else {
                  PRINTF("##### Optical Rain Sensor - value reset.\n");
                  publish_sensor_status(TOPIC_OPTICAL_RAIN_SENSOR, interruption_counter);
               }
            }
            counter = 0;
         }
      }
      etimer_restart(&et);
   }

   PRINTF("<<<--- Ending process thread 'interruption_sensor_reset_interval_process'\n");
   processes_running--;
   process_status_array[INTERRUPTION_SENSOR_RESET_INTERVAL_PROCESS] = PROCESS_STOPPED;

   PROCESS_END();
}

/********************************** GENERAL ***********************************/

/**
 * Sends command requesting timestamp update every day
 */
PROCESS_THREAD(request_timestamp_update_process, ev, data) {
   PROCESS_BEGIN();

   process_status_array[REQUEST_TIMESTAMP_UPDATE_PROCESS] = PROCESS_RUNNING;
   processes_running++;
   PRINTF("--->>> Starting process thread 'request_timestamp_update_process'\n");


   static struct etimer et;
   static uint32_t counter = 0;

   while ((!is_rebooting) && (is_connected)) {
      etimer_set(&et, CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
      counter++;
      if (counter >= TIMESTAMP_UPDATE_REQUEST_INTERVAL) {
         publish_board_status(BOARD_STATUS_TIMESTAMP_UPDATE_REQUEST);
         counter = 0;
      }
   }

   PRINTF("<<<--- Ending process thread 'request_timestamp_update_process'\n");
   processes_running--;
   process_status_array[REQUEST_TIMESTAMP_UPDATE_PROCESS] = PROCESS_STOPPED;

   PROCESS_END();
}

PROCESS_THREAD(report_board_general_status_process, ev, data) {
   PROCESS_BEGIN();

   process_status_array[REPORT_BOARD_GENERAL_STATUS_PROCESS] = PROCESS_RUNNING;
   processes_running++;
   PRINTF("--->>> Starting process thread 'report_board_general_status_process'\n");

   static struct etimer et;
   static uint32_t counter = 0;

   while ((!is_rebooting) && (is_connected)) {
      etimer_set(&et, CLOCK_SECOND);
      PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
      counter++;
      if (counter >= (test_mode_on ? TEST_MODE_READING_INTERVAL : REPORT_BOARD_GENERAL_STATUS_INTERVAL)) {
         counter = 0;
         report_board_status(NORMAL_BOARD_STATUS);
      }
   }

   PRINTF("<<<--- Ending process thread 'report_board_general_status_process'\n");
   processes_running--;
   process_status_array[REPORT_BOARD_GENERAL_STATUS_PROCESS] = PROCESS_STOPPED;

   PROCESS_END();
}

PROCESS_THREAD(detect_test_mode_process, ev, data) {
   PROCESS_BEGIN();

   process_status_array[DETECT_TEST_MODE_PROCESS] = PROCESS_RUNNING;
   PRINTF("--->>> Starting process thread 'detect_test_mode_process'\n");
   processes_running++;

   test_mode_activation_ended = process_alloc_event();

   SENSORS_ACTIVATE(button_sensor);

   while ((!is_rebooting) && (test_mode_available)) {
      PROCESS_YIELD();
      if ((ev == sensors_event) && ((data == &button_left_sensor) || (data == &button_right_sensor))) {
         PRINTF("\n***** ***** TEST MODE ON ***** *****\n\n");
         test_mode_on = true;
         break;
      }
      if ((ev == test_mode_activation_ended) && (!test_mode_available)) {
         PRINTF("!!!!! Test mode activation ended.\n");
      }
   }

   SENSORS_DEACTIVATE(button_sensor);

   if (test_mode_on) {

      static uint8_t leds_array[4][10] = {
         {1, 1, 1, 1, 1, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 1, 1, 1, 1, 1},
         {1, 1, 0, 1, 1, 0, 0, 0, 0, 0},
         {0, 0, 0, 0, 0, 1, 1, 0, 1, 1}
      };

      static uint8_t counter = 0, led_green, led_red;

      static struct etimer et;

      while (!is_rebooting) {
         led_red = leds_array[(test_mode_available ? 0 : 2)][counter];
         led_green = leds_array[(test_mode_available ? 1 : 3)][counter];

         if (led_red) {
            leds_on(LEDS_RED);
         } else {
            leds_off(LEDS_RED);
         }
         if (led_green) {
            leds_on(LEDS_GREEN);
         } else {
            leds_off(LEDS_GREEN);
         }

         etimer_set(&et, 0.1 * CLOCK_SECOND);
         PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
         counter++;
         if (counter == 10) {
            counter = 0;
         }
      }
   }

   PRINTF("<<<--- Ending process thread 'detect_test_mode_process'\n");
   processes_running--;
   process_status_array[DETECT_TEST_MODE_PROCESS] = PROCESS_STOPPED;

   PROCESS_END();
}
