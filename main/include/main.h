#include "freertos/FreeRTOS.h"
#include "task.h"
#include "nvs_flash.h"
#include "driver/i2c.h"

#include "global_definitions.h"
#include "malloc_logger.h"
#include "ota.h"
#include "sht21.h"

#ifndef MAIN_HEADER
#define MAIN_HEADER

#ifdef TESTING
#define AP_CONNECTION_STATUS_LED_PIN         GPIO_NUM_0
#define SERVER_AVAILABILITY_STATUS_LED_PIN   GPIO_NUM_5
#define RELAY_PIN                            GPIO_NUM_4
#define SWITCHER_INPUT_PIN                   GPIO_NUM_14
#define I2C_MASTER_SDA_IO                    GPIO_NUM_12 // gpio number for I2C master data, D6
#define I2C_MASTER_SCL_IO                    GPIO_NUM_13 // gpio number for I2C master clock, D4
#else
#define AP_CONNECTION_STATUS_LED_PIN         GPIO_NUM_13
#define SERVER_AVAILABILITY_STATUS_LED_PIN   GPIO_NUM_14
#define RELAY_PIN                            GPIO_NUM_12
#define SWITCHER_INPUT_PIN                   GPIO_NUM_4
#define I2C_MASTER_SDA_IO                    GPIO_NUM_0
#define I2C_MASTER_SCL_IO                    GPIO_NUM_2
#endif

#define I2C_MASTER_NUM I2C_NUM_0 // I2C port number for master dev

#define ERRORS_CHECKER_INTERVAL_MS (10 * 1000)
#define STATUS_REQUESTS_SEND_INTERVAL_MS (60 * 1000)
#define SCAN_ACCESS_POINT_TASK_INTERVAL (10 * 60 * 1000) // 10 minutes

#define MILLISECONDS_COUNTER_DIVIDER 10

#define MAX_REPETITIVE_ALLOWED_AP_ERRORS_AMOUNT 15
#define MAX_REPETITIVE_ALLOWED_ERRORS_AMOUNT 10

#define SYSTEM_RESTART_REASON_TYPE_RTC_ADDRESS  64
#define CONNECTION_ERROR_CODE_RTC_ADDRESS       SYSTEM_RESTART_REASON_TYPE_RTC_ADDRESS + 1

typedef enum {
   ACCESS_POINT_CONNECTION_ERROR = 1,
   REQUEST_CONNECTION_ERROR,
   SOFTWARE_UPGRADE,
   TCP_SERVER_ERROR
} SYSTEM_RESTART_REASON_TYPE;

const char SEND_STATUS_INFO_TASK_NAME[] = "send_status_info_task";

const char RESPONSE_SERVER_SENT_OK[] = "\"statusCode\":\"OK\"";
const char STATUS_INFO_POST_REQUEST[] =
      "POST /server/esp8266/bathroomFan HTTP/1.1\r\n"
      "Content-Length: <1>\r\n"
      "Host: <2>\r\n"
      "User-Agent: ESP8266\r\n"
      "Content-Type: application/json\r\n"
      "Connection: close\r\n"
      "Accept: application/json\r\n\r\n"
      "<3>\r\n";
const char STATUS_INFO_REQUEST_PAYLOAD_TEMPLATE[] =
      "{"
      "\"gain\":\"<1>\","
      "\"deviceName\":\"<2>\","
      "\"errors\":<3>,"
      "\"uptime\":<4>,"
      "\"buildTimestamp\":\"<5>\","
      "\"freeHeapSpace\":<6>,"
      "\"resetReason\":\"<7>\","
      "\"systemRestartReason\":\"<8>\","
      "\"temperature\":<9>,"
      "\"temperatureRaw\":<10>,"
      "\"humidity\":<11>,"
      "\"switchedOnManually\":<12>,"
      "\"switchedOnManuallySecondsLeft\":<13>"
      "}";

const char UPDATE_FIRMWARE[] = "\"updateFirmware\":true";
const char TURN_ON[] = "\"turnOn\":true";

static void stop_both_leds_blinking();
static void start_both_leds_blinking(unsigned int ms);
static void i2c_master_init();
static void i2c_master_deinit();
static char* create_request();

#endif