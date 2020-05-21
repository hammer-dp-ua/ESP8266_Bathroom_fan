/**
 * Required connections:
 * GPIO15 to GND
 * GPIO0 to "+" or to "GND" for flashing
 * EN to "+"
 */

#include "main.h"

static unsigned int hundred_milliseconds_counter_g = 0;
static unsigned int hundred_milliseconds_manually_switched_on_fan_time_g = 0;
static int signal_strength_g = 0;
static unsigned short errors_counter_g = 0;
static unsigned char repetitive_request_errors_counter_g = 0;
static unsigned char repetitive_ap_connecting_errors_counter_g = 0;
static unsigned int repetitive_tcp_server_errors_counter_g = 0;
static unsigned char manually_turned_on_fan_working_time_minutes_g = 10;

static esp_timer_handle_t milliseconds_timer_g = NULL;
static esp_timer_handle_t errors_checker_timer_g = NULL;
static esp_timer_handle_t blink_both_leds_timer_g = NULL;
static esp_timer_handle_t status_sender_timer_g = NULL;
static esp_timer_handle_t ap_scanning_timer_g = NULL;
static esp_timer_handle_t anti_contact_bounce_timer_g = NULL;
static esp_timer_handle_t fan_turn_off_timer_manual_switch_g = NULL;
static esp_timer_handle_t fan_turn_off_timer_server_down_g = NULL;

static SemaphoreHandle_t wirelessNetworkActionsSemaphore_g = NULL;
static xQueueHandle network_events_queue_g = NULL;

static void hundred_milliseconds_counter_cb() {
   hundred_milliseconds_counter_g++;
}

static void start_100_milliseconds_counter() {
   esp_timer_create_args_t timer_config = {
         .callback = &hundred_milliseconds_counter_cb
   };

   ESP_ERROR_CHECK(esp_timer_create(&timer_config, &milliseconds_timer_g))
   ESP_ERROR_CHECK(esp_timer_start_periodic(milliseconds_timer_g, (1000 / MILLISECONDS_COUNTER_DIVIDER) * 1000)) // 1000/10 = 100 ms
}

static void scan_access_point_task() {
   unsigned int rescan_when_not_connected_delay = 30 * 1000 / portTICK_PERIOD_MS; // 30 secs
   wifi_scan_config_t scan_config;
   unsigned short scanned_access_points_amount = 1;
   wifi_ap_record_t scanned_access_points[1];

   scan_config.ssid = (unsigned char *) ACCESS_POINT_NAME;
   scan_config.bssid = 0;
   scan_config.channel = 0;
   scan_config.show_hidden = false;

   for (;;) {
      if (!is_connected_to_wifi()) {
         vTaskDelay(rescan_when_not_connected_delay);
         continue;
      }

      #ifdef ALLOW_USE_PRINTF
      printf("\nStart of Wi-Fi scanning... %u\n", hundred_milliseconds_counter_g);
      #endif

      xSemaphoreTake(wirelessNetworkActionsSemaphore_g, portMAX_DELAY);

      bool scanned;

      if (esp_wifi_scan_start(&scan_config, true) == ESP_OK &&
            esp_wifi_scan_get_ap_records(&scanned_access_points_amount, scanned_access_points) == ESP_OK) {
         signal_strength_g = scanned_access_points[0].rssi;
         scanned = true;

         #ifdef ALLOW_USE_PRINTF
         printf("Scanned %u access points", scanned_access_points_amount);
         for (unsigned char i = 0; i < scanned_access_points_amount; i++) {
            printf("\nScan index: %u, ssid: %s, rssi: %d", i, scanned_access_points[i].ssid, scanned_access_points[i].rssi);
         }
         printf("\n");
         #endif
      } else {
         #ifdef ALLOW_USE_PRINTF
         printf("Wi-Fi scanning skipped. %u\n", hundred_milliseconds_counter_g);
         #endif

         scanned = false;
      }

      xSemaphoreGive(wirelessNetworkActionsSemaphore_g);

      if (scanned) {
         break;
      } else {
         vTaskDelay(rescan_when_not_connected_delay);
      }
   }

   clear_access_point_scanning_event();
   vTaskDelete(NULL);
}

static void blink_both_leds_cb() {
   if (gpio_get_level(AP_CONNECTION_STATUS_LED_PIN)) {
      gpio_set_level(AP_CONNECTION_STATUS_LED_PIN, 0);
      gpio_set_level(SERVER_AVAILABILITY_STATUS_LED_PIN, 1);
   } else {
      gpio_set_level(AP_CONNECTION_STATUS_LED_PIN, 1);
      gpio_set_level(SERVER_AVAILABILITY_STATUS_LED_PIN, 0);
   }
}

static void start_both_leds_blinking(unsigned int ms) {
   esp_timer_create_args_t timer_config = {
         .callback = &blink_both_leds_cb
   };

   ESP_ERROR_CHECK(esp_timer_create(&timer_config, &blink_both_leds_timer_g))
   ESP_ERROR_CHECK(esp_timer_start_periodic(blink_both_leds_timer_g, ms * 1000))
}

static void stop_both_leds_blinking() {
   ESP_ERROR_CHECK(esp_timer_stop(blink_both_leds_timer_g))
   ESP_ERROR_CHECK(esp_timer_delete(blink_both_leds_timer_g))

   gpio_set_level(AP_CONNECTION_STATUS_LED_PIN, 0);
   gpio_set_level(SERVER_AVAILABILITY_STATUS_LED_PIN, 0);
}

static void blink_on_send(gpio_num_t pin) {
   int initial_pin_state = gpio_get_level(pin);
   unsigned char i;

   for (i = 0; i < 4; i++) {
      bool set_pin = initial_pin_state == 1 ? i % 2 == 1 : i % 2 == 0;

      if (set_pin) {
         gpio_set_level(pin, 1);
      } else {
         gpio_set_level(pin, 0);
      }
      vTaskDelay(100 / portTICK_PERIOD_MS);
   }

   if (pin == AP_CONNECTION_STATUS_LED_PIN) {
      if (is_connected_to_wifi()) {
         gpio_set_level(pin, 1);
      } else {
         gpio_set_level(pin, 0);
      }
   }
}

static void on_response_ok() {
   clear_server_is_down_event();
   repetitive_request_errors_counter_g = 0;
   gpio_set_level(SERVER_AVAILABILITY_STATUS_LED_PIN, 1);
}

static void on_response_error() {
   repetitive_request_errors_counter_g++;
   errors_counter_g++;
   gpio_set_level(SERVER_AVAILABILITY_STATUS_LED_PIN, 0);
   save_server_is_down_event();
}

static void send_status_info_task() {
   xSemaphoreTake(wirelessNetworkActionsSemaphore_g, portMAX_DELAY);
   blink_on_send(SERVER_AVAILABILITY_STATUS_LED_PIN);

   char signal_strength[5];
   snprintf(signal_strength, 5, "%d", signal_strength_g);
   char errors_counter[6];
   snprintf(errors_counter, 6, "%u", errors_counter_g);
   char uptime[11];
   snprintf(uptime, 11, "%u", hundred_milliseconds_counter_g / MILLISECONDS_COUNTER_DIVIDER);
   char *build_timestamp = "";
   char free_heap_space[7];
   snprintf(free_heap_space, 7, "%u", esp_get_free_heap_size());
   char *reset_reason = "";
   char *system_restart_reason = "";
   char *switchedOnManually = is_turned_on_by_switcher() ? "true" : "false";

   char switched_on_manually_seconds_left_param[6];
   unsigned int switched_on_manually_seconds_left = 0;
   if (is_turned_on_by_switcher() && !is_turned_on_by_server() &&
         hundred_milliseconds_manually_switched_on_fan_time_g > 0 &&
         hundred_milliseconds_counter_g > hundred_milliseconds_manually_switched_on_fan_time_g) {
      unsigned int seconds_passed =
            (hundred_milliseconds_counter_g - hundred_milliseconds_manually_switched_on_fan_time_g) / 10;
      switched_on_manually_seconds_left = manually_turned_on_fan_working_time_minutes_g * 60 - seconds_passed;
   }
   snprintf(switched_on_manually_seconds_left_param, 6, "%u", switched_on_manually_seconds_left);

   float temperature = 0.0F;
   char temperature_param[10];
   temperature_param[0] = '\0';
   char temperature_raw_param[6];
   temperature_raw_param[0] = '\0';
   unsigned short temperature_raw = 0;
   i2c_master_init();
   sht21_get_temperature(&temperature, &temperature_raw);
   i2c_master_deinit();
   snprintf(temperature_raw_param, 6, "%u", temperature_raw);
   snprintf(temperature_param, 10, "%d.%u", (int) temperature, abs((int) (temperature * 100)) - (abs((int) (temperature)) * 100));

   float humidity = 0.0F;
   char humidity_param[10];
   humidity_param[0] = '\0';
   i2c_master_init();
   sht21_get_humidity(&humidity);
   i2c_master_deinit();
   snprintf(humidity_param, 10, "%u.%u", (unsigned int) humidity, abs((int) (humidity * 100)) - (abs((int) (humidity)) * 100));

   if (!is_first_status_info_sent()) {
      char build_timestamp_filled[30];
      snprintf(build_timestamp_filled, 30, "%s", __TIMESTAMP__);
      build_timestamp = build_timestamp_filled;

      esp_reset_reason_t rst_info = esp_reset_reason();

      switch (rst_info) {
         case ESP_RST_UNKNOWN:
            reset_reason = "Unknown";
            break;
         case ESP_RST_POWERON:
            reset_reason = "Power on";
            break;
         case ESP_RST_EXT:
            reset_reason = "Reset by external pin";
            break;
         case ESP_RST_SW:
            reset_reason = "Software";
            break;
         case ESP_RST_PANIC:
            reset_reason = "Exception/panic";
            break;
         case ESP_RST_INT_WDT:
            reset_reason = "Watchdog";
            break;
         case ESP_RST_TASK_WDT:
            reset_reason = "Task watchdog";
            break;
         case ESP_RST_WDT:
            reset_reason = "Other watchdog";
            break;
         case ESP_RST_DEEPSLEEP:
            reset_reason = "Deep sleep";
            break;
         case ESP_RST_BROWNOUT:
            reset_reason = "Brownout";
            break;
         case ESP_RST_SDIO:
            reset_reason = "SDIO";
            break;
         default:
            break;
      }

      SYSTEM_RESTART_REASON_TYPE system_restart_reason_type;

      rtc_mem_read(SYSTEM_RESTART_REASON_TYPE_RTC_ADDRESS, &system_restart_reason_type, 4);

      if (system_restart_reason_type == ACCESS_POINT_CONNECTION_ERROR) {
         int connection_error_code = 1;
         char system_restart_reason_inner[31];

         rtc_mem_read(CONNECTION_ERROR_CODE_RTC_ADDRESS, &connection_error_code, 4);

         snprintf(system_restart_reason_inner, 31, "AP connections error. Code: %d", connection_error_code);
         system_restart_reason = system_restart_reason_inner;
      } else if (system_restart_reason_type == REQUEST_CONNECTION_ERROR) {
         int connection_error_code = 1;
         char system_restart_reason_inner[25];

         rtc_mem_read(CONNECTION_ERROR_CODE_RTC_ADDRESS, &connection_error_code, 4);

         snprintf(system_restart_reason_inner, 25, "Requests error. Code: %d", connection_error_code);
         system_restart_reason = system_restart_reason_inner;
      } else if (system_restart_reason_type == SOFTWARE_UPGRADE) {
         system_restart_reason = "Software upgrade";
      }
   }

   const char *request_payload_template_parameters[] =
         {signal_strength, DEVICE_NAME, errors_counter, uptime, build_timestamp, free_heap_space, reset_reason,
          system_restart_reason, temperature_param, temperature_raw_param, humidity_param, switchedOnManually, switched_on_manually_seconds_left_param, NULL};
   char *request_payload = set_string_parameters(STATUS_INFO_REQUEST_PAYLOAD_TEMPLATE, request_payload_template_parameters);

   unsigned short request_payload_length = strnlen(request_payload, 0xFFFF);
   char request_payload_length_string[6];
   snprintf(request_payload_length_string, 6, "%u", request_payload_length);
   const char *request_template_parameters[] = {request_payload_length_string, SERVER_IP_ADDRESS, request_payload, NULL};
   char *request = set_string_parameters(STATUS_INFO_POST_REQUEST, request_template_parameters);
   FREE(request_payload);

   #ifdef ALLOW_USE_PRINTF
   printf("\nCreated Info Request: %s\nTime: %u\n", request, hundred_milliseconds_counter_g);
   #endif

   char *response = send_request(request, 512, &hundred_milliseconds_counter_g);

   FREE(request);

   if (response == NULL) {
      on_response_error();
   } else {
      if (strstr(response, RESPONSE_SERVER_SENT_OK)) {
         on_response_ok();

         if (!is_first_status_info_sent()) {
            save_first_status_info_sent_event();

            unsigned int overwrite_value = 0xFFFF;
            rtc_mem_write(SYSTEM_RESTART_REASON_TYPE_RTC_ADDRESS, &overwrite_value, 4);
         }

         #ifdef ALLOW_USE_PRINTF
         printf("Status Info Response OK. Time: %u\n", hundred_milliseconds_counter_g);
         #endif

         if (strstr(response, UPDATE_FIRMWARE)) {
            save_being_updated_event();

            SYSTEM_RESTART_REASON_TYPE reason = SOFTWARE_UPGRADE;
            rtc_mem_write(SYSTEM_RESTART_REASON_TYPE_RTC_ADDRESS, &reason, 4);

            update_firmware(AP_CONNECTION_STATUS_LED_PIN, SERVER_AVAILABILITY_STATUS_LED_PIN);
         }

         bool is_numeric_param;
         char *manually_turned_on_timeout_setting = get_gson_element_value(response, "manuallyTurnedOnTimeoutSetting",
               &is_numeric_param, hundred_milliseconds_counter_g);

         if (manually_turned_on_timeout_setting != NULL) {
            if (is_numeric_param) {
               int manually_turned_on_timeout_setting_numeric = atoi(manually_turned_on_timeout_setting);
               if (manually_turned_on_timeout_setting_numeric > 0) {
                  manually_turned_on_fan_working_time_minutes_g = manually_turned_on_timeout_setting_numeric;
               }
            }
            FREE(manually_turned_on_timeout_setting);
         }

         if (strstr(response, TURN_ON)) {
            save_turned_on_by_server_event();
            gpio_set_level(RELAY_PIN, 1);
         } else {
            clear_turned_on_by_server_event();
            // Don't turn off if turned on by switcher
            if (!is_turned_on_by_switcher()) {
               gpio_set_level(RELAY_PIN, 0);
            }
         }
      } else {
         on_response_error();
      }

      FREE(response);
   }

   #ifdef ALLOW_USE_PRINTF
   printf("send_status_info_task to be deleted. Time: %u\n", hundred_milliseconds_counter_g);
   #endif

   clear_sending_status_info_event();
   xSemaphoreGive(wirelessNetworkActionsSemaphore_g);
   vTaskDelete(NULL);
}

static void send_sending_status_info_event_cb() {
   unsigned int event = SEND_STATUS_INFO_EVENT;
   xQueueSendToBack(network_events_queue_g, &event, portMAX_DELAY);
}

static void schedule_sending_status_info(unsigned int timeout_ms) {
   esp_timer_create_args_t timer_config = {
         .callback = &send_sending_status_info_event_cb
   };

   ESP_ERROR_CHECK(esp_timer_create(&timer_config, &status_sender_timer_g))
   ESP_ERROR_CHECK(esp_timer_start_periodic(status_sender_timer_g, timeout_ms * 1000))
}

static void pins_config() {
   gpio_config_t output_pins;
   output_pins.mode = GPIO_MODE_OUTPUT;
   output_pins.pin_bit_mask = (1<<AP_CONNECTION_STATUS_LED_PIN) | (1<<SERVER_AVAILABILITY_STATUS_LED_PIN) |
         (1<<RELAY_PIN);
   output_pins.pull_up_en = GPIO_PULLUP_DISABLE;
   output_pins.pull_down_en = GPIO_PULLDOWN_DISABLE;
   ESP_ERROR_CHECK(gpio_config(&output_pins))
   gpio_set_level(AP_CONNECTION_STATUS_LED_PIN, 0);
   gpio_set_level(SERVER_AVAILABILITY_STATUS_LED_PIN, 0);
   gpio_set_level(RELAY_PIN, 0);

   gpio_config_t input_pins;
   input_pins.mode = GPIO_MODE_INPUT;
   input_pins.pin_bit_mask = (1<<SWITCHER_INPUT_PIN);
   input_pins.pull_up_en = GPIO_PULLUP_ENABLE;
   input_pins.intr_type = GPIO_INTR_ANYEDGE;
   ESP_ERROR_CHECK(gpio_config(&input_pins))
}

static void resume_pins_interrupts_cb() {
   ESP_ERROR_CHECK(gpio_set_intr_type(SWITCHER_INPUT_PIN, GPIO_INTR_ANYEDGE))

   #ifdef ALLOW_USE_PRINTF
   printf("\nPins interrupts resumed. Time: %u\n", hundred_milliseconds_counter_g);
   #endif
}

static void gpio_isr_handler(void *arg) {
   if (was_pin_interrupt_initialized()) {
      ESP_ERROR_CHECK(gpio_set_intr_type(SWITCHER_INPUT_PIN, GPIO_INTR_DISABLE))
      ESP_ERROR_CHECK(esp_timer_start_once(anti_contact_bounce_timer_g, 5 * 1000 * 1000)) // resume interrupts after 5s

      unsigned int event = SWITCHER_EVENT;
      xQueueSendToBackFromISR(network_events_queue_g, &event, NULL);
   } else {
      save_pin_interrupt_was_initialized_event();
   }
}

static void pins_isr_config() {
   esp_timer_create_args_t timer_config = {
         .callback = &resume_pins_interrupts_cb
   };
   ESP_ERROR_CHECK(esp_timer_create(&timer_config, &anti_contact_bounce_timer_g))

   // change GPIO interrupt type for one pin
   ESP_ERROR_CHECK(gpio_set_intr_type(SWITCHER_INPUT_PIN, GPIO_INTR_ANYEDGE))
   ESP_ERROR_CHECK(gpio_install_isr_service(0))
   // The handler will be invoked immediately after added
   ESP_ERROR_CHECK(gpio_isr_handler_add(SWITCHER_INPUT_PIN, gpio_isr_handler, NULL))
}

static void on_wifi_connected() {
   gpio_set_level(AP_CONNECTION_STATUS_LED_PIN, 1);
   repetitive_ap_connecting_errors_counter_g = 0;

   send_sending_status_info_event_cb();
}

static void on_wifi_disconnected() {
   repetitive_ap_connecting_errors_counter_g++;
   gpio_set_level(AP_CONNECTION_STATUS_LED_PIN, 0);
   gpio_set_level(SERVER_AVAILABILITY_STATUS_LED_PIN, 0);
}

static void blink_on_wifi_connection_task() {
   blink_on_send(AP_CONNECTION_STATUS_LED_PIN);
   vTaskDelete(NULL);
}

static void blink_on_wifi_connection() {
   xTaskCreate(blink_on_wifi_connection_task, "blink_on_wifi_connection_task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}

/**
 * Created as a workaround to handle unknown issues.
 */
void check_errors_amount_cb() {
   bool restart = false;

   if (repetitive_request_errors_counter_g >= MAX_REPETITIVE_ALLOWED_ERRORS_AMOUNT) {
      #ifdef ALLOW_USE_PRINTF
      printf("\nRequest errors amount: %u\n", repetitive_request_errors_counter_g);
      #endif

      SYSTEM_RESTART_REASON_TYPE system_restart_reason_type = REQUEST_CONNECTION_ERROR;

      rtc_mem_write(SYSTEM_RESTART_REASON_TYPE_RTC_ADDRESS, &system_restart_reason_type, 4);
      restart = true;
   } else if (repetitive_ap_connecting_errors_counter_g >= MAX_REPETITIVE_ALLOWED_ERRORS_AMOUNT) {
      #ifdef ALLOW_USE_PRINTF
      printf("\nAP connection errors amount: %u\n", repetitive_ap_connecting_errors_counter_g);
      #endif

      SYSTEM_RESTART_REASON_TYPE system_restart_reason_type = ACCESS_POINT_CONNECTION_ERROR;

      rtc_mem_write(SYSTEM_RESTART_REASON_TYPE_RTC_ADDRESS, &system_restart_reason_type, 4);
      restart = true;
   } else if (repetitive_tcp_server_errors_counter_g >= MAX_REPETITIVE_ALLOWED_ERRORS_AMOUNT + 10) {
      #ifdef ALLOW_USE_PRINTF
      printf("\nTCP server errors amount: %u\n", repetitive_ap_connecting_errors_counter_g);
      #endif

      SYSTEM_RESTART_REASON_TYPE system_restart_reason_type = TCP_SERVER_ERROR;

      rtc_mem_write(SYSTEM_RESTART_REASON_TYPE_RTC_ADDRESS, &system_restart_reason_type, 4);
      restart = true;
   }

   if (restart) {
      esp_restart();
   }
}

static void schedule_errors_checker(unsigned int timeout_ms) {
   esp_timer_create_args_t timer_config = {
         .callback = &check_errors_amount_cb
   };

   ESP_ERROR_CHECK(esp_timer_create(&timer_config, &errors_checker_timer_g))
   ESP_ERROR_CHECK(esp_timer_start_periodic(errors_checker_timer_g, timeout_ms * 1000L))
}

static void send_access_point_scanning_event_cb() {
   unsigned int event = SCAN_ACCESS_POINT_EVENT;
   xQueueSendToBack(network_events_queue_g, &event, portMAX_DELAY);
}

static void schedule_access_point_scanning(unsigned int timeout_ms) {
   esp_timer_create_args_t timer_config = {
         .callback = &send_access_point_scanning_event_cb
   };

   ESP_ERROR_CHECK(esp_timer_create(&timer_config, &ap_scanning_timer_g))
   ESP_ERROR_CHECK(esp_timer_start_periodic(ap_scanning_timer_g, timeout_ms * 1000L))
}

static void manually_fan_turned_on_timeout_cb() {
   clear_turned_on_by_switcher_event();

   if (!is_turned_on_by_server() || is_server_down()) {
      gpio_set_level(RELAY_PIN, 0);
   }
   hundred_milliseconds_manually_switched_on_fan_time_g = 0;
}

static void schedule_fan_turning_off(esp_timer_handle_t timer, void (*callback)()) {
   if (timer != NULL) {
      ESP_ERROR_CHECK(esp_timer_stop(timer))
      ESP_ERROR_CHECK(esp_timer_delete(timer))
   }

   esp_timer_create_args_t timer_config = {
         .callback = callback
   };

   ESP_ERROR_CHECK(esp_timer_create(&timer_config, &timer))
   unsigned long timeout_us = ((unsigned long) manually_turned_on_fan_working_time_minutes_g) * 60L * 1000L * 1000L;
   ESP_ERROR_CHECK(esp_timer_start_once(timer, timeout_us))
}

static void event_processing_task() {
   unsigned int event;

   for (;;) {
      if (xQueueReceive(network_events_queue_g, &event, portMAX_DELAY)) {
         if (is_being_updated()) {
            vTaskDelete(NULL);
         }

         if (event == SEND_STATUS_INFO_EVENT) {
            if (!is_connected_to_wifi() || is_status_info_being_sent()) {
               continue;
            }

            save_sending_status_info_event();
            xTaskCreate(send_status_info_task, SEND_STATUS_INFO_TASK_NAME, configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
         } else if (event == SWITCHER_EVENT) {
            if (!is_turned_on_by_server()) {
               if (is_turned_on_by_switcher()) {
                  gpio_set_level(RELAY_PIN, 0);
                  clear_turned_on_by_switcher_event();
                  hundred_milliseconds_manually_switched_on_fan_time_g = 0;

                  if (fan_turn_off_timer_manual_switch_g != NULL) {
                     ESP_ERROR_CHECK(esp_timer_stop(fan_turn_off_timer_manual_switch_g))
                     ESP_ERROR_CHECK(esp_timer_delete(fan_turn_off_timer_manual_switch_g))
                  }
               } else {
                  gpio_set_level(RELAY_PIN, 1);
                  save_turned_on_by_switcher_event();
                  schedule_fan_turning_off(fan_turn_off_timer_manual_switch_g, manually_fan_turned_on_timeout_cb);
                  hundred_milliseconds_manually_switched_on_fan_time_g = hundred_milliseconds_counter_g;
               }
            }
         } else if (event == SCAN_ACCESS_POINT_EVENT) {
            if (is_access_point_is_being_scanned()) {
               continue;
            }

            save_access_point_scanning_event();
            xTaskCreate(scan_access_point_task, "scan_ap_task", configMINIMAL_STACK_SIZE * 3, NULL, 1, NULL);
         }
      }
   }
}

static void i2c_master_init() {
   i2c_config_t conf;
   conf.mode = I2C_MODE_MASTER;
   conf.sda_io_num = I2C_MASTER_SDA_IO;
   conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
   conf.scl_io_num = I2C_MASTER_SCL_IO;
   conf.scl_pullup_en = GPIO_PULLUP_DISABLE;

   ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode));
   ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
}

static void i2c_master_deinit() {
   i2c_driver_delete(I2C_MASTER_NUM);
   gpio_config_t output_pins;
   output_pins.mode = GPIO_MODE_INPUT;
   output_pins.pin_bit_mask = (1<<I2C_MASTER_SCL_IO) | (1<<I2C_MASTER_SDA_IO);
   output_pins.pull_up_en = GPIO_PULLUP_DISABLE;
   output_pins.pull_down_en = GPIO_PULLDOWN_DISABLE;
   gpio_config(&output_pins);
}

void app_main(void) {
   start_100_milliseconds_counter();

   pins_config();

   init_events();
   init_utils(&hundred_milliseconds_counter_g);

   network_events_queue_g = xQueueCreate(10, sizeof(unsigned int));
   xTaskCreate(event_processing_task, "event_processing_task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

   start_both_leds_blinking(100);
   vTaskDelay(5000 / portTICK_PERIOD_MS);
   stop_both_leds_blinking();

   #ifdef ALLOW_USE_PRINTF
   const esp_partition_t *running = esp_ota_get_running_partition();
   printf("\nRunning partition type: label: %s, %d, subtype: %d, offset: 0x%X, size: 0x%X, build timestamp: %s\n",
         running->label, running->type, running->subtype, running->address, running->size, __TIMESTAMP__);
   #endif

   vTaskDelay(1000 / portTICK_PERIOD_MS);
   pins_isr_config();

   wirelessNetworkActionsSemaphore_g = xSemaphoreCreateBinary();
   xSemaphoreGive(wirelessNetworkActionsSemaphore_g);

   wifi_init_sta(on_wifi_connected, on_wifi_disconnected, blink_on_wifi_connection);

   schedule_access_point_scanning(SCAN_ACCESS_POINT_TASK_INTERVAL);
   schedule_errors_checker(ERRORS_CHECKER_INTERVAL_MS);
   schedule_sending_status_info(STATUS_REQUESTS_SEND_INTERVAL_MS);
}
