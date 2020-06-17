#ifndef SHUTTERS_EVENTS_H
#define SHUTTERS_EVENTS_H

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "event_groups.h"

#define FIRST_STATUS_INFO_SENT_FLAG             (1 << 0)
#define UPDATE_FIRMWARE_FLAG                    (1 << 1)
#define SERVER_IS_DOWN_FLAG                     (1 << 2)
#define STATUS_INFO_IS_BEING_SENT_FLAG          (1 << 3)
#define WIFI_CONNECTED_FLAG                     (1 << 4)
#define ACCESS_POINT_IS_BEING_SCANNED_FLAG      (1 << 5)
#define ESP_EVENT_LOOP_INITIALIZED_FLAG         (1 << 6)
#define TURNED_ON_BY_SWITCHER_FLAG              (1 << 7)
#define TURNED_ON_BY_SERVER_FLAG                (1 << 8)
#define PINS_INTERRUPT_DISABLED_FLAG            (1 << 9)

#define SEND_STATUS_INFO_EVENT                  (1 << 10)
#define SCAN_ACCESS_POINT_EVENT                 (1 << 11)
#define SWITCHER_EVENT                          (1 << 12)

void init_events();
void save_being_updated_event();
bool is_being_updated();
void save_server_is_down_event();
void clear_server_is_down_event();
bool is_server_down();
void save_sending_status_info_event();
void clear_sending_status_info_event();
bool is_status_info_being_sent();
bool is_first_status_info_sent();
void save_first_status_info_sent_event();
void save_connected_to_wifi_event();
void clear_connected_to_wifi_event();
bool is_connected_to_wifi();
void save_access_point_scanning_event();
void clear_access_point_scanning_event();
bool is_access_point_is_being_scanned();
void save_pin_interrupt_was_initialized_event();
bool was_pin_interrupt_initialized();
void save_turned_on_by_switcher_event();
void clear_turned_on_by_switcher_event();
bool is_turned_on_by_switcher();
void save_turned_on_by_server_event();
void clear_turned_on_by_server_event();
bool is_turned_on_by_server();
void save_pins_interrupt_disabled_event();
bool is_pins_interrupt_disabled();
void clear_pins_interrupt_disabled();

#endif