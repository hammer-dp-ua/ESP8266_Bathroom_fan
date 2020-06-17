#include "events.h"

static volatile EventGroupHandle_t flags_g = NULL;

void init_events() {
   if (flags_g == NULL) {
      flags_g = xEventGroupCreate();
   }
}

void save_being_updated_event() {
   xEventGroupSetBits(flags_g, UPDATE_FIRMWARE_FLAG);
}

bool is_being_updated() {
   return (xEventGroupGetBits(flags_g) & UPDATE_FIRMWARE_FLAG);
}

void save_server_is_down_event() {
   xEventGroupSetBits(flags_g, SERVER_IS_DOWN_FLAG);
}

void clear_server_is_down_event() {
   xEventGroupClearBits(flags_g, SERVER_IS_DOWN_FLAG);
}

bool is_server_down() {
   return xEventGroupGetBits(flags_g) & SERVER_IS_DOWN_FLAG;
}

void save_sending_status_info_event() {
   xEventGroupSetBits(flags_g, STATUS_INFO_IS_BEING_SENT_FLAG);
}

void clear_sending_status_info_event() {
   xEventGroupClearBits(flags_g, STATUS_INFO_IS_BEING_SENT_FLAG);
}

bool is_status_info_being_sent() {
   return xEventGroupGetBits(flags_g) & STATUS_INFO_IS_BEING_SENT_FLAG;
}

void save_first_status_info_sent_event() {
   xEventGroupSetBits(flags_g, FIRST_STATUS_INFO_SENT_FLAG);
}

bool is_first_status_info_sent() {
   return xEventGroupGetBits(flags_g) & FIRST_STATUS_INFO_SENT_FLAG;
}

void save_connected_to_wifi_event() {
   xEventGroupSetBits(flags_g, WIFI_CONNECTED_FLAG);
}

void clear_connected_to_wifi_event() {
   xEventGroupClearBits(flags_g, WIFI_CONNECTED_FLAG);
}

bool is_connected_to_wifi() {
   return xEventGroupGetBits(flags_g) & WIFI_CONNECTED_FLAG;
}

void save_access_point_scanning_event() {
   xEventGroupSetBits(flags_g, ACCESS_POINT_IS_BEING_SCANNED_FLAG);
}

void clear_access_point_scanning_event() {
   xEventGroupClearBits(flags_g, ACCESS_POINT_IS_BEING_SCANNED_FLAG);
}

bool is_access_point_is_being_scanned() {
   return xEventGroupGetBits(flags_g) & ACCESS_POINT_IS_BEING_SCANNED_FLAG;
}

void save_esp_event_loop_initialized_event() {
   xEventGroupSetBits(flags_g, ESP_EVENT_LOOP_INITIALIZED_FLAG);
}

bool is_esp_event_loop_initialized() {
   return xEventGroupGetBits(flags_g) & ESP_EVENT_LOOP_INITIALIZED_FLAG;
}

void save_turned_on_by_switcher_event() {
   xEventGroupSetBits(flags_g, TURNED_ON_BY_SWITCHER_FLAG);
}

void clear_turned_on_by_switcher_event() {
   xEventGroupClearBits(flags_g, TURNED_ON_BY_SWITCHER_FLAG);
}

bool is_turned_on_by_switcher() {
   return xEventGroupGetBits(flags_g) & TURNED_ON_BY_SWITCHER_FLAG;
}

void save_turned_on_by_server_event() {
   xEventGroupSetBits(flags_g, TURNED_ON_BY_SERVER_FLAG);
}

void clear_turned_on_by_server_event() {
   xEventGroupClearBits(flags_g, TURNED_ON_BY_SERVER_FLAG);
}

bool is_turned_on_by_server() {
   return xEventGroupGetBits(flags_g) & TURNED_ON_BY_SERVER_FLAG;
}

void save_pins_interrupt_disabled_event() {
   xEventGroupSetBits(flags_g, PINS_INTERRUPT_DISABLED_FLAG);
}

bool is_pins_interrupt_disabled() {
   return xEventGroupGetBits(flags_g) & PINS_INTERRUPT_DISABLED_FLAG;
}

void clear_pins_interrupt_disabled() {
   xEventGroupClearBits(flags_g, PINS_INTERRUPT_DISABLED_FLAG);
}