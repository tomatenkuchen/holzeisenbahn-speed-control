#pragma once

#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"
#include <string>
#include <string_view>

namespace gap {

constexpr std::string TAG = "gap";

class Gap {
  constexpr static std::string_view esp_uri = "\x17//espressif.com";

public:
  Gap(std::string device_name);

  /// start advertizing on demand. stops when connection is established
  void start_advertising();

private:
  uint8_t own_addr_type;
  uint8_t addr_val[6] = {0};
  std::string device_name;
  ble_hs_adv_fields adv_fields;
  ble_hs_adv_fields rsp_fields;
  ble_gap_adv_params adv_params;

  int connect_event(ble_gap_event *event);
  void mtu_event(ble_gap_event *event);
  void subscribe_event(ble_gap_event *event);
  void notify_event(ble_gap_event *event);
  void advertizing_complete_event(ble_gap_event *event);
  void update_event(ble_gap_event *event);
  void disconnect_event(ble_gap_event *event);

  void format_addr(char *addr_str, uint8_t addr[]);

  void print_conn_desc(ble_gap_conn_desc *desc);

  /// NimBLE applies an event-driven model to keep GAP service going
  /// gap_event_handler is a callback function registered when calling
  /// ble_gap_adv_start API and called when a GAP event arrives
  int event_handler(ble_gap_event *event);
};

} // namespace gap
