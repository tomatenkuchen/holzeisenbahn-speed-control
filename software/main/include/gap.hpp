#pragma once

#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"
#include <string>
#include <string_view>

namespace gap {

constexpr std::string TAG = "gap";

class Gap {
public:
  Gap(std::string device_name);

  void advertizing_start();

private:
  uint8_t own_addr_type;
  uint8_t addr_val[6] = {0};
  std::string_view esp_uri = "\x17//espressif.com";

  void start_advertising();

  /* NimBLE applies an event-driven model to keep GAP service going
   * gap_event_handler is a callback function registered when calling
   * ble_gap_adv_start API and called when a GAP event arrives */
  int gap_event_handler(struct ble_gap_event *event, void *arg);

  void format_addr(char *addr_str, uint8_t addr[]);

  void print_conn_desc(ble_gap_conn_desc *desc);
};

} // namespace gap
