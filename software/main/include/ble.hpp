/**
 * @file ble.hpp
 * @brief bluetooth low energy binder class
 * @author tomatenkuchen
 * @date 2024-12-29
 * @license GPLv2 @see license.md
 */

#pragma once

#include "gap.hpp"
#include "gatt.hpp"
#include "host/ble_gatt.h"
#include "nimble.hpp"

namespace ble {

class BLE {
  Nimble nimble;
  GAP gap;
  GATT gatt;

public:
  BLE(std::string const &app_name);

  void gatt_service_register_callback(ble_hatt_register_ctxt ctxt, void *arg);
  void nimble_host_task();

  template <typename uuid_type, uint8_t N>
  void send_indication(GATT::Service<uuid_type, N> service);

private:
  void init_nimble_host();
  void prepare_nvm();
  void nimble_host_config_init();
};
} // namespace ble
