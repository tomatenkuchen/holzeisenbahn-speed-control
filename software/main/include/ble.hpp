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

namespace ble {

class BLE {
public:
  BLE(std::string const &app_name);
  ~BLE();

  void gatt_service_register_callback(ble_hatt_register_ctxt ctxt, void *arg);
  void nimble_host_task();

  template <typename uuid_type, uint8_t N>
  void send_indication(gatt::Service<uuid_type, N> service);

private:
  void init_nimble_host();
  void prepare_nvm();
  void nimble_host_config_init();
};
} // namespace ble
