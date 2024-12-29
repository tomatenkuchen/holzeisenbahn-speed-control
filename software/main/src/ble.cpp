#include "ble.hpp"
#include "gap.hpp"
#include "gatt.hpp"
#include "nimble/nimble_port.h"
#include "nvs_flash.h"
#include <stdexcept>

// espressif forces me to create missing protoypes for them...
extern "C" void ble_store_config_init(void);
void on_stack_reset(int reason);
void on_stack_sync();
void ble_gatt_server_register_callback(ble_gatt_register_ctxt *ctxt, void *arg);

namespace ble {

BLE::BLE(std::string const &app_name) : gap(app_name) {
  prepare_nvm();
  init_nimble_host();
  nimble_host_config_init();
  ble::gatt::service_init();
}

BLE::~BLE() {}

void BLE::prepare_nvm() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    ret = nvs_flash_init();
  }
  if (ret != ESP_OK) {
    throw std::runtime_error(std::string("failed to initialize nvs flash"));
  }
}

void BLE::init_nimble_host() {
  if (nimble_port_init()) {
    throw std::runtime_error(std::string("failed to initialize nimble stack"));
  }
}

void BLE::nimble_host_config_init() {
  ble_hs_cfg.reset_cb = on_stack_reset;
  ble_hs_cfg.sync_cb = on_stack_sync;
  ble_hs_cfg.gatts_register_cb = ble_gatt_server_register_callback;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

  ble_store_config_init();
}

void BLE::nimble_host_task() { ble_nimble_port_run(); }

template <typename uuid_type, uint8_t N>
void BLE::send_indication(gatt::Service<uuid_type, N> &service) {
  if (service.is_indicated &&
      service.characteristic.is_connection_handle_initialized) {
    ble_gatts_indicate(service.characteristic.connection_handle_id,
                       service.characteristic.value_handle);
  }
}
} // namespace ble

void on_stack_reset(int reason) {
  ESP_LOGI("GATT-Server", "nimble stack reset, reset reason: %d", reason);
}
void on_stack_sync() { ble::gap::advertizing_init(); }
void ble_gatt_server_register_callback(ble_gatt_register_ctxt *ctxt,
                                       void *arg) {
  ble::gatt::service_register_cb(ctxt, arg);
}
