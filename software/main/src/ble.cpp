#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>

#include "ble.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_log_level.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_id.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/hci_common.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "services/gap/ble_svc_gap.h"

extern "C" void ble_store_config_init();

namespace ble {

constexpr std::string TAG = "ble";

Ble::Ble(std::string _device_name, ble_gap_event_fn *_external_event_handler,
         ble_gatt_svc_def *services, Antenna antenna)
    : external_event_handler{_external_event_handler} {
  choose_antenna(antenna);
  ESP_LOGI("ble", "constructor: antenna chosen");
  init_nvs();
  ESP_LOGI("ble", "constructor: nvs init ok");
  init_nimble_hci();
  ESP_LOGI("ble", "constructor: nimble hci init ok");
  init_nimble_port();
  ESP_LOGI("ble", "constructor: nimble port init ok");
  init_gap(_device_name);
  ESP_LOGI("ble", "constructor: gap init ok");
  init_gatt(services);
  ESP_LOGI("ble", "constructor: gatt init ok");
}

void Ble::format_addr(char *addr_str, uint8_t addr[]) {
  sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1], addr[2], addr[3], addr[4],
          addr[5]);
}

int Ble::connect_event(ble_gap_event *event) {
  // A new connection was established or a connection attempt failed
  ESP_LOGI(TAG.c_str(), "connection %s; status=%d",
           event->connect.status == 0 ? "established" : "failed", event->connect.status);

  if (event->connect.status != 0) {
    // Connection failed
    start_advertising();
    return 0;
  }

  // Check connection handle
  ble_gap_conn_desc desc;
  if (ble_gap_conn_find(event->connect.conn_handle, &desc) != 0) {
    throw std::runtime_error("failed to find connection by handle");
  }

  // Try to update connection parameters
  ble_gap_upd_params params = {
      .itvl_min = desc.conn_itvl,
      .itvl_max = desc.conn_itvl,
      .latency = 3,
      .supervision_timeout = desc.supervision_timeout,
  };

  if (ble_gap_update_params(event->connect.conn_handle, &params) != 0) {
    throw std::runtime_error("failed to update connection parameters");
  }

  return 0;
}

void Ble::disconnect_event(ble_gap_event *event) {
  ESP_LOGI("ble", "disconnected");
  start_advertising();
}

void Ble::advertizing_complete_event(ble_gap_event *event) {
  ESP_LOGI("ble", "advertizing complete. restart");
  start_advertising();
}

void Ble::event_handler(ble_gap_event *event) {
  // Handle different GAP event
  switch (event->type) {
    // Connect event
    case BLE_GAP_EVENT_CONNECT:
      connect_event(event);
      break;
    case BLE_GAP_EVENT_DISCONNECT:
      disconnect_event(event);
      break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
      advertizing_complete_event(event);
      break;
    default:
      ESP_LOGI("ble", "gap event type %d", event->type);
      break;
  }
}

void Ble::start_advertising() {
  ESP_LOGI("ble", "start advertizing");

  if (ble_hs_util_ensure_addr(0) != 0) {
    throw std::runtime_error("failed to ensure address");
  }

  if (ble_hs_id_infer_auto(0, &own_addr_type) != 0) {
    throw std::runtime_error("failed to infer auto address");
  }

  uint8_t address_value[18] = {0};
  if (ble_hs_id_copy_addr(own_addr_type, address_value, nullptr) != 0) {
    throw std::runtime_error("failed address copy");
  }

  // print address
  ESP_LOGI("ble", "start ad: address: %02x:%02x:%02x:%02x:%02x:%02x", address_value[0],
           address_value[1], address_value[2], address_value[3], address_value[4],
           address_value[5]);

  if (ble_gap_adv_set_fields(&adv_fields) != 0) {
    throw std::runtime_error("failed to set advertising data");
  }

  if (ble_gap_adv_rsp_set_fields(&rsp_fields) != 0) {
    throw std::runtime_error("failed to set scan response data");
  }

  // Start advertising
  if (ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, external_event_handler,
                        NULL) != 0) {
    throw std::runtime_error("failed to start advertising");
  }

  ESP_LOGI(TAG.c_str(), "advertising started!");
}

void Ble::stop_advertizing() { ble_gap_adv_stop(); }

void Ble::init_nimble_hci() {
  // if (esp_nimble_hci_and_controller_init() != ESP_OK) {
  //   throw std::runtime_error("failed to synchronise controller and nimble host");
  // }
}

void Ble::init_gatt(ble_gatt_svc_def *services) {
  ble_svc_gatt_init();

  if (ble_gatts_count_cfg(services) != 0) {
    throw std::runtime_error("gatt service counter update failed");
  }

  if (ble_gatts_add_svcs(services) != 0) {
    throw std::runtime_error("gatt service inclusion failed");
  }
}

void Ble::init_gap(std::string _device_name) {
  // fill in device name to advertizing struct
  device_name = _device_name;
  adv_fields.name = reinterpret_cast<uint8_t const *>(device_name.c_str());
  adv_fields.name_len = static_cast<uint8_t>(device_name.size());
  adv_fields.name_is_complete = 1;

  ble_svc_gap_init();

  ESP_LOGI("ble", "init gap: gap init ok");

  if (ble_svc_gap_device_name_set(_device_name.c_str()) != 0) {
    throw std::runtime_error("gap device name could not be set");
  }

  ESP_LOGI("ble", "init gap: gap name set ok");
}

void Ble::nimble_host_task() {
  nimble_port_run();
  vTaskDelete(NULL);
}

void Ble::init_nvs() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  if (ret != ESP_OK) {
    throw std::runtime_error("nvs init failed");
  }
}

void Ble::init_nimble_port() {
  if (nimble_port_init() != ESP_OK) {
    throw std::runtime_error("nimble port init failed");
  }
}

void Ble::choose_antenna(Antenna antenna) {
  // enable rf switch
  gpio_reset_pin(rf_switch_gpio);
  gpio_set_direction(rf_switch_gpio, GPIO_MODE_OUTPUT);
  gpio_set_level(rf_switch_gpio, false);

  // enable switch pin
  gpio_reset_pin(antenna_switch_gpio);
  gpio_set_direction(antenna_switch_gpio, GPIO_MODE_OUTPUT);

  gpio_set_level(antenna_switch_gpio, antenna == Antenna::external);
}

}  // namespace ble
