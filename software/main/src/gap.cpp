#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_log_level.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gap.hpp"
#include "gatt_svc.hpp"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <stdexcept>
#include <string>
#include <string_view>

namespace gap {

Gap::Gap(std::string _device_name)
    : device_name{_device_name},

      adv_fields{
          .flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP,

          .name = device_name.c_str(),
          .name_len = device_name.size(),
          .name_is_complete = 1,

          // Set device tx power
          .tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO,
          .tx_pwr_lvl_is_present = 1,

          // Set device appearance
          .appearance = 0x0200,
          .appearance_is_present = 1,

          // Set device LE role
          .le_role = 0,
          .le_role_is_present = 1,
      },

      rsp_fields{
          .device_addr = addr_val,
          .device_addr_type = own_addr_type,
          .device_addr_is_present = 1,

          .uri = reinterpret_cast<uint8_t const *>(esp_uri.data()),
          .uri_len = esp_uri.size(),

          .adv_itvl = BLE_GAP_ADV_ITVL_MS(500),
          .adv_itvl_is_present = 1,
      },

      // Set non-connetable and general discoverable mode to be a beacon
      adv_params{

          .conn_mode = BLE_GAP_CONN_MODE_UND,
          .disc_mode = BLE_GAP_DISC_MODE_GEN,

          // Set advertising interval
          .itvl_min = BLE_GAP_ADV_ITVL_MS(500),
          .itvl_max = BLE_GAP_ADV_ITVL_MS(510),
      } {

  ble_svc_gap_init();

  // if (ble_svc_gap_device_name_set(device_name.c_str()) != 0) {
  // throw std::runtime_error("failed to set device name");
  // }
}

void Gap::advertizing_start(ble_gap_event_fn gap_event_callback) {
  // Make sure we have proper BT identity address set (random preferred)
  if (ble_hs_util_ensure_addr(0) != 0) {
    throw std::runtime_error("device does not have any available bt address!");
  }

  // Figure out BT address to use while advertising (no privacy for now)
  if (ble_hs_id_infer_auto(0, &own_addr_type) != 0) {
    throw std::runtime_error("failed to infer address type");
  }

  if (ble_hs_id_copy_addr(own_addr_type, addr_val, NULL) != 0) {
    throw std::runtime_error("failed to copy device address");
  }

  char addr_str[18] = {0};
  format_addr(addr_str, addr_val);
  ESP_LOGI(TAG.c_str(), "device address: %s", addr_str);

  start_advertising(gap_event_callback);
}

void Gap::start_advertising(ble_gap_event_fn gap_event_callback) {
  // auto const name = ble_svc_gap_device_name();

  if (ble_gap_adv_set_fields(&adv_fields) != 0) {
    throw std::runtime_error("failed to set advertising data");
  }

  if (ble_gap_adv_rsp_set_fields(&rsp_fields) != 0) {
    throw std::runtime_error("failed to set scan response data");
  }

  // Start advertising
  if (ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                        gap_event_handler, NULL) != 0) {
    throw std::runtime_error("failed to start advertising");
  }

  ESP_LOGI(TAG.c_str(), "advertising started!");
}

int Gap::connect_event(ble_gap_event *event) {

  // A new connection was established or a connection attempt failed
  ESP_LOGI(TAG.c_str(), "connection %s; status=%d",
           event->connect.status == 0 ? "established" : "failed",
           event->connect.status);

  // Connection succeeded
  if (event->connect.status != 0) {
    start_advertising();
    return 0;
  }

  // Check connection handle
  ble_gap_conn_desc desc;
  if (ble_gap_conn_find(event->connect.conn_handle, &desc) != 0) {
    throw std::runtime_error("failed to find connection by handle");
  }

  // Print connection descriptor
  print_conn_desc(&desc);

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

void Gap::disconnect_event(ble_gap_event *event) {
  // A connection was terminated, print connection descriptor
  ESP_LOGI(TAG.c_str(), "disconnected from peer; reason=%d",
           event->disconnect.reason);

  // Restart advertising
  start_advertising();
}

void Gap::update_event(ble_gap_event *event) {
  // The central has updated the connection parameters
  ESP_LOGI(TAG.c_str(), "connection updated; status=%d",
           event->conn_update.status);

  // Print connection descriptor
  ble_gap_conn_desc desc;
  if (ble_gap_conn_find(event->conn_update.conn_handle, &desc) != 0) {
    throw std::runtime_error("failed to find connection by handle");
  }

  print_conn_desc(&desc);
}

void Gap::advertizing_complete_event(ble_gap_event *event) {
  // Advertising completed, restart advertising
  ESP_LOGI(TAG.c_str(), "advertise complete; reason=%d",
           event->adv_complete.reason);
  start_advertising();
}

void Gap::notify_event(ble_gap_event *event) {
  if ((event->notify_tx.status != 0) &&
      (event->notify_tx.status != BLE_HS_EDONE)) {
    // Print notification info on error
    ESP_LOGI(TAG.c_str(),
             "notify event; conn_handle=%d attr_handle=%d "
             "status=%d is_indication=%d",
             event->notify_tx.conn_handle, event->notify_tx.attr_handle,
             event->notify_tx.status, event->notify_tx.indication);
  }
}

void Gap::subscribe_event(ble_gap_event *event) {

  // Print subscription info to log
  ESP_LOGI(TAG.c_str(),
           "subscribe event; conn_handle=%d attr_handle=%d "
           "reason=%d prevn=%d curn=%d previ=%d curi=%d",
           event->subscribe.conn_handle, event->subscribe.attr_handle,
           event->subscribe.reason, event->subscribe.prev_notify,
           event->subscribe.cur_notify, event->subscribe.prev_indicate,
           event->subscribe.cur_indicate);

  // GATT subscribe event callback
  gatt_svr_subscribe_cb(event);
}

void Gap::mtu_event(ble_gap_event *event) {
  // Print MTU update info to log
  ESP_LOGI(TAG.c_str(), "mtu update event; conn_handle=%d cid=%d mtu=%d",
           event->mtu.conn_handle, event->mtu.channel_id, event->mtu.value);
}

int Gap::event_handler(ble_gap_event *event) {

  // Handle different GAP event
  switch (event->type) {

  // Connect event
  case BLE_GAP_EVENT_CONNECT:
    return connect_event(event);
  case BLE_GAP_EVENT_DISCONNECT:
    disconnect_event(event);
    return 0;
  case BLE_GAP_EVENT_CONN_UPDATE:
    update_event(event);
    return 0;
  case BLE_GAP_EVENT_ADV_COMPLETE:
    advertizing_complete_event(event);
    return 0;
  case BLE_GAP_EVENT_NOTIFY_TX:
    notify_event(event);
    return 0;
  case BLE_GAP_EVENT_SUBSCRIBE:
    subscribe_event(event);
    return 0;
  case BLE_GAP_EVENT_MTU:
    mtu_event(event);
    return 0;
  }

  return 0;
}

void Gap::format_addr(char *addr_str, uint8_t addr[]) {
  sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1], addr[2],
          addr[3], addr[4], addr[5]);
}

void Gap::print_conn_desc(ble_gap_conn_desc *desc) {
  char addr_str[18] = {0};

  ESP_LOGI(TAG.c_str(), "connection handle: %d", desc->conn_handle);

  format_addr(addr_str, desc->our_id_addr.val);
  ESP_LOGI(TAG.c_str(), "device id address: type=%d, value=%s",
           desc->our_id_addr.type, addr_str);

  format_addr(addr_str, desc->peer_id_addr.val);
  ESP_LOGI(TAG.c_str(), "peer id address: type=%d, value=%s",
           desc->peer_id_addr.type, addr_str);

  ESP_LOGI(TAG.c_str(),
           "conn_itvl=%d, conn_latency=%d, supervision_timeout=%d, "
           "encrypted=%d, authenticated=%d, bonded=%d\n",
           desc->conn_itvl, desc->conn_latency, desc->supervision_timeout,
           desc->sec_state.encrypted, desc->sec_state.authenticated,
           desc->sec_state.bonded);
}

} // namespace gap
