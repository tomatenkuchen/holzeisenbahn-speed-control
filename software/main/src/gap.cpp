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

namespace {
constexpr std::string TAG = "gap";
inline void format_addr(char *addr_str, uint8_t addr[]);
void print_conn_desc(struct ble_gap_conn_desc *desc);
void start_advertising(void);
int gap_event_handler(struct ble_gap_event *event, void *arg);
uint8_t own_addr_type;
uint8_t addr_val[6] = {0};
std::string_view esp_uri = "\x17//espressif.com";

inline void format_addr(char *addr_str, uint8_t addr[]) {
  sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1], addr[2],
          addr[3], addr[4], addr[5]);
}

void print_conn_desc(struct ble_gap_conn_desc *desc) {
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

void start_advertising() {
  int rc = 0;
  const char *name;
  struct ble_hs_adv_fields adv_fields = {0};
  struct ble_hs_adv_fields rsp_fields = {0};
  struct ble_gap_adv_params adv_params = {0};

  /* Set advertising flags */
  adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

  /* Set device name */
  name = ble_svc_gap_device_name();
  adv_fields.name = (uint8_t *)name;
  adv_fields.name_len = strlen(name);
  adv_fields.name_is_complete = 1;

  /* Set device tx power */
  adv_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
  adv_fields.tx_pwr_lvl_is_present = 1;

  /* Set device appearance */
  adv_fields.appearance = BLE_GAP_APPEARANCE_GENERIC_TAG;
  adv_fields.appearance_is_present = 1;

  /* Set device LE role */
  adv_fields.le_role = BLE_GAP_LE_ROLE_PERIPHERAL;
  adv_fields.le_role_is_present = 1;

  /* Set advertiement fields */
  rc = ble_gap_adv_set_fields(&adv_fields);
  if (rc != 0) {
    ESP_LOGE(TAG.c_str(), "failed to set advertising data, error code: %d", rc);
    return;
  }

  rsp_fields.device_addr = addr_val;
  rsp_fields.device_addr_type = own_addr_type;
  rsp_fields.device_addr_is_present = 1;

  rsp_fields.uri = reinterpret_cast<uint8_t const *>(esp_uri.data());
  rsp_fields.uri_len = esp_uri.size();

  rsp_fields.adv_itvl = BLE_GAP_ADV_ITVL_MS(500);
  rsp_fields.adv_itvl_is_present = 1;

  rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
  if (rc != 0) {
    ESP_LOGE(TAG.c_str(), "failed to set scan response data, error code: %d",
             rc);
    return;
  }

  /* Set non-connetable and general discoverable mode to be a beacon */
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

  /* Set advertising interval */
  adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(500);
  adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(510);

  /* Start advertising */
  rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                         gap_event_handler, NULL);
  if (rc != 0) {
    ESP_LOGE(TAG.c_str(), "failed to start advertising, error code: %d", rc);
    return;
  }
  ESP_LOGI(TAG.c_str(), "advertising started!");
}

/*
 * NimBLE applies an event-driven model to keep GAP service going
 * gap_event_handler is a callback function registered when calling
 * ble_gap_adv_start API and called when a GAP event arrives
 */
int gap_event_handler(struct ble_gap_event *event, void *arg) {
  /* Local variables */
  int rc = 0;
  struct ble_gap_conn_desc desc;

  /* Handle different GAP event */
  switch (event->type) {

  /* Connect event */
  case BLE_GAP_EVENT_CONNECT:
    /* A new connection was established or a connection attempt failed. */
    ESP_LOGI(TAG.c_str(), "connection %s; status=%d",
             event->connect.status == 0 ? "established" : "failed",
             event->connect.status);

    /* Connection succeeded */
    if (event->connect.status == 0) {
      /* Check connection handle */
      rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
      if (rc != 0) {
        ESP_LOGE(TAG.c_str(),
                 "failed to find connection by handle, error code: %d", rc);
        return rc;
      }

      /* Print connection descriptor */
      print_conn_desc(&desc);

      /* Try to update connection parameters */
      struct ble_gap_upd_params params = {.itvl_min = desc.conn_itvl,
                                          .itvl_max = desc.conn_itvl,
                                          .latency = 3,
                                          .supervision_timeout =
                                              desc.supervision_timeout};
      rc = ble_gap_update_params(event->connect.conn_handle, &params);
      if (rc != 0) {
        ESP_LOGE(TAG.c_str(),
                 "failed to update connection parameters, error code: %d", rc);
        return rc;
      }
    }
    /* Connection failed, restart advertising */
    else {
      start_advertising();
    }
    return rc;

  /* Disconnect event */
  case BLE_GAP_EVENT_DISCONNECT:
    /* A connection was terminated, print connection descriptor */
    ESP_LOGI(TAG.c_str(), "disconnected from peer; reason=%d",
             event->disconnect.reason);

    /* Restart advertising */
    start_advertising();
    return rc;

  /* Connection parameters update event */
  case BLE_GAP_EVENT_CONN_UPDATE:
    /* The central has updated the connection parameters. */
    ESP_LOGI(TAG.c_str(), "connection updated; status=%d",
             event->conn_update.status);

    /* Print connection descriptor */
    rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
    if (rc != 0) {
      ESP_LOGE(TAG.c_str(),
               "failed to find connection by handle, error code: %d", rc);
      return rc;
    }
    print_conn_desc(&desc);
    return rc;

  /* Advertising complete event */
  case BLE_GAP_EVENT_ADV_COMPLETE:
    /* Advertising completed, restart advertising */
    ESP_LOGI(TAG.c_str(), "advertise complete; reason=%d",
             event->adv_complete.reason);
    start_advertising();
    return rc;

  /* Notification sent event */
  case BLE_GAP_EVENT_NOTIFY_TX:
    if ((event->notify_tx.status != 0) &&
        (event->notify_tx.status != BLE_HS_EDONE)) {
      /* Print notification info on error */
      ESP_LOGI(TAG.c_str(),
               "notify event; conn_handle=%d attr_handle=%d "
               "status=%d is_indication=%d",
               event->notify_tx.conn_handle, event->notify_tx.attr_handle,
               event->notify_tx.status, event->notify_tx.indication);
    }
    return rc;

  /* Subscribe event */
  case BLE_GAP_EVENT_SUBSCRIBE:
    /* Print subscription info to log */
    ESP_LOGI(TAG.c_str(),
             "subscribe event; conn_handle=%d attr_handle=%d "
             "reason=%d prevn=%d curn=%d previ=%d curi=%d",
             event->subscribe.conn_handle, event->subscribe.attr_handle,
             event->subscribe.reason, event->subscribe.prev_notify,
             event->subscribe.cur_notify, event->subscribe.prev_indicate,
             event->subscribe.cur_indicate);

    /* GATT subscribe event callback */
    gatt_svr_subscribe_cb(event);
    return rc;

  /* MTU update event */
  case BLE_GAP_EVENT_MTU:
    /* Print MTU update info to log */
    ESP_LOGI(TAG.c_str(), "mtu update event; conn_handle=%d cid=%d mtu=%d",
             event->mtu.conn_handle, event->mtu.channel_id, event->mtu.value);
    return rc;
  }

  return rc;
}

} // namespace

void adv_init() {
  /* Make sure we have proper BT identity address set (random preferred) */
  int rc = ble_hs_util_ensure_addr(0);
  if (rc != 0) {
    ESP_LOGE(TAG.c_str(), "device does not have any available bt address!");
    return;
  }

  /* Figure out BT address to use while advertising (no privacy for now) */
  rc = ble_hs_id_infer_auto(0, &own_addr_type);
  if (rc != 0) {
    ESP_LOGE(TAG.c_str(), "failed to infer address type, error code: %d", rc);
    return;
  }

  // Printing ADDR */
  char addr_str[18] = {0};
  rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
  if (rc != 0) {
    ESP_LOGE(TAG.c_str(), "failed to copy device address, error code: %d", rc);
    return;
  }
  format_addr(addr_str, addr_val);
  ESP_LOGI(TAG.c_str(), "device address: %s", addr_str);

  // Start advertising. */
  start_advertising();
}

void gap_init(std::string device_name) {

  ble_svc_gap_init();

  if (ble_svc_gap_device_name_set(device_name.c_str()) != 0) {
    throw std::runtime_error("failed to set device name");
  }
}
