#include "gap.hpp"
#include "gatt.hpp"
#include <stdexcept>
#include <string>

extern "C" {
#include "esp_log.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
}

namespace ble {

GAP::GAP(std::string const &app_name) {
  ble_svc_gap_init();

  if (ble_svc_gap_device_name_set(app_name.c_str())) {
    throw std::runtime_error("failed to set device name to NimBLE-GATT");
  }
}
GAP::~GAP() {}

inline void format_addr(char *address_string, uint8_t addr[]);
void print_conn_desc(ble_gap_conn_desc *desc);
void start_advertising();
int gap_event_handler(struct ble_gap_event *event, void *arg);

constexpr std::string esp_uri = "\x17//espressif.com";

/* Private functions */
inline void format_addr(char *address_string, uint8_t addr[]) {
  sprintf(address_string, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1],
          addr[2], addr[3], addr[4], addr[5]);
}

void print_conn_desc(ble_gap_conn_desc *desc) {
  char address_string[18] = {0};

  /* Connection handle */
  ESP_LOGI("GATT-Server", "connection handle: %d", desc->conn_handle);

  format_addr(address_string, desc->our_id_addr.val);
  ESP_LOGI("GATT-Server", "device id address: type=%d, value=%s",
           desc->our_id_addr.type, address_string);

  /* Peer ID address */
  format_addr(address_string, desc->peer_id_addr.val);
  ESP_LOGI("GATT-Server", "peer id address: type=%d, value=%s",
           desc->peer_id_addr.type, address_string);

  /* Connection info */
  ESP_LOGI("GATT-Server",
           "conn_itvl=%d, conn_latency=%d, supervision_timeout=%d, "
           "encrypted=%d, authenticated=%d, bonded=%d\n",
           desc->conn_itvl, desc->conn_latency, desc->supervision_timeout,
           desc->sec_state.encrypted, desc->sec_state.authenticated,
           desc->sec_state.bonded);
}

void advertize() {
  const char *name = ble_svc_gap_device_name();

  ble_hs_adv_fields advertizing_fields = {0};
  /* Set advertising flags */
  advertizing_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

  /* Set device name */
  advertizing_fields.name = (uint8_t *)name;
  advertizing_fields.name_len = strlen(name);
  advertizing_fields.name_is_complete = 1;

  /* Set device tx power */
  advertizing_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
  advertizing_fields.tx_pwr_lvl_is_present = 1;

  /* Set device appearance */
  advertizing_fields.appearance = 0x200;
  advertizing_fields.appearance_is_present = 1;

  /* Set device LE role */
  advertizing_fields.le_role = 0;
  advertizing_fields.le_role_is_present = 1;

  /* Set advertiement fields */
  if (ble_gap_adv_set_fields(&advertizing_fields)) {
    throw std::runtime_error("failed to set advertising data");
  }

  ble_hs_adv_fields response_fields = {0};
  /* Set device address */
  response_fields.device_addr = address_value;
  response_fields.device_addr_type = own_addr_type;
  response_fields.device_addr_is_present = 1;

  /* Set URI */
  response_fields.uri = reinterpret_cast<uint8_t const *>(esp_uri.c_str());
  response_fields.uri_len = esp_uri.size();

  /* Set advertising interval */
  response_fields.adv_itvl = BLE_GAP_ADV_ITVL_MS(500);
  response_fields.adv_itvl_is_present = 1;

  /* Set scan response fields */
  if (ble_gap_adv_rsp_set_fields(&response_fields)) {
    throw std::runtime_error("failed to set scan response data");
  }

  ble_gap_adv_params advertizing_parameters = {0};
  /* Set non-connetable and general discoverable mode to be a beacon */
  advertizing_parameters.conn_mode = BLE_GAP_CONN_MODE_UND;
  advertizing_parameters.disc_mode = BLE_GAP_DISC_MODE_GEN;

  /* Set advertising interval */
  advertizing_parameters.itvl_min = BLE_GAP_ADV_ITVL_MS(500);
  advertizing_parameters.itvl_max = BLE_GAP_ADV_ITVL_MS(510);

  /* Start advertising */
  if (ble_gap_adv_start(own_addr_type, nullptr, BLE_HS_FOREVER,
                        &advertizing_parameters, gap_event_handler, nullptr)) {
    throw std::runtime_error("failed to start advertising");
  }
  ESP_LOGI("GATT-Server", "advertising started!");
}

/**
 * NimBLE applies an event-driven model to keep GAP service going
 * gap_event_handler is a callback function registered when calling
 * ble_gap_adv_start API and called when a GAP event arrives */
int gap_event_handler(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT:
    /* Connection succeeded */
    if (event->connect.status == 0) {
      /* A new connection was established or a connection attempt failed. */
      ESP_LOGI("GATT-Server", "connection established");

      /* Check connection handle */
      ble_gap_conn_desc desc;
      if (ble_gap_conn_find(event->connect.conn_handle, &desc)) {
        throw std::runtime_error("failed to find connection by handle");
      }

      print_conn_desc(&desc);

      struct ble_gap_upd_params params = {.itvl_min = desc.conn_itvl,
                                          .itvl_max = desc.conn_itvl,
                                          .latency = 3,
                                          .supervision_timeout =
                                              desc.supervision_timeout};
      if (ble_gap_update_params(event->connect.conn_handle, &params)) {
        throw std::runtime_error("failed to update connection parameters");
      }
    } else {
      start_advertising();
      ESP_LOGI("GATT-Server", "connection failed; status=%d",
               event->connect.status);
    }
    return 0;

  case BLE_GAP_EVENT_DISCONNECT:
    start_advertising();
    ESP_LOGI("GATT-Server", "disconnected from peer; reason=%d",
             event->disconnect.reason);
    return 0;

  case BLE_GAP_EVENT_CONN_UPDATE:
    /* The central has updated the connection parameters. */
    ESP_LOGI("GATT-Server", "connection updated; status=%d",
             event->conn_update.status);

    ble_gap_conn_desc desc;
    if (ble_gap_conn_find(event->conn_update.conn_handle, &desc)) {
      throw std::runtime_error("failed to find connection by handle");
    }
    print_conn_desc(&desc);
    return 0;

  case BLE_GAP_EVENT_ADV_COMPLETE:
    start_advertising();
    ESP_LOGI("GATT-Server", "advertise complete; reason=%d",
             event->adv_complete.reason);
    return 0;

  case BLE_GAP_EVENT_NOTIFY_TX:
    if ((event->notify_tx.status != 0) &&
        (event->notify_tx.status != BLE_HS_EDONE)) {
      // gap event error
      ESP_LOGI("GATT-Server",
               "notify event; conn_handle=%d attr_handle=%d "
               "status=%d is_indication=%d",
               event->notify_tx.conn_handle, event->notify_tx.attr_handle,
               event->notify_tx.status, event->notify_tx.indication);
    }
    return 0;

  case BLE_GAP_EVENT_SUBSCRIBE:
    ESP_LOGI("GATT-Server",
             "subscribe event; conn_handle=%d attr_handle=%d "
             "reason=%d prevn=%d curn=%d previ=%d curi=%d",
             event->subscribe.conn_handle, event->subscribe.attr_handle,
             event->subscribe.reason, event->subscribe.prev_notify,
             event->subscribe.cur_notify, event->subscribe.prev_indicate,
             event->subscribe.cur_indicate);

    ble::gatt::service_subscribe_cb(event);
    return 0;

  case BLE_GAP_EVENT_MTU:
    ESP_LOGI("GATT-Server", "mtu update event; conn_handle=%d cid=%d mtu=%d",
             event->mtu.conn_handle, event->mtu.channel_id, event->mtu.value);
    return 0;
  }

  return 0;
}

void GAP::reinit_advertising() {
  /* Make sure we have proper BT identity address set (random preferred) */
  if (ble_hs_util_ensure_addr(0)) {
    throw std::runtime_error("device does not have any available bt address!");
  }

  /* Figure out BT address to use while advertising (no privacy for now) */
  if (ble_hs_id_infer_auto(0, &own_addr_type)) {
    throw std::runtime_error("failed to infer address type");
  }

  /* Printing ADDR */
  if (ble_hs_id_copy_addr(own_addr_type, address_value, nullptr)) {
    throw std::runtime_error("failed to copy device address");
  }

  char address_string[18] = {0};
  format_addr(address_string, address_value);
  ESP_LOGI("GATT-Server", "device address: %s", address_string);

  advertize();
}

} // namespace ble
