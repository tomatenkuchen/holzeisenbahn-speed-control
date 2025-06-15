#include "ble.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "led.hpp"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "services/gap/ble_svc_gap.h"
#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>

extern "C" void ble_store_config_init();

namespace ble {

namespace {

constexpr std::string TAG = "ble";

int heart_rate_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                          ble_gatt_access_ctxt *ctxt, void *arg);

int led_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                   ble_gatt_access_ctxt *ctxt, void *arg);

const ble_uuid16_t heart_rate_svc_uuid = BLE_UUID16_INIT(0x180D);

uint8_t heart_rate_chr_val[2] = {0};
uint16_t heart_rate_chr_val_handle;
const ble_uuid16_t heart_rate_chr_uuid = BLE_UUID16_INIT(0x2A37);

uint16_t heart_rate_chr_conn_handle = 0;
bool heart_rate_chr_conn_handle_inited = false;
bool heart_rate_ind_status = false;

/* Automation IO service */
const ble_uuid16_t auto_io_svc_uuid = BLE_UUID16_INIT(0x1815);

uint16_t led_chr_val_handle;

const ble_uuid128_t led_chr_uuid =
    BLE_UUID128_INIT(0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15, 0xde, 0xef,
                     0x12, 0x12, 0x25, 0x15, 0x00, 0x00);

ble_gatt_chr_def const heart_rate_characteristic = {
    .uuid = &heart_rate_chr_uuid.u,
    .access_cb = heart_rate_chr_access,
    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE,
    .val_handle = &heart_rate_chr_val_handle,
};

std::array<ble_gatt_chr_def, 2> heart_rate_characteristics = {
    heart_rate_characteristic,
    {0},
};

ble_gatt_svc_def heart_rate_service = {
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = &heart_rate_svc_uuid.u,
    .characteristics = heart_rate_characteristics.data(),
};

ble_gatt_chr_def const led_characteristic = {
    .uuid = &led_chr_uuid.u,
    .access_cb = led_chr_access,
    .flags = BLE_GATT_CHR_F_WRITE,
    .val_handle = &led_chr_val_handle,
};

std::array<ble_gatt_chr_def, 2> led_characteristics = {
    led_characteristic,
    {0},
};

ble_gatt_svc_def const led_service = {
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = &auto_io_svc_uuid.u,
    .characteristics = led_characteristics.data(),
};

std::array<ble_gatt_svc_def, 3> const ble_services = {
    heart_rate_service,
    led_service,
    {0},
};

int heart_rate_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                          ble_gatt_access_ctxt *ctxt, void *arg) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
    throw std::runtime_error("gatt: bad heart rate access");
  }

  // Verify connection handle
  if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
    ESP_LOGI(TAG.c_str(), "characteristic read; conn_handle=%d attr_handle=%d",
             conn_handle, attr_handle);
  } else {
    ESP_LOGI(TAG.c_str(), "characteristic read by nimble stack; attr_handle=%d",
             attr_handle);
  }

  // Verify attribute handle
  if (attr_handle == heart_rate_chr_val_handle) {
    // Update access buffer value
    // heart_rate_chr_val[1] = get_heart_rate();
    int rc = os_mbuf_append(ctxt->om, &heart_rate_chr_val,
                            sizeof(heart_rate_chr_val));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
  }

  return BLE_ATT_ERR_UNLIKELY;
}

int led_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                   ble_gatt_access_ctxt *ctxt, void *arg) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
    return BLE_ERR_UNSPECIFIED;
  }

  if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
    ESP_LOGI(TAG.c_str(), "characteristic write; conn_handle=%d attr_handle=%d",
             conn_handle, attr_handle);
  } else {
    ESP_LOGI(TAG.c_str(),
             "characteristic write by nimble stack; attr_handle=%d",
             attr_handle);
  }

  // Verify attribute handle
  if (attr_handle != led_chr_val_handle) {
    throw std::runtime_error(
        "unexpected access operation on led characteristic");
  }

  // Verify access buffer length
  if (ctxt->om->om_len != 1) {
    throw std::runtime_error(
        "unexpected access operation on led characteristic");
  }

  /* Turn the LED on or off according to the operation bit */
  if (ctxt->om->om_data[0]) {
    led::on();
    ESP_LOGI(TAG.c_str(), "led turned on!");
  } else {
    led::off();
    ESP_LOGI(TAG.c_str(), "led turned off!");
  }

  return 0;
}

void log_event_type(ble_gap_event const *const event) {
  if (event->subscribe.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
    ESP_LOGI(TAG.c_str(), "subscribe event; conn_handle=%d attr_handle=%d",
             event->subscribe.conn_handle, event->subscribe.attr_handle);
  } else {
    ESP_LOGI(TAG.c_str(), "subscribe by nimble stack; attr_handle=%d",
             event->subscribe.attr_handle);
  }
}

void update_heartrate_subscription_state(ble_gap_event const *const event) {
  heart_rate_chr_conn_handle = event->subscribe.conn_handle;
  heart_rate_chr_conn_handle_inited = true;
  heart_rate_ind_status = event->subscribe.cur_indicate;
}

void Ble::service_register_event(ble_gatt_register_ctxt *ctxt) {
  char buf[BLE_UUID_STR_LEN];
  ESP_LOGD(TAG.c_str(), "registered service %s with handle=%d",
           ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf), ctxt->svc.handle);
}

void Ble::characteristic_register_event(ble_gatt_register_ctxt *ctxt) {
  char buf[BLE_UUID_STR_LEN];
  ESP_LOGD(TAG.c_str(),
           "registering characteristic %s with "
           "def_handle=%d val_handle=%d",
           ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf), ctxt->chr.def_handle,
           ctxt->chr.val_handle);
}

void Ble::descriptor_register_event(ble_gatt_register_ctxt *ctxt) {
  char buf[BLE_UUID_STR_LEN];
  ESP_LOGD(TAG.c_str(), "registering descriptor %s with handle=%d",
           ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf), ctxt->dsc.handle);
}

void Ble::service_register_callback(ble_gatt_register_ctxt *ctxt, void *arg) {
  switch (ctxt->op) {

  case BLE_GATT_REGISTER_OP_SVC:
    service_register_event(ctxt);
    return;

  case BLE_GATT_REGISTER_OP_CHR:
    characteristic_register_event(ctxt);
    return;

  case BLE_GATT_REGISTER_OP_DSC:
    descriptor_register_event(ctxt);
    return;

  default:
    throw std::runtime_error("gatt service register error");
  }
}

void Ble::send_heart_rate_indication() {
  if (heart_rate_ind_status && heart_rate_chr_conn_handle_inited) {
    ble_gatts_indicate(heart_rate_chr_conn_handle, heart_rate_chr_val_handle);
    ESP_LOGI(TAG.c_str(), "heart rate indication sent!");
  }
}

void Ble::server_subscribe_callback(ble_gap_event *event) {
  log_event_type(event);

  // Check attribute handle
  if (event->subscribe.attr_handle == heart_rate_chr_val_handle) {
    update_heartrate_subscription_state(event);
  }
}

constexpr static std::string_view esp_uri = "\x17//espressif.com";
uint8_t own_addr_type;
uint8_t addr_val[6] = {0};
std::string device_name;

ble_hs_adv_fields adv_fields = {
    .flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP,

    // Set device tx power
    .tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO,
    .tx_pwr_lvl_is_present = 1,

    // Set device appearance
    .appearance = 0x0200,
    .appearance_is_present = 1,

    // Set device LE role
    .le_role = 0,
    .le_role_is_present = 1,
};

ble_hs_adv_fields rsp_fields = {
    .adv_itvl = BLE_GAP_ADV_ITVL_MS(500),
    .adv_itvl_is_present = 1,

    .device_addr = addr_val,
    .device_addr_type = own_addr_type,
    .device_addr_is_present = 1,

    .uri = reinterpret_cast<uint8_t const *>(esp_uri.data()),
    .uri_len = static_cast<uint8_t>(esp_uri.size()),
};

ble_gap_adv_params adv_params = {
    .conn_mode = BLE_GAP_CONN_MODE_UND,
    .disc_mode = BLE_GAP_DISC_MODE_GEN,

    // Set advertising interval
    .itvl_min = BLE_GAP_ADV_ITVL_MS(500),
    .itvl_max = BLE_GAP_ADV_ITVL_MS(510),
};

void format_addr(char *addr_str, uint8_t addr[]) {
  sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1], addr[2],
          addr[3], addr[4], addr[5]);
}

void print_conn_desc(ble_gap_conn_desc *desc) {
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

int connect_event(ble_gap_event *event) {

  // A new connection was established or a connection attempt failed
  ESP_LOGI(TAG.c_str(), "connection %s; status=%d",
           event->connect.status == 0 ? "established" : "failed",
           event->connect.status);

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

void disconnect_event(ble_gap_event *event) {
  // A connection was terminated, print connection descriptor
  ESP_LOGI(TAG.c_str(), "disconnected from peer; reason=%d",
           event->disconnect.reason);

  // Restart advertising
  start_advertising();
}

void update_event(ble_gap_event *event) {
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

void advertizing_complete_event(ble_gap_event *event) {
  // Advertising completed, restart advertising
  ESP_LOGI(TAG.c_str(), "advertise complete; reason=%d",
           event->adv_complete.reason);
  start_advertising();
}

void notify_event(ble_gap_event *event) {
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

} // namespace

void Ble::subscribe_event(ble_gap_event *event) {

  // Print subscription info to log
  ESP_LOGI(TAG.c_str(),
           "subscribe event; conn_handle=%d attr_handle=%d "
           "reason=%d prevn=%d curn=%d previ=%d curi=%d",
           event->subscribe.conn_handle, event->subscribe.attr_handle,
           event->subscribe.reason, event->subscribe.prev_notify,
           event->subscribe.cur_notify, event->subscribe.prev_indicate,
           event->subscribe.cur_indicate);

  // GATT subscribe event callback
  gatt.service_subscribe_callback(event);
}

void Ble::mtu_event(ble_gap_event *event) {
  // Print MTU update info to log
  ESP_LOGI(TAG.c_str(), "mtu update event; conn_handle=%d cid=%d mtu=%d",
           event->mtu.conn_handle, event->mtu.channel_id, event->mtu.value);
}

/// callback routine for gap event servicing
/// @param event type of event that occured
/// @param args additional info besides event data. not used by any callback but
/// required by callback type
int Ble::event_handler(ble_gap_event *event, void *args) {

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

void Ble::start_advertising() {
  if (ble_gap_adv_set_fields(&adv_fields) != 0) {
    throw std::runtime_error("failed to set advertising data");
  }

  if (ble_gap_adv_rsp_set_fields(&rsp_fields) != 0) {
    throw std::runtime_error("failed to set scan response data");
  }

  // Start advertising
  if (ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                        event_handler, NULL) != 0) {
    throw std::runtime_error("failed to start advertising");
  }

  ESP_LOGI(TAG.c_str(), "advertising started!");
}

void stop_advertizing() { ble_gap_adv_stop(); }

Ble::init_gatt() {
  ble_svc_gatt_init();

  if (ble_gatts_count_cfg(ble_services.data()) != 0) {
    throw std::runtime_error("gatt service counter update failed");
  }

  if (ble_gatts_add_svcs(ble_services.data()) != 0) {
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
}

Ble::Ble(std::string _device_name, Antenna antenna) {
  choose_antenna(antenna);
  init_nvs();
  init_nimble_port();
  nimble_host_config_init();
  gap_init(_device_name);
  init_gatt();
}

void Ble::nimble_host_task() {
  nimble_port_run();
  vTaskDelete(NULL);
}

void Ble::advertize() {
  ESP_LOGI(TAG.c_str(), "start advertizing");
  start_advertising();
}

void Ble::init_nvs() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
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
} // namespace ble
