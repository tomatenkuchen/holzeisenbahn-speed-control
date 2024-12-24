#include "gatt_svc.hpp"
#include "heart_rate.hpp"
#include "led.hpp"
#include <cstdint>
#include <stdexcept>

extern "C" {
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"
}

namespace {

int heart_rate_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg);

int led_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                   struct ble_gatt_access_ctxt *ctxt, void *arg);

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

const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    /* Heart rate service */
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = &heart_rate_svc_uuid.u,
     .characteristics =
         (struct ble_gatt_chr_def[]){
             {/* Heart rate characteristic */
              .uuid = &heart_rate_chr_uuid.u,
              .access_cb = heart_rate_chr_access,
              .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_INDICATE,
              .val_handle = &heart_rate_chr_val_handle},
             {
                 0, /* No more characteristics in this service. */
             }}},

    /* Automation IO service */
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &auto_io_svc_uuid.u,
        .characteristics =
            (struct ble_gatt_chr_def[]){/* LED characteristic */
                                        {.uuid = &led_chr_uuid.u,
                                         .access_cb = led_chr_access,
                                         .flags = BLE_GATT_CHR_F_WRITE,
                                         .val_handle = &led_chr_val_handle},
                                        {0}},
    },

    {
        0, /* No more services. */
    },
};

/* Private functions */
int heart_rate_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg) {

  switch (ctxt->op) {

  /* Read characteristic event */
  case BLE_GATT_ACCESS_OP_READ_CHR:
    /* Verify connection handle */
    if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
      ESP_LOGI("GATT-Server",
               "characteristic read; conn_handle=%d attr_handle=%d",
               conn_handle, attr_handle);
    } else {
      ESP_LOGI("GATT-Server",
               "characteristic read by nimble stack; attr_handle=%d",
               attr_handle);
    }

    /* Verify attribute handle */
    if (attr_handle == heart_rate_chr_val_handle) {
      /* Update access buffer value */
      heart_rate_chr_val[1] = get_heart_rate();
      int rc = os_mbuf_append(ctxt->om, &heart_rate_chr_val,
                              sizeof(heart_rate_chr_val));
      return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    goto error;

  /* Unknown event */
  default:
    goto error;
  }

error:
  ESP_LOGE(
      "GATT-Server",
      "unexpected access operation to heart rate characteristic, opcode: %d",
      ctxt->op);
  return BLE_ATT_ERR_UNLIKELY;
}

int led_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                   struct ble_gatt_access_ctxt *ctxt, void *arg) {
  /* Local variables */
  int rc;

  /* Handle access events */
  /* Note: LED characteristic is write only */
  switch (ctxt->op) {

  /* Write characteristic event */
  case BLE_GATT_ACCESS_OP_WRITE_CHR:
    /* Verify connection handle */
    if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
      ESP_LOGI("GATT-Server",
               "characteristic write; conn_handle=%d attr_handle=%d",
               conn_handle, attr_handle);
    } else {
      ESP_LOGI("GATT-Server",
               "characteristic write by nimble stack; attr_handle=%d",
               attr_handle);
    }

    /* Verify attribute handle */
    if (attr_handle == led_chr_val_handle) {
      /* Verify access buffer length */
      if (ctxt->om->om_len == 1) {
        /* Turn the LED on or off according to the operation bit */
        if (ctxt->om->om_data[0]) {
          led_on();
          ESP_LOGI("GATT-Server", "led turned on!");
        } else {
          led_off();
          ESP_LOGI("GATT-Server", "led turned off!");
        }
      } else {
        goto error;
      }
      return rc;
    }
    goto error;

  /* Unknown event */
  default:
    goto error;
  }

error:
  ESP_LOGE("GATT-Server",
           "unexpected access operation to led characteristic, opcode: %d",
           ctxt->op);
  return BLE_ATT_ERR_UNLIKELY;
}

} // namespace

void send_heart_rate_indication() {
  if (heart_rate_ind_status && heart_rate_chr_conn_handle_inited) {
    ble_gatts_indicate(heart_rate_chr_conn_handle, heart_rate_chr_val_handle);
    ESP_LOGI("GATT-Server", "heart rate indication sent!");
  }
}

/**
 *  Handle GATT attribute register events
 *      - Service register event
 *      - Characteristic register event
 *      - Descriptor register event */
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
  char buf[BLE_UUID_STR_LEN];

  switch (ctxt->op) {
  case BLE_GATT_REGISTER_OP_SVC:
    /* Service register event */
    ESP_LOGD("GATT-Server", "registered service %s with handle=%d",
             ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf), ctxt->svc.handle);
    break;

  case BLE_GATT_REGISTER_OP_CHR:
    /* Characteristic register event */
    ESP_LOGD("GATT-Server",
             "registering characteristic %s with def_handle=%d val_handle=%d",
             ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
             ctxt->chr.def_handle, ctxt->chr.val_handle);
    break;

  case BLE_GATT_REGISTER_OP_DSC:
    /* Descriptor register event */
    ESP_LOGD("GATT-Server", "registering descriptor %s with handle=%d",
             ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf), ctxt->dsc.handle);
    break;

  default:
    std::runtime_error("unknown attribute registered");
    break;
  }
}

/*
 *  GATT server subscribe event callback
 *      1. Update heart rate subscription status
 */
void gatt_svr_subscribe_cb(struct ble_gap_event *event) {
  /* Check connection handle */
  if (event->subscribe.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
    ESP_LOGI("GATT-Server", "subscribe event; conn_handle=%d attr_handle=%d",
             event->subscribe.conn_handle, event->subscribe.attr_handle);
  } else {
    ESP_LOGI("GATT-Server", "subscribe by nimble stack; attr_handle=%d",
             event->subscribe.attr_handle);
  }

  /* Check attribute handle */
  if (event->subscribe.attr_handle == heart_rate_chr_val_handle) {
    /* Update heart rate subscription status */
    heart_rate_chr_conn_handle = event->subscribe.conn_handle;
    heart_rate_chr_conn_handle_inited = true;
    heart_rate_ind_status = event->subscribe.cur_indicate;
  }
}

/*
 *  GATT server initialization
 *      1. Initialize GATT service
 *      2. Update NimBLE host GATT services counter
 *      3. Add GATT services to server */
void gatt_svc_init() {

  ble_svc_gatt_init();

  if (ble_gatts_count_cfg(gatt_svr_svcs)) {
    std::runtime_error("gatt counter error");
  }

  if (ble_gatts_add_svcs(gatt_svr_svcs)) {
    std::runtime_error("gatt add service failed");
  }
}
