#pragma once

extern "C" {
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
}

namespace ble::gatt {
void send_heart_rate_indication();
void server_register_cb(ble_gatt_register_ctxt *ctxt, void *arg);
void server_subscribe_cb(ble_gap_event *event);
void service_init();
} // namespace ble::gatt
