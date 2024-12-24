#pragma once

void send_heart_rate_indication();
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
void gatt_svr_subscribe_cb(struct ble_gap_event *event);
void gatt_svc_init();
