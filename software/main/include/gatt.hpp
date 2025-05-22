#pragma once

#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"

void send_heart_rate_indication();

/// Handle GATT attribute register events
///     - Service register event
///     - Characteristic register event
///     - Descriptor register event
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);

/// GATT server subscribe event callback
///     1. Update heart rate subscription status
void gatt_svr_subscribe_cb(struct ble_gap_event *event);

/// GATT server initialization
///     1. Initialize GATT service
///     2. Update NimBLE host GATT services counter
///     3. Add GATT services to server
void gatt_svc_init();

namespace gatt {

class Gatt {
public:
  Gatt() {}

  ~Gatt() {}

  void service_register_callback(ble_gatt_register_ctxt *ctxt, void *arg);

private:
};

} // namespace gatt
