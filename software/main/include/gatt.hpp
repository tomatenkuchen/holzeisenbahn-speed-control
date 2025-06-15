/// @file gatt.hpp
/// @brief contains ble gatt service logic

#pragma once

#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"

namespace gatt {

class Gatt {
public:
  Gatt() {}

  /// Handle GATT attributes register events
  /// @param ctxt state structure for callback meta data
  /// @param arg additional arguments if given
  void service_register_callback(ble_gatt_register_ctxt *ctxt, void *arg);

  /// gets called when a client subscribes to a characteristic
  /// @param event carries event meta data to process
  void server_subscribe_callback(ble_gap_event *event);

  /// update indication flag for heart rate characteristic
  void send_heart_rate_indication();

private:
  void service_register_event(ble_gatt_register_ctxt *ctxt);
  void characteristic_register_event(ble_gatt_register_ctxt *ctxt);
  void descriptor_register_event(ble_gatt_register_ctxt *ctxt);
};

} // namespace gatt
