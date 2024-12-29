/**
 * @file gatt.hpp
 * @brief ble gatt layer
 * @author tomatenkuchen
 * @date 2024-12-29
 * @license GPLv2 @see license.md
 */

#pragma once

#include <array>

extern "C" {
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
}

namespace ble {
class GATT {
public:
  struct UUID16 {
    uint16_t uuid;
  };

  struct UUID128 {
    std::array<uint8_t, 16>;
  };

  template <typename uuid_type, typename value_type, uint8_t N>
  struct Characteristic {
    uuid_type uuid;
    std::array<value_type, N + 1> value = {0};
    uint16_t value_handle;
    uint16_t connection_handle_id;
    bool is_connection_handle_initialized;
  };

  template <typename uuid_type, uint8_t N, typename character_uuid_type,
            typename character_value_type, uint8_t character_N>
  struct Service {
    uuid_type uuid;
    bool is_indicated;
    std::array<
        Characteristic<character_uuid_type, character_value_type, character_N>,
        N>
        characteristic;
  };

  GATT();
  void send_indication();
  void service_register_cb(ble_gatt_register_ctxt *ctxt, void *arg);
  void service_subscribe_cb(ble_gap_event *event);

private:
  std::array<...>;
};
} // namespace ble
