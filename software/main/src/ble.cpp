#include "ble.hpp"
#include "gap.hpp"
#include "gatt.hpp"
#include <stdexcept>

namespace ble {

BLE::BLE(std::string const &app_name) : gap(app_name) { gatt.service_init(); }

void BLE::nimble_host_task() { ble_nimble_port_run(); }

template <typename uuid_type, uint8_t N>
void BLE::send_indication(GATT::Service<uuid_type, N> &service) {
  if (service.is_indicated &&
      service.characteristic.is_connection_handle_initialized) {
    ble_gatts_indicate(service.characteristic.connection_handle_id,
                       service.characteristic.value_handle);
  }
}
} // namespace ble
