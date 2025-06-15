/// @file ble.hpp
/// @brief bluetooth low energy class
/// @copyright GPL v2.0

#include "hal/gpio_types.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "services/gatt/ble_svc_gatt.h"
#include <array>
#include <string>
#include <variant>

namespace ble {

/// uuid for predfined characteristic
struct UUID16 {
  uint16_t uuid;
};

/// uuid for custom characteristics
struct UUID128 {
  std::array<uint8_t, 16> uuid;
};

/// since we need arrays of characteristics and services, we want a variant to
/// accomordate different types of charachteristics, custom and predefined
using UUID = std::variant<UUID16, UUID128>;

/// flags defining the type of communication that is desired with the
/// characteristic
template <typename Value> struct Characteristic {
  enum class Flag {
    broadcast,
    read,
    write_no_response,
    write,
    notify,
    indicate,
    auth_sign_write,
    reliable_write,
    aux_write,
    read_encrypted,
    read_authenticated,
    read_authorized,
    write_encrypted,
    write_authenticated,
    write_authorized,
  };
  /// characteristic uuid
  UUID uuid;
  /// actual value
  Value value;
  /// characteristic com priviledges
  Flag flags;
};

/// service definition
template <typename Value, uint8_t N> struct Service {
  UUID uuid;
  std::array<Characteristic<Value>, N> characteristics;
};

/// bluetooth low energy abstraction class
class Ble {
public:
  /// @brief antenna output
  enum class Antenna {
    internal,
    external,
  };

  /// @brief constructor
  /// @param device_name advertizing name of device
  /// @param antenna choose which antenna to use
  Ble(std::string _device_name, Antenna antenna = Antenna::internal);

  /// Handle GATT attributes register events
  /// @param ctxt state structure for callback meta data
  /// @param arg additional arguments if given
  void service_register_callback(ble_gatt_register_ctxt *ctxt, void *arg);

  /// gets called when a client subscribes to a characteristic
  /// @param event carries event meta data to process
  void server_subscribe_callback(ble_gap_event *event);

  /// update indication flag for heart rate characteristic
  void send_heart_rate_indication();

  /// @brief nimble base task
  void nimble_host_task();

  /// @brief control advertizing
  void advertize();

  /// @brief handles gap events
  int event_handler(ble_gap_event *event);

  /// start advertizing on demand. stops when connection is established
  void start_advertising();

  /// stop current advertizing in progress
  void stop_advertizing();

private:
  /// switch pin to switch antenna switch on or off
  constexpr static inline gpio_num_t rf_switch_gpio =
      static_cast<gpio_num_t>(3);
  /// switch to select antenna
  constexpr static inline gpio_num_t antenna_switch_gpio =
      static_cast<gpio_num_t>(14);

  void init_nvs();
  void init_gap(std::string _device_name);
  void init_gatt();
  void init_nimble_port();
  void nimble_host_config_init();
  void choose_antenna(Antenna antenna);
  void mtu_event(ble_gap_event *event);
  int event_handler(ble_gap_event *event, void *args);
  void service_register_event(ble_gatt_register_ctxt *ctxt);
  void characteristic_register_event(ble_gatt_register_ctxt *ctxt);
  void descriptor_register_event(ble_gatt_register_ctxt *ctxt);
};
} // namespace ble
