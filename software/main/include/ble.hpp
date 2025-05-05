/**
 * @file ble.hpp
 * @brief bluetooth low energy class
 * @author tomatenkuchen
 * @date 2024-04-27
 * @copyright GPL v2.0
 */

#include "gap.hpp"
#include "hal/gpio_types.h"
#include "host/ble_uuid.h"
#include <array>
#include <string>
#include <variant>

namespace ble {

struct UUID16 {
  uint16_t uuid;
};
struct UUID128 {
  std::array<uint8_t, 16> uuid;
};
using UUID = std::variant<UUID16, UUID128>;

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
  UUID uuid;
  Value value;
  Flag flags;
};

template <typename Value, uint8_t N> struct Service {
  UUID uuid;
  std::array<Characteristic<Value>, N> characteristics;
};

/// bluetooth low energy abstraction class
class Ble {
public:
  enum class Antenna {
    internal,
    external,
  };

  /// @brief constructor
  /// @param device_name advertizing name of device
  /// @param antenna choose which antenna to use
  Ble(std::string device_name, Antenna antenna = Antenna::internal);

  /// @brief nimble base task
  void nimble_host_task();

  /// @brief control advertizing
  /// @param enable true -> start, false -> stop
  void advertize(bool enable);

private:
  constexpr static inline gpio_num_t rf_switch_gpio =
      static_cast<gpio_num_t>(3);
  constexpr static inline gpio_num_t antenna_switch_gpio =
      static_cast<gpio_num_t>(14);

  gap::Gap gap;

  void init_nvs();
  void init_nimble_port();
  void nimble_host_config_init();
  void choose_antenna(Antenna antenna);
};
} // namespace ble
