/**
 * @file ble.hpp
 * @brief bluetooth low energy class
 * @author tomatenkuchen
 * @date 2024-04-27
 * @copyright GPL v2.0
 */

#include "hal/gpio_types.h"
#include <string>
#include <variant>

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

  void init_nvs();
  void init_nimble_port();
  void nimble_host_config_init();
  void choose_antenna(Antenna antenna);
};
