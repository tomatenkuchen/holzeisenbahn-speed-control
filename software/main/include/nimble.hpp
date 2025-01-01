/**
 * @file nimble.hpp
 * @brief ble nimble framework glue code
 * @author tomatenkuchen
 * @date 2024-12-30
 * @license GPLv2 @see license.md
 */

#pragma once

namespace ble {
class Nimble {
public:
  Nimble();

private:
  void init_nimble_host();
  void prepare_nvm();
  void nimble_host_config_init();
};
} // namespace ble
