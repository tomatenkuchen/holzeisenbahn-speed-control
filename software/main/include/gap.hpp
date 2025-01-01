/**
 * @file gap.hpp
 * @brief ble gap layer helper class
 * @author tomatenkuchen
 * @date 2024-12-29
 * @license GPLv2 @see license.md
 */

#pragma once

namespace ble {
class GAP {
  uint8_t own_addr_type;
  uint8_t address_value[6] = {0};
  std::string app_name;

public:
  GAP(std::string const &app_name);

  void advertize(bool init = false);

private:
  void init_advertising();
};
} // namespace ble
