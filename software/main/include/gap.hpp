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
public:
  GAP(std::string const &app_name);
  ~GAP();

  void reinit_advertising();
  void advertize();

private:
  uint8_t own_addr_type;
  uint8_t address_value[6] = {0};
};
} // namespace ble
