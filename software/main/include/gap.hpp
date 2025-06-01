/// @file gap.hpp
/// @brief ble gap service interface
/// @author tomatenkuchen
/// @date 2024-04-27
/// @copyright GPL v2.0

#pragma once

#include <string>

namespace gap {

/// initialize gap server and start advertizing
void init(std::string _device_name);

/// start advertizing on demand. stops when connection is established
void start_advertising();

/// stop current advertizing in progress
void stop_advertizing();

} // namespace gap
