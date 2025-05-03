#pragma once

#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"
#include <string>

#define BLE_GAP_APPEARANCE_GENERIC_TAG 0x0200
#define BLE_GAP_URI_PREFIX_HTTPS 0x17
#define BLE_GAP_LE_ROLE_PERIPHERAL 0x00

/// @brief initialize advertizing
void adv_init();

/// @brief initialize gap server
/// @param device_name advertizing name
void gap_init(std::string device_name);
