/// @file ble.hpp
/// @brief bluetooth low energy class
/// @copyright GPL v2.0

#include <array>
#include <string>
#include <variant>

#include "hal/gpio_types.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "services/gatt/ble_svc_gatt.h"

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

/// flags defining the type of communication that is desired with the
/// characteristic
template <typename Value>
struct Characteristic {
  /// characteristic uuid
  UUID uuid;
  /// actual value
  Value value;
  /// characteristic com priviledges
  Flag flags;
};

/// service definition
template <typename Value, uint8_t N>
struct Service {
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
  /// @param _external_event_handler event handler for gap. needs to be staticall defined. use ble's
  /// @param services services proveded by ble
  /// @param antenna choose which antenna to use
  Ble(std::string _device_name, ble_gap_event_fn *_external_event_handler,
      ble_gatt_svc_def *services, Antenna antenna = Antenna::internal);

  /// gets called when a client subscribes to a characteristic
  /// @param event carries event meta data to process
  void service_subscribe_callback(ble_gap_event *event);

  /// @brief nimble base task
  void nimble_host_task();

  /// @brief handles gap events
  void event_handler(ble_gap_event *event);

  /// start advertizing on demand. stops when connection is established
  void start_advertising();

  /// stop current advertizing in progress
  void stop_advertizing();

 private:
  /// switch pin to switch antenna switch on or off
  constexpr static inline gpio_num_t rf_switch_gpio = static_cast<gpio_num_t>(3);
  /// switch to select antenna
  constexpr static inline gpio_num_t antenna_switch_gpio = static_cast<gpio_num_t>(14);

  constexpr static inline std::string_view esp_uri = "\x17//espressif.com";
  uint8_t own_addr_type = 0;
  uint8_t addr_val[6] = {0};
  std::string device_name;
  ble_gap_event_fn *external_event_handler;

  ble_hs_adv_fields adv_fields = {
      .flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP,

      // Set device tx power
      .tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO,
      .tx_pwr_lvl_is_present = 1,

      // Set device appearance
      .appearance = 0x0200,
      .appearance_is_present = 1,

      // Set device LE role
      .le_role = 0,
      .le_role_is_present = 1,
  };

  ble_hs_adv_fields rsp_fields = {
      .adv_itvl = BLE_GAP_ADV_ITVL_MS(500),
      .adv_itvl_is_present = 1,

      .device_addr = addr_val,
      .device_addr_type = own_addr_type,
      .device_addr_is_present = 1,

      .uri = reinterpret_cast<uint8_t const *>(esp_uri.data()),
      .uri_len = static_cast<uint8_t>(esp_uri.size()),
  };

  ble_gap_adv_params adv_params = {
      .conn_mode = BLE_GAP_CONN_MODE_UND,
      .disc_mode = BLE_GAP_DISC_MODE_GEN,

      // Set advertising interval
      .itvl_min = BLE_GAP_ADV_ITVL_MS(500),
      .itvl_max = BLE_GAP_ADV_ITVL_MS(510),
  };

  void init_nvs();

  void init_gap(std::string _device_name);

  ///
  void init_gatt(ble_gatt_svc_def *service);

  void init_nimble_port();

  void nimble_host_config_init();

  void choose_antenna(Antenna antenna);

  /// gets called when a service is registered
  void service_register_event(ble_gatt_register_ctxt const *const ctxt);

  void characteristic_register_event(ble_gatt_register_ctxt *ctxt);

  void descriptor_register_event(ble_gatt_register_ctxt *ctxt);

  /// print address
  void format_addr(char *addr_str, uint8_t addr[]);

  /// print connection description
  void print_conn_desc(ble_gap_conn_desc *desc);

  /// handle connect event
  int connect_event(ble_gap_event *event);

  /// handle disconnect event
  void disconnect_event(ble_gap_event *event);

  ///
  void update_event(ble_gap_event *event);

  ///
  void advertizing_complete_event(ble_gap_event *event);

  ///
  void notify_event(ble_gap_event *event);

  void subscribe_event(ble_gap_event *event);

  void mtu_event(ble_gap_event *event);

  /// callback routine for gap event servicing
  /// @param event type of event that occured
  /// @param args additional info besides event data. not used by any callback but
  /// required by callback type
  int event_handler(ble_gap_event *event, void *args);
};

}  // namespace ble
