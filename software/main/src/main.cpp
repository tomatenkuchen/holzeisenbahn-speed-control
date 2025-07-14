#include <chrono>
#include <stdexcept>
#include <string>

#include "ble.hpp"
#include "esp_log.h"
#include "esp_log_level.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_store.h"
#include "lok/led.hpp"
#include "lok/speed_ctrl.hpp"
#include "sdkconfig.h"

using namespace std::chrono_literals;

namespace {

constexpr std::string TAG = "main";
constexpr uint8_t num_of_rgb_leds = 2;

extern "C" void ble_store_config_init();

ble::Ble *ble_ptr;
lok::Led<num_of_rgb_leds> *led_ptr;
lok::SpeedControl *speed_ctrl_ptr;

int led1_chr_access(uint16_t conn_handle, uint16_t attr_handle, ble_gatt_access_ctxt *ctxt,
                    void *arg);
int led2_chr_access(uint16_t conn_handle, uint16_t attr_handle, ble_gatt_access_ctxt *ctxt,
                    void *arg);

/* Automation IO service */
const ble_uuid16_t auto_io_svc_uuid = BLE_UUID16_INIT(0x1815);

uint16_t led1_chr_val_handle;

const ble_uuid128_t led1_chr_uuid = BLE_UUID128_INIT(
    0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15, 0xde, 0xef, 0x12, 0x12, 0x25, 0x15, 0x00, 0x00);

ble_gatt_chr_def const led1_characteristic = {
    .uuid = &led1_chr_uuid.u,
    .access_cb = led1_chr_access,
    .flags = BLE_GATT_CHR_F_WRITE,
    .val_handle = &led1_chr_val_handle,
};

uint16_t led2_chr_val_handle;

const ble_uuid128_t led2_chr_uuid = BLE_UUID128_INIT(
    0x24, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15, 0xde, 0xef, 0x12, 0x12, 0x25, 0x15, 0x00, 0x00);

ble_gatt_chr_def const led2_characteristic = {
    .uuid = &led2_chr_uuid.u,
    .access_cb = led2_chr_access,
    .flags = BLE_GATT_CHR_F_WRITE,
    .val_handle = &led2_chr_val_handle,
};

std::array<ble_gatt_chr_def, 3> led_characteristics = {
    led1_characteristic,
    led2_characteristic,
    {0},
};

ble_gatt_svc_def const led_service = {
    .type = BLE_GATT_SVC_TYPE_PRIMARY,
    .uuid = &auto_io_svc_uuid.u,
    .characteristics = led_characteristics.data(),
};

std::array<ble_gatt_svc_def, 2> ble_services = {
    led_service,
    {0},
};

int led1_chr_access(uint16_t conn_handle, uint16_t attr_handle, ble_gatt_access_ctxt *ctxt,
                    void *arg) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
    return BLE_ERR_UNSPECIFIED;
  }

  // Verify attribute handle
  if (attr_handle != led1_chr_val_handle) {
    throw std::runtime_error("unexpected access operation on led characteristic");
  }

  // Verify access buffer length
  if (ctxt->om->om_len != 1) {
    throw std::runtime_error("unexpected access operation on led characteristic");
  }

  // Turn the LED on or off according to the operation bit
  if (ctxt->om->om_data[0]) {
    ESP_LOGI("main", "led1 turned on");
    led_ptr->set_color(lok::color::red, 0, 1s);
  } else {
    ESP_LOGI("main", "led1 turned off");
    led_ptr->set_color(lok::color::off, 0, 1s);
  }

  return 0;
}

int led2_chr_access(uint16_t conn_handle, uint16_t attr_handle, ble_gatt_access_ctxt *ctxt,
                    void *arg) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
    return BLE_ERR_UNSPECIFIED;
  }

  // Verify attribute handle
  if (attr_handle != led2_chr_val_handle) {
    throw std::runtime_error("unexpected access operation on led characteristic");
  }

  // Verify access buffer length
  if (ctxt->om->om_len != 1) {
    throw std::runtime_error("unexpected access operation on led characteristic");
  }

  // Turn the LED on or off according to the operation bit
  if (ctxt->om->om_data[0]) {
    ESP_LOGI("main", "led2 turned on");
    led_ptr->set_color(lok::color::white, 1, 1s);
  } else {
    ESP_LOGI("main", "led2 turned off");
    led_ptr->set_color(lok::color::off, 1, 1s);
  }

  return 0;
}

void measure_pin_callback(void *params) { speed_ctrl_ptr->on_tacho_event(); }

void speed_control_task(void *param) {
  lok::SpeedControl::Config cfg = {
      .tacho_pin_callback = measure_pin_callback,
  };
  lok::SpeedControl speed_control(cfg);
  speed_ctrl_ptr = &speed_control;

  while (true) {
    vTaskDelay(100);
  }

  vTaskDelete(NULL);
}

/// callback routine for gap event servicing
/// @param event type of event that occured
/// @param args additional info besides event data. not used by any callback but
/// required by callback type
int event_handler(ble_gap_event *event, void *args) {
  ESP_LOGI("main", "event callback");
  ble_ptr->event_handler(event);

  return 0;
}

void on_stack_reset(int reason) { ESP_LOGI("main", "ble stack reset"); }

void on_stack_sync() { ble_ptr->start_advertising(); }

void service_register_callback(ble_gatt_register_ctxt *ctxt, void *arg) {
  ESP_LOGI(TAG.c_str(), "gatt service register callback called");
}

void add_callbacks() {
  ble_hs_cfg.reset_cb = on_stack_reset;
  ble_hs_cfg.sync_cb = on_stack_sync;
  ble_hs_cfg.gatts_register_cb = service_register_callback;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

  ble_store_config_init();
}

void ble_nimble_task(void *param) {
  constexpr gpio_num_t led_gpio = static_cast<gpio_num_t>(15);
  constexpr lok::Led<num_of_rgb_leds>::Config config = {
      .pins =
          {
              {
                  {
                      .red_pin = gpio_num_t(0),
                      .red_channel = LEDC_CHANNEL_0,
                      .green_pin = gpio_num_t(1),
                      .green_channel = LEDC_CHANNEL_1,
                      .blue_pin = gpio_num_t(2),
                      .blue_channel = LEDC_CHANNEL_2,
                  },
                  {
                      .red_pin = gpio_num_t(3),
                      .red_channel = LEDC_CHANNEL_3,
                      .green_pin = gpio_num_t(4),
                      .green_channel = LEDC_CHANNEL_4,
                      .blue_pin = gpio_num_t(5),
                      .blue_channel = LEDC_CHANNEL_5,
                  },
              },
          },
      .timer = LEDC_TIMER_0,
      .timer_mode = LEDC_LOW_SPEED_MODE,
      .timer_resolution = LEDC_TIMER_13_BIT,
      .timer_frequency = 4000,
  };

  lok::Led<num_of_rgb_leds> Led(config);
  led_ptr = &Led;

  ESP_LOGI("main", "led init complete");

  ble::Ble ble("henri-lok", event_handler, ble_services.data(), ble::Ble::Antenna::external);
  ble_ptr = &ble;

  ESP_LOGI("main", "ble init complete");

  add_callbacks();

  ESP_LOGI("main", "callbacks added");

  ble.nimble_host_task();

  while (true) {
    ESP_LOGI("main", "ble heart beat");
    vTaskDelay(200);
  }
}

}  // namespace

extern "C" void app_main() {
  try {
    xTaskCreate(ble_nimble_task, "ble task", 8 * 1024, NULL, 5, NULL);
    xTaskCreate(speed_control_task, "Speed Control", 8 * 1024, NULL, 5, NULL);
  } catch (std::runtime_error &e) {
    std::string const err_msg = e.what();
    ESP_LOGE("main", "error: %s", err_msg.c_str());
  } catch (...) {
    ESP_LOGE("main", "unknown error occured");
  }
}
