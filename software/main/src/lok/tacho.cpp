#include <chrono>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "hal/gpio_types.h"
#include "lok/tacho.hpp"

namespace lok {

void MeasureSpeed::init_timer() {
  if (gptimer_new_timer(&cfg.timer_cfg, &timer_handle) != ESP_OK) {
    throw std::runtime_error("measure speed: timer init failed");
  }
  gptimer_enable(timer_handle);
  gptimer_start(timer_handle);

  uint64_t raw_count;
  gptimer_get_raw_count(timer_handle, &raw_count);
  timestamp_latest_tacho_event = std::chrono::microseconds(raw_count);

  ESP_LOGI("tacho", "latest tacho count: %d", raw_count);
}

void MeasureSpeed::init_pin() {
  gpio_config(&cfg.tacho_pin);
  ESP_LOGI("tacho", "interrupt pin configed");

  // change gpio interrupt type for one pin
  gpio_set_intr_type(tacho_input_pin, GPIO_INTR_ANYEDGE);
  ESP_LOGI("tacho", "interrupt type configured");

  auto const install_res = gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
  ESP_LOGI("tacho", "interrupt service installed with res: %d", install_res);

  auto const add_handler_res =
      gpio_isr_handler_add(tacho_input_pin, cfg.tacho_pin_callback, nullptr);
  ESP_LOGI("tacho", "interrupt handler installed with state %d", add_handler_res);
}

MeasureSpeed::MeasureSpeed(Config const &_cfg)
    : cfg{_cfg} {
  ESP_LOGI("tacho", "constructor");
  init_timer();
  init_pin();
}

MeasureSpeed::~MeasureSpeed() {
  gptimer_stop(timer_handle);
  gptimer_disable(timer_handle);
  gptimer_del_timer(timer_handle);
  gpio_uninstall_isr_service();
  gpio_reset_pin(tacho_input_pin);
}

/// execute this function on a tacho event
void MeasureSpeed::on_tacho_event() {
  ESP_LOGI("tacho", "tacho event");

  auto const current_time = get_current_time();
  delta_t_measurements = current_time - timestamp_latest_tacho_event;
  speed_m_per_s = cfg.wheel_circumference_m * 1'000'000. / delta_t_measurements.count();
  timestamp_latest_tacho_event = current_time;
}

float MeasureSpeed::get_speed_m_per_s() const { return speed_m_per_s; }

std::chrono::microseconds MeasureSpeed::get_latest_delta_t() const { return delta_t_measurements; }

std::chrono::microseconds MeasureSpeed::get_current_time() {
  uint64_t new_count;
  gptimer_get_raw_count(timer_handle, &new_count);
  uint64_t const delta_count = new_count;
  return std::chrono::microseconds(delta_count * 1'000'000 / cfg.timer_cfg.resolution_hz);
}

}  // namespace lok
