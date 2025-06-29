#include <cstdint>
#include <limits>
#include <stdexcept>

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "lok/tacho.hpp"

namespace lok {

MeasureSpeed::MeasureSpeed(Config const &_cfg) : cfg{_cfg} {
  if (gptimer_new_timer(&cfg.timer_cfg, &timer_handle) != ESP_OK) {
    throw std::runtime_error("measure speed: timer init failed");
  }
  gptimer_enable(timer_handle);
  gptimer_start(timer_handle);
  gptimer_get_raw_count(timer_handle, &timestamp_latest_tacho_event);
}

MeasureSpeed::~MeasureSpeed() {
  gptimer_stop(timer_handle);
  gptimer_disable(timer_handle);
  gptimer_del_timer(timer_handle);
}

/// execute this function on a tacho event
void MeasureSpeed::on_tacho_event() {
  uint64_t new_count;
  gptimer_get_raw_count(timer_handle, &new_count);
  uint64_t const delta_count = new_count - timestamp_latest_tacho_event;
  speed_m_per_s = cfg.wheel_circumpherance_m * cfg.timer_cfg.resolution_hz / delta_count;
  timestamp_latest_tacho_event = new_count;
}

float MeasureSpeed::get_speed_m_per_s() const { return speed_m_per_s; }

void MeasureSpeed::x() {
  gpio_config_t x = {
      .intr_type = GPIO_INTR_POSEDGE,
      .mode = GPIO_MODE_OUTPUT,
      .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
      .pull_down_en = 0,
      .pull_up_en = 0,
      .pin_bit_mask = GPIO_INPUT_PIN_SEL,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = 1,
  };
  gpio_config(&io_conf);
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void *)GPIO_INPUT_IO_0);
}  // namespace lok

}  // namespace lok
