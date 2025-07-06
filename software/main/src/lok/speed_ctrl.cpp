#include <array>

#include "lok/inverter.hpp"
#include "lok/speed_ctrl.hpp"

namespace lok {

SpeedControl::SpeedControl() : inverter(inverter_cfg), pid(pid_cfg) {}

void SpeedControl::set_ref_speed_m_per_s(float speed_m_per_s) { speed_ref_m_per_s = speed_m_per_s; }

void SpeedControl::on_tacho_event() {
  measure.on_tacho_event();
  auto const delta_t = measure.get_latest_delta_t();
  float const current_speed_m_per_s = measure.get_speed_m_per_s();
  float const current_speed_m_per_s_direction =
      speed_ref_m_per_s > 0 ? current_speed_m_per_s : -current_speed_m_per_s;
  float const error_m_per_s = speed_ref_m_per_s - current_speed_m_per_s;
  auto const voltage = pid.update(error_m_per_s);  // todo: add , delta_t);
  auto const voltage_mV = static_cast<uint32_t>(voltage * 1000);

  // set new voltage
  if (voltage > 0) {
    std::array<MilliVolts, 3> v = {{voltage_mV, 0, 0}};
    inverter.set_voltages(v);
  } else {
    std::array<MilliVolts, 3> v = {{0, voltage_mV, 0}};
    inverter.set_voltages(v);
  }
}

}  // namespace lok
