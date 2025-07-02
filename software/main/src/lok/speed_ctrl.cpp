#include "lok/speed_ctrl.hpp"

namespace lok {

SpeedControl::SpeedControl() {}
SpeedControl::~SpeedControl() {}

void SpeedControl::set_ref_speed_m_per_s(float speed_m_per_s) { speed_ref_m_per_s = speed_m_per_s; }

void SpeedControl::on_tacho_event() {
  measure.on_tacho_event();
  float const current_speed_m_per_s = measure.get_speed_m_per_s();
  float const current_speed_m_per_s_direction =
      speed_ref_m_per_s > 0 ? current_speed_m_per_s : -current_speed_m_per_s;
  float const error_m_per_s = speed_ref_m_per_s - current_speed_m_per_s;
  int32_t const voltage = pid.update(error_m_per_s);

  // set new voltage depending on
  if (speed_ref_m_per_s > 0) {
    inverter.set_duty({voltage, 0, 0});
  } else if (speed_ref_m_per_s < 0) {
    inverter.set_duty({0, voltage, 0});
  } else {
    inverter.set_duty({0, 0, 0});
  }
}

}  // namespace lok
