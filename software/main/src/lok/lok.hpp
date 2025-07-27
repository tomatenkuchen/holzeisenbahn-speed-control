/// @file lok.hpp
/// @brief combination class for all functions
/// @license GNU GPL V2

#pragma once

#include "lok/led.hpp"
#include "lok/speed_ctrl.hpp"

namespace lok {

class Lok {
 public:
  struct Config {
    SpeedControl::Config speed_control_config;
  };

  Lok(Config const& _cfg);

  void on_tacho_event();

 private:
  SpeedControl speed_ctrl;
  // Led<2> led;
};

Lok::Lok(Config const& _cfg)
    : speed_ctrl(_cfg.speed_control_config) {}

void Lok::on_tacho_event() { speed_ctrl.on_tacho_event(); }

}  // namespace lok
