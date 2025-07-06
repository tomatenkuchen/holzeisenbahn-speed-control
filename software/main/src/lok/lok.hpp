/// @file lok.hpp
/// @brief combination class for all functions
/// @license GNU GPL V2

#pragma once

#include "lok/speed_ctrl.hpp"

namespace lok {

class Lok {
 public:
  Lok();

 private:
  SpeedControl speed_ctrl;
};  // namespace lok

Lok::Lok() {}

}  // namespace lok
