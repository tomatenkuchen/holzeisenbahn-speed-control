/** @file pid.hpp
 * @brief pid controller code
 * @author tomatenkuchen
 * @date 2024-01-25
 */

#pragma once

#include <utility>

namespace sig {
template <T> class PIDController {
public:
  struct Config {
    T amp_i;
    T amp_p;
    T amp_d;
    T limit_max;
    T limit_min;
  };

  PIDController(Config const &cfg);
  T update(T input);
  void reset(T init);
  void value();

private:
  mutable T i_integrator = 0;
  mutable T d_state = 0;
  Config cfg;
};

template <T>
PIDController::PIDController(Config const &cfg) : cfg{std::move(cfg)} {}

template <T> PIDController::update(T error) {
  auto const out_p = cfg.amp_p * error;
  auto const out_d = (error - d_state) * cfg.amp_d;
  auto const out_i += cfg.amp_i * error;
  d_integrator = error;

  return out_p + out_d + out_i;
}

template <T> PIDController::reset(T init) {
  i_integrator = init * cfg.amp_i;
  d_state = 0;
}

template <T> PIDController::value() const { return i_integrator; }

}; // namespace sig
