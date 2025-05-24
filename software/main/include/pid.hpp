/** @file pid.hpp
 * @brief pid controller code
 * @author tomatenkuchen
 * @date 2024-01-25
 */

#pragma once

namespace sig {

/// @brief PID controller class for signal control
template <typename T> class PIDController {
public:
  struct Config {
    /// amplification factor for integrator
    T amp_i;
    /// amplification factor for proportional
    T amp_p;
    /// amplification factor for differential
    T amp_d;
    /// max controller output
    T limit_max;
    /// min controller output
    T limit_min;
  };

  /// @brief constructs controller by config
  /// @param cfg configuration
  PIDController(Config const &cfg);

  /// @brief feed new control error to controller and calculate response
  /// @param input new control error input
  /// @return controller output (sometimes shown as y in literature)
  T update(T input);

  /// @brief resets state in controller
  /// @param init new initial value for integrator
  void reset(T init);

  /// @return current value of integrator state without new input
  T value() const;

private:
  /// integrator state
  mutable T i_state = 0;
  /// differentiator state
  mutable T d_state = 0;
  /// configuration
  Config cfg;
};

template <typename T>
PIDController<T>::PIDController(PIDController<T>::Config const &_cfg)
    : cfg{_cfg} {}

template <typename T> T PIDController<T>::update(T input) {
  auto const out_p = cfg.amp_p * input;

  i_state += cfg.amp_i * input;

  auto const out_d = (input - d_state) * cfg.amp_d;
  d_state = input;

  return out_p + out_d + i_state;
}

template <typename T> void PIDController<T>::reset(T init) {
  i_state = init * cfg.amp_i;
  d_state = 0;
}

template <typename T> T PIDController<T>::value() const { return i_state; }

}; // namespace sig
