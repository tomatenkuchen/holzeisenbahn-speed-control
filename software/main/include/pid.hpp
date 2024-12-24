/**
 * @file pid.hpp
 * @brief controller module for PID type
 * @author tomatenkuchen
 * @date 2023-03-27
 * @license MIT
 */

#include <concepts>
#include <cstdint>
#include <limits>

/**
 * @brief controller class
 */
template <typename T>
  requires std::integral<T> || std::floating_point<T>
class Pid {
public:
  /** @brief Limit structure to define maximum output of controller */
  struct Limits {
    T max;
    T min;
  };

  /** @brief configuration for I-channel (integral)
   * @param K_i amplification factor for integral channel
   * @param i-channel-init initial value of integrator
   */
  struct ChannelConfig {
    T K;
    T channel_init;
  };

  /** amplification factor for proportional channel */
  T K_p;
  /** channel configuration of i-channel */
  ChannelConfig cfg_i;
  /** channel configuration of d-channel */
  ChannelConfig cfg_d;
  /** output limits of controller */
  Limits output_limits;

  /**
   * @param K_p amplification P-channel (proportional)
   * @param lim limits for outputs
   * @param cfg_i amplification I-channel (integral)
   * @param cfg_d amplification D-channel (derivative) */
  Pid(T K_p,
      Limits lim = {std::numeric_limits<T>::max(),
                    std::numeric_limits<T>::min()},
      ChannelConfig cfg_i = {0, 0}, ChannelConfig cfg_d = {0, 0})
      : i_state{cfg_i.channel_init}, d_state{cfg_d.channel_init}, K_p{K_p},
        cfg_i{cfg_i}, cfg_d{cfg_d}, output_limits{lim} {}

  Pid(Pid const &) = default;
  Pid(Pid &&) = default;
  Pid &operator=(Pid const &) = default;
  Pid &operator=(Pid &&) = default;

  /**
   * @brief operates a new control step on the new data. needs to be exectued in
   * constant time stamps
   * @param e difference between set point and measured value -> control error
   */
  T update(T e) {
    // p-channel
    auto p = e * K_p;
    // d-channel
    auto d = (e - d_state) * cfg_d.K;
    d_state = e;
    // i-channel
    i_state += e * cfg_i.K;
    // sum of channels, limited to output limits
    result = crop_to_limits(p + d + i_state, output_limits);
    // to avoid i-channel windup, also limit i-channel-state
    i_state = crop_to_limits(i_state, output_limits);

    return result;
  }

  /** @return controller output */
  T output() const { return result; }

  /** @brief reset controller to initial settings */
  void reset() {
    i_state = cfg_i.channel_init;
    d_state = cfg_d.channel_init;
  }

private:
  /** state variable for i-channel */
  T i_state;
  /** state variable for d-channel */
  T d_state;
  /** result of control */
  T result = 0;

  /** @brief returns input when input is within limits. returns limit if not */
  static T crop_to_limits(T input, Limits limits) {
    return (input > limits.max ? limits.max
                               : (input < limits.min ? limits.min : input));
  }
};
