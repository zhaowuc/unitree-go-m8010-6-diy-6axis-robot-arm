#include "go_m8010_driver.hpp"
#include "j1_final_trajectory.hpp"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <sys/prctl.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
using go_m8010::CommandState;
using go_m8010::GoM8010Driver;
using go_m8010::State;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDtS = 0.01;
constexpr double kKp = 0.50;
constexpr double kKd = 0.05;
constexpr double kMaxVelocityDegS = 12.0;
constexpr double kMaxAccelerationDegS2 = 40.0;
constexpr double kGearRatio = 6.3299999237060547;
constexpr double kActiveDeadlineS = 15.0;
constexpr const char* kRevision = "V15_19D_J1_FINAL_MOTION";
constexpr const char* kConfirmation = "READY_FOR_J1_FINAL_MOTION_TEST";
constexpr const char* kPort =
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_"
    "FTASQA6F-if03-port0";

volatile std::sig_atomic_t g_stop_requested = 0;
void signal_handler(int) { g_stop_requested = 1; }
double radians(double degrees) { return degrees * kPi / 180.0; }
double degrees(double radians_value) { return radians_value * 180.0 / kPi; }
bool at_least(double value, double limit) {
  return value >= limit - 1e-9;
}

struct Options {
  bool execute = false;
  bool self_test = false;
  std::string confirmation;
  std::string output;
};

bool valid_output_path(const std::string& path) {
  constexpr const char* prefix = "/tmp/v15_19d_j1_final_";
  if (path.rfind(prefix, 0) != 0 || path.size() <= std::strlen(prefix) + 4 ||
      path.size() > 160 || path.substr(path.size() - 4) != ".csv") {
    return false;
  }
  const std::size_t begin = std::strlen(prefix);
  const std::size_t end = path.size() - 4;
  for (std::size_t i = begin; i < end; ++i) {
    const unsigned char value = static_cast<unsigned char>(path[i]);
    if (!(std::isalnum(value) != 0 || value == '_' || value == '-')) {
      return false;
    }
  }
  return true;
}

Options parse_options(int argc, char** argv) {
  Options result;
  bool saw_execute = false, saw_self_test = false;
  bool saw_confirm = false, saw_output = false;
  for (int i = 1; i < argc; ++i) {
    const std::string argument(argv[i]);
    const auto value = [&]() {
      if (++i >= argc) throw std::runtime_error("MISSING_ARGUMENT_VALUE");
      return std::string(argv[i]);
    };
    if (argument == "--execute" && !saw_execute) {
      result.execute = saw_execute = true;
    } else if (argument == "--self-test" && !saw_self_test) {
      result.self_test = saw_self_test = true;
    } else if (argument == "--confirm" && !saw_confirm) {
      saw_confirm = true;
      result.confirmation = value();
    } else if (argument == "--output" && !saw_output) {
      saw_output = true;
      result.output = value();
    } else {
      throw std::runtime_error("UNKNOWN_OR_DUPLICATE_ARGUMENT");
    }
  }
  if (result.self_test) {
    if (result.execute || saw_confirm || saw_output) {
      throw std::runtime_error("SELF_TEST_ARGUMENT_CONFLICT");
    }
    return result;
  }
  if (!result.execute) {
    if (saw_confirm || saw_output) {
      throw std::runtime_error("DRY_RUN_ARGUMENT_CONFLICT");
    }
    return result;
  }
  if (result.confirmation != kConfirmation ||
      !valid_output_path(result.output)) {
    throw std::runtime_error("EXECUTION_AUTHORITY_INVALID");
  }
  return result;
}

class ProcessLock {
 public:
  ProcessLock()
      : fd_(open("/tmp/go_m8010_ftasqa6f_channel3.lock",
                 O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600)) {
    if (fd_ < 0 || flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      throw std::runtime_error("HARDWARE_LOCK_FAILED");
    }
  }
  ~ProcessLock() {
    if (fd_ >= 0) {
      flock(fd_, LOCK_UN);
      close(fd_);
    }
  }
  ProcessLock(const ProcessLock&) = delete;
  ProcessLock& operator=(const ProcessLock&) = delete;

 private:
  int fd_ = -1;
};

class ContinuousLog {
 public:
  explicit ContinuousLog(const std::string& path)
      : fd_(open(path.c_str(), O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC |
                                  O_NOFOLLOW,
                 0600)) {
    if (fd_ < 0) throw std::runtime_error("LOG_OPEN_FAILED");
    write_all(
        "stage,sequence,monotonic_s,send_recv,correct,response_id,"
        "feedback_valid,feedback_mode,q_raw,q_raw_unwrapped,"
        "q_actual_joint_deg,qdot_single_deg_s,qdot_fast_deg_s,"
        "qdot_slow_deg_s,velocity_valid,dq_sdk,tau_sdk,temp_c,merror,"
        "command_mode,tau_command,dq_command_motor,q_command_motor,kp,kd,"
        "q_des_joint_deg,dq_des_joint_deg,tracking_error_deg,"
        "invalid_consecutive,fast35_consecutive,position_warning,"
        "speed_warning,torque_warning,temperature_warning,status\n");
  }
  ~ContinuousLog() {
    if (fd_ >= 0) close(fd_);
  }
  ContinuousLog(const ContinuousLog&) = delete;
  ContinuousLog& operator=(const ContinuousLog&) = delete;

  void record(const std::string& row) {
    // Continuous evidence is written on every cycle.  The deque is a separate
    // last-200-row emergency ring, never a delayed-write buffer.
    write_all(row);
    ring_.push_back(row);
    while (ring_.size() > 200) ring_.pop_front();
  }

  void event(const std::string& status) {
    std::ostringstream row;
    row << "EVENT,0,nan,0,0,255,0,255,nan,nan,nan,nan,nan,nan,0,nan,"
           "nan,0,0,0,0,nan,nan,0,0,nan,nan,nan,0,0,0,0,0,0,"
        << status << '\n';
    record(row.str());
  }

  void sync() {
    if (fsync(fd_) != 0) throw std::runtime_error("LOG_FSYNC_FAILED");
  }

 private:
  void write_all(const std::string& data) {
    std::size_t position = 0;
    while (position < data.size()) {
      const ssize_t count =
          write(fd_, data.data() + position, data.size() - position);
      if (count < 0 && errno == EINTR) continue;
      if (count <= 0) throw std::runtime_error("LOG_WRITE_FAILED");
      position += static_cast<std::size_t>(count);
    }
  }
  int fd_ = -1;
  std::deque<std::string> ring_;
};

struct LegMetrics {
  double peak_deg = std::numeric_limits<double>::quiet_NaN();
  double max_slow_speed_deg_s = 0.0;
  double duration_s = std::numeric_limits<double>::quiet_NaN();
};

struct RunStats {
  LegMetrics positive;
  LegMetrics negative;
  double first_return_error_deg = std::numeric_limits<double>::quiet_NaN();
  double final_return_error_deg = std::numeric_limits<double>::quiet_NaN();
  double max_abs_angle_deg = 0.0;
  double max_fast_deg_s = 0.0;
  double max_slow_deg_s = 0.0;
  double max_tracking_deg = 0.0;
  double max_abs_tau = 0.0;
  int max_temperature_c = std::numeric_limits<int>::min();
  std::uint64_t merror_count = 0;
  std::uint64_t invalid_frames = 0;
  int max_invalid_consecutive = 0;
  bool position_warning = false;
  bool speed_warning = false;
  bool torque_warning = false;
  bool temperature_warning = false;
  int final_feedback_mode = -1;
};

struct SafetyTrip : std::runtime_error {
  explicit SafetyTrip(std::string why, bool power_off = false)
      : std::runtime_error(why), reason(std::move(why)),
        manual_power_off(power_off) {}
  std::string reason;
  bool manual_power_off = false;
};

void require_foc_entry_near_session_zero(double anchor_deg) {
  if (!std::isfinite(anchor_deg) || std::abs(anchor_deg) >= 1.0) {
    throw SafetyTrip("FOC_ENTRY_SESSION_ZERO_OUTSIDE_1_DEG");
  }
}

class DurationGate {
 public:
  bool update(bool condition, double now_s, double duration_s) {
    resume(now_s);
    if (!condition) {
      started_.reset();
      return false;
    }
    if (!started_) started_ = now_s;
    return now_s - *started_ >= duration_s - 1e-9;
  }

  void pause(double now_s) {
    if (!paused_) paused_ = now_s;
  }

  void resume(double now_s) {
    if (!paused_) return;
    if (started_ && now_s > *paused_) {
      *started_ += now_s - *paused_;
    }
    paused_.reset();
  }

 private:
  std::optional<double> started_;
  std::optional<double> paused_;
};

class SafetyMonitor {
 public:
  explicit SafetyMonitor(RunStats& stats) : stats_(stats) {}

  void observe(const State& state, int expected_mode, bool reference_defined,
               bool active_foc, bool foc_entry, double entry_anchor_deg,
               double q_des_deg, double final_target_deg) {
    if (!state.feedback_valid) {
      pause_duration_gates(state.monotonic_s);
      ++stats_.invalid_frames;
      ++invalid_consecutive_;
      stats_.max_invalid_consecutive =
          std::max(stats_.max_invalid_consecutive, invalid_consecutive_);
      if (state.send_recv && state.correct && state.motor_id == 0 &&
          state.merror != 0) {
        ++stats_.merror_count;
        throw SafetyTrip("MERROR_NONZERO");
      }
      if (state.send_recv && state.correct && state.motor_id == 0) {
        stats_.max_temperature_c =
            std::max(stats_.max_temperature_c, state.temperature_c);
        if (state.temperature_c < 0) {
          throw SafetyTrip("NEGATIVE_TEMPERATURE_INVALID");
        }
        if (state.temperature_c >= 70) {
          throw SafetyTrip("TEMPERATURE_70C_GATE");
        }
        if (std::isfinite(state.tau_sdk)) {
          stats_.max_abs_tau =
              std::max(stats_.max_abs_tau, std::abs(state.tau_sdk));
          if (at_least(std::abs(state.tau_sdk), 5.0)) {
            throw SafetyTrip("TAU_FEEDBACK_5_GATE");
          }
        }
      }
      if (invalid_consecutive_ >= 5) {
        throw SafetyTrip("FIVE_CONSECUTIVE_INVALID_FRAMES");
      }
      return;  // Freeze all duration gates and fast-window state.
    }
    resume_duration_gates(state.monotonic_s);
    invalid_consecutive_ = 0;
    update_common(state, reference_defined);

    const double q_deg = degrees(state.q_joint);
    const double fast = std::abs(degrees(state.qdot_joint_fast));
    const double slow = std::abs(degrees(state.qdot_joint_slow));
    const double tracking = std::abs(q_des_deg - q_deg);

    // Statistics are latched before any gate can throw so a fault report does
    // not understate the actual trigger frame.
    if (active_foc && std::isfinite(tracking)) {
      stats_.max_tracking_deg = std::max(stats_.max_tracking_deg, tracking);
    }

    // Position authority is always evaluated before every other active gate.
    if (reference_defined && at_least(std::abs(q_deg), 20.0)) {
      throw SafetyTrip("ABSOLUTE_20_DEG_POSITION_EMERGENCY", true);
    }
    if (reference_defined && at_least(std::abs(q_deg), 18.0)) {
      throw SafetyTrip("ABSOLUTE_18_DEG_POSITION_GATE");
    }
    if (state.merror != 0) throw SafetyTrip("MERROR_NONZERO");
    if (state.mode != static_cast<unsigned>(expected_mode)) {
      throw SafetyTrip("FEEDBACK_MODE_MISMATCH");
    }
    if (state.temperature_c < 0) throw SafetyTrip("NEGATIVE_TEMPERATURE_INVALID");
    if (state.temperature_c >= 70) throw SafetyTrip("TEMPERATURE_70C_GATE");

    if (reference_defined && std::abs(q_deg) > 15.0) {
      stats_.position_warning = true;
    }
    if (std::abs(state.tau_sdk) > 2.0) stats_.torque_warning = true;
    if (state.temperature_c >= 60) stats_.temperature_warning = true;

    if (reference_defined && state.velocity_valid) {
      if (at_least(fast, 40.0)) {
        throw SafetyTrip("FAST_40_DEG_S_EMERGENCY");
      }
      if (at_least(fast, 35.0)) {
        ++fast35_consecutive_;
        if (fast35_consecutive_ >= 2) {
          throw SafetyTrip("FAST_35_DEG_S_TWO_WINDOWS");
        }
      } else {
        fast35_consecutive_ = 0;
      }
      if (slow_hard_.update(at_least(slow, 30.0), state.monotonic_s,
                            0.100)) {
        throw SafetyTrip("SLOW_30_DEG_S_100MS_GATE");
      }
      if (slow_warning_.update(slow > 20.0, state.monotonic_s, 0.150)) {
        stats_.speed_warning = true;
      }
    }

    if (at_least(std::abs(state.tau_sdk), 5.0)) {
      throw SafetyTrip("TAU_FEEDBACK_5_GATE");
    }
    if (!active_foc) return;
    if (!state.velocity_valid) {
      throw SafetyTrip("VELOCITY_ESTIMATOR_UNAVAILABLE_DURING_FOC");
    }
    if (at_least(tracking, 8.0)) {
      throw SafetyTrip("TRACKING_ERROR_8_DEG_GATE");
    }
    if (tracking_hard_.update(tracking > 5.0, state.monotonic_s, 0.300)) {
      throw SafetyTrip("TRACKING_ERROR_OVER_5_DEG_300MS");
    }
    const bool stall = std::abs(final_target_deg - q_deg) > 5.0 &&
                       slow < 0.5 && tracking > 5.0;
    if (stall_gate_.update(stall, state.monotonic_s, 1.0)) {
      throw SafetyTrip("J1_REAL_STALL_WITH_KP0P50");
    }
    if (foc_entry) {
      if (std::abs(q_deg - entry_anchor_deg) > 1.0) {
        throw SafetyTrip("FOC_ENTRY_DISPLACEMENT_OVER_1_DEG");
      }
      if (slow > 10.0) throw SafetyTrip("FOC_ENTRY_SLOW_SPEED_OVER_10");
    }
  }

  const char* observe_brake(const State& state, bool reference_defined,
                            bool& manual_power_off) {
    if (!state.feedback_valid) {
      pause_duration_gates(state.monotonic_s);
      ++stats_.invalid_frames;
      ++invalid_consecutive_;
      stats_.max_invalid_consecutive =
          std::max(stats_.max_invalid_consecutive, invalid_consecutive_);
      if (state.send_recv && state.correct && state.motor_id == 0 &&
          state.merror != 0) {
        ++stats_.merror_count;
        return "MERROR_NONZERO_DURING_BRAKE";
      }
      if (state.send_recv && state.correct && state.motor_id == 0) {
        stats_.max_temperature_c =
            std::max(stats_.max_temperature_c, state.temperature_c);
        if (state.temperature_c < 0) {
          return "NEGATIVE_TEMPERATURE_DURING_BRAKE";
        }
        if (state.temperature_c >= 70) {
          return "TEMPERATURE_70C_DURING_BRAKE";
        }
        if (std::isfinite(state.tau_sdk)) {
          stats_.max_abs_tau =
              std::max(stats_.max_abs_tau, std::abs(state.tau_sdk));
          if (at_least(std::abs(state.tau_sdk), 5.0)) {
            return "TAU_FEEDBACK_5_DURING_BRAKE";
          }
        }
      }
      return invalid_consecutive_ >= 5
                 ? "FIVE_CONSECUTIVE_INVALID_FRAMES_DURING_BRAKE"
                 : nullptr;
    }
    resume_duration_gates(state.monotonic_s);
    invalid_consecutive_ = 0;
    update_common(state, reference_defined);
    const double q_deg = degrees(state.q_joint);
    const double fast = std::abs(degrees(state.qdot_joint_fast));
    const double slow = std::abs(degrees(state.qdot_joint_slow));
    if (reference_defined && at_least(std::abs(q_deg), 20.0)) {
      manual_power_off = true;
      return "ABSOLUTE_20_DEG_POSITION_DURING_BRAKE";
    }
    if (reference_defined && at_least(std::abs(q_deg), 18.0)) {
      return "ABSOLUTE_18_DEG_POSITION_DURING_BRAKE";
    }
    if (state.merror != 0) return "MERROR_NONZERO_DURING_BRAKE";
    if (state.temperature_c < 0) return "NEGATIVE_TEMPERATURE_DURING_BRAKE";
    if (state.temperature_c >= 70) return "TEMPERATURE_70C_DURING_BRAKE";
    if (reference_defined && std::abs(q_deg) > 15.0) {
      stats_.position_warning = true;
    }
    if (std::abs(state.tau_sdk) > 2.0) stats_.torque_warning = true;
    if (state.temperature_c >= 60) stats_.temperature_warning = true;
    if (reference_defined && state.velocity_valid) {
      if (at_least(fast, 40.0)) return "FAST_40_DEG_S_DURING_BRAKE";
      if (at_least(fast, 35.0)) {
        ++fast35_consecutive_;
        if (fast35_consecutive_ >= 2) {
          return "FAST_35_DEG_S_TWO_WINDOWS_DURING_BRAKE";
        }
      } else {
        fast35_consecutive_ = 0;
      }
      if (slow_hard_.update(at_least(slow, 30.0), state.monotonic_s,
                            0.100)) {
        return "SLOW_30_DEG_S_100MS_DURING_BRAKE";
      }
      if (slow_warning_.update(slow > 20.0, state.monotonic_s, 0.150)) {
        stats_.speed_warning = true;
      }
    }
    if (at_least(std::abs(state.tau_sdk), 5.0)) {
      return "TAU_FEEDBACK_5_DURING_BRAKE";
    }
    return nullptr;
  }

  int invalid_consecutive() const noexcept { return invalid_consecutive_; }
  int fast35_consecutive() const noexcept { return fast35_consecutive_; }

 private:
  void pause_duration_gates(double now_s) {
    slow_hard_.pause(now_s);
    slow_warning_.pause(now_s);
    tracking_hard_.pause(now_s);
    stall_gate_.pause(now_s);
  }

  void resume_duration_gates(double now_s) {
    slow_hard_.resume(now_s);
    slow_warning_.resume(now_s);
    tracking_hard_.resume(now_s);
    stall_gate_.resume(now_s);
  }

  void update_common(const State& state, bool reference_defined) {
    if (state.merror != 0) ++stats_.merror_count;
    stats_.max_abs_tau = std::max(stats_.max_abs_tau, std::abs(state.tau_sdk));
    stats_.max_temperature_c =
        std::max(stats_.max_temperature_c, state.temperature_c);
    if (reference_defined) {
      stats_.max_abs_angle_deg =
          std::max(stats_.max_abs_angle_deg, std::abs(degrees(state.q_joint)));
      if (state.velocity_valid) {
        stats_.max_fast_deg_s = std::max(
            stats_.max_fast_deg_s, std::abs(degrees(state.qdot_joint_fast)));
        stats_.max_slow_deg_s = std::max(
            stats_.max_slow_deg_s, std::abs(degrees(state.qdot_joint_slow)));
      }
    }
  }

  RunStats& stats_;
  int invalid_consecutive_ = 0;
  int fast35_consecutive_ = 0;
  DurationGate slow_hard_;
  DurationGate slow_warning_;
  DurationGate tracking_hard_;
  DurationGate stall_gate_;
};

class Scheduler100Hz {
 public:
  Scheduler100Hz() : next_(Clock::now()) {}

  void wait_next() {
    next_ += std::chrono::milliseconds(10);
    const auto now = Clock::now();
    if (now >= next_) {
      // The current exchange already overran its slot.  Rebase immediately so
      // a later fast exchange cannot create a burst of catch-up frames.
      next_ = now;
      return;
    }
    std::this_thread::sleep_until(next_);
  }

  void reset() { next_ = Clock::now(); }

 private:
  Clock::time_point next_;
};

std::string csv_row(const std::string& stage, const State& state,
                    const CommandState& command, double q_des_deg,
                    double dq_des_deg_s, const SafetyMonitor& monitor,
                    const RunStats& stats, bool reference_defined,
                    const std::string& status) {
  const double q_actual_deg =
      state.feedback_valid && reference_defined
          ? degrees(state.q_joint)
          : std::numeric_limits<double>::quiet_NaN();
  const double tracking = state.feedback_valid && std::isfinite(q_des_deg)
                              ? q_des_deg - q_actual_deg
                              : std::numeric_limits<double>::quiet_NaN();
  std::ostringstream row;
  row << std::setprecision(17) << stage << ',' << state.sequence << ','
      << state.monotonic_s << ',' << state.send_recv << ',' << state.correct
      << ',' << state.motor_id << ',' << state.feedback_valid << ','
      << state.mode << ',' << state.q_raw << ',' << state.q_raw_unwrapped
      << ',' << q_actual_deg << ',' << degrees(state.qdot_joint_single) << ','
      << degrees(state.qdot_joint_fast) << ','
      << degrees(state.qdot_joint_slow) << ',' << state.velocity_valid << ','
      << state.dq_sdk << ',' << state.tau_sdk << ',' << state.temperature_c
      << ',' << state.merror << ',' << command.mode << ','
      << command.tau_rotor_nm << ',' << command.dq_rotor_rad_s << ','
      << command.q_rotor_rad << ',' << command.kp << ',' << command.kw << ','
      << q_des_deg << ',' << dq_des_deg_s << ',' << tracking << ','
      << monitor.invalid_consecutive() << ',' << monitor.fast35_consecutive()
      << ',' << stats.position_warning << ',' << stats.speed_warning << ','
      << stats.torque_warning << ',' << stats.temperature_warning << ','
      << status << '\n';
  return row.str();
}

State synthetic_state(double time_s, double q_deg, double fast_deg_s,
                      double slow_deg_s, double tau = 0.0,
                      int temperature = 30, int merror = 0,
                      unsigned mode = 1) {
  State state;
  state.monotonic_s = time_s;
  state.send_recv = true;
  state.correct = true;
  state.motor_id = 0;
  state.mode = mode;
  state.feedback_valid = true;
  state.velocity_valid = true;
  state.q_joint = radians(q_deg);
  state.qdot_joint_fast = radians(fast_deg_s);
  state.qdot_joint_slow = radians(slow_deg_s);
  state.tau_sdk = tau;
  state.temperature_c = temperature;
  state.merror = merror;
  return state;
}

template <typename Function>
void expect_trip(Function&& function, const std::string& reason) {
  try {
    function();
  } catch (const SafetyTrip& trip) {
    if (trip.reason == reason) return;
    throw std::runtime_error("SAFETY_SELF_TEST_WRONG_REASON_" + trip.reason);
  }
  throw std::runtime_error("SAFETY_SELF_TEST_MISSED_" + reason);
}

void run_safety_self_tests() {
  require_foc_entry_near_session_zero(0.0);
  require_foc_entry_near_session_zero(0.999999999);
  require_foc_entry_near_session_zero(-0.999999999);
  expect_trip(
      [] { require_foc_entry_near_session_zero(1.0); },
      "FOC_ENTRY_SESSION_ZERO_OUTSIDE_1_DEG");
  expect_trip(
      [] { require_foc_entry_near_session_zero(-1.0); },
      "FOC_ENTRY_SESSION_ZERO_OUTSIDE_1_DEG");
  expect_trip(
      [] {
        require_foc_entry_near_session_zero(
            std::numeric_limits<double>::quiet_NaN());
      },
      "FOC_ENTRY_SESSION_ZERO_OUTSIDE_1_DEG");
  {
    RunStats stats;
    SafetyMonitor monitor(stats);
    monitor.observe(synthetic_state(0.0, 0, 20, 20), 1, true, true,
                    false, 0, 0, 10);
    if (stats.speed_warning) throw std::runtime_error("SPEED_20_NOT_NORMAL");
  }
  {
    RunStats stats;
    SafetyMonitor monitor(stats);
    expect_trip(
        [&] {
          monitor.observe(synthetic_state(0.0, 18, 0, 0), 1, true, true,
                          false, 0, 10, 10);
        },
        "ABSOLUTE_18_DEG_POSITION_GATE");
  }
  {
    RunStats stats;
    SafetyMonitor monitor(stats);
    bool manual = false;
    try {
      monitor.observe(synthetic_state(0.0, 20, 0, 0), 1, true, true,
                      false, 0, 10, 10);
    } catch (const SafetyTrip& trip) {
      manual = trip.reason == "ABSOLUTE_20_DEG_POSITION_EMERGENCY" &&
               trip.manual_power_off;
    }
    if (!manual) throw std::runtime_error("POSITION_20_MANUAL_GATE_FAILED");
  }
  {
    RunStats stats;
    SafetyMonitor monitor(stats);
    monitor.observe(synthetic_state(0.0, 0, 35, 0), 1, true, true, false,
                    0, 0, 10);
    expect_trip(
        [&] {
          monitor.observe(synthetic_state(0.01, 0, 35, 0), 1, true, true,
                          false, 0, 0, 10);
        },
        "FAST_35_DEG_S_TWO_WINDOWS");
  }
  {
    RunStats stats;
    SafetyMonitor monitor(stats);
    expect_trip(
        [&] {
          monitor.observe(synthetic_state(0.0, 0, 40, 0), 1, true, true,
                          false, 0, 0, 10);
        },
        "FAST_40_DEG_S_EMERGENCY");
  }
  {
    RunStats stats;
    SafetyMonitor monitor(stats);
    monitor.observe(synthetic_state(0.0, 0, 0, 30), 1, true, true, false,
                    0, 0, 10);
    monitor.observe(synthetic_state(0.099, 0, 0, 30), 1, true, true, false,
                    0, 0, 10);
    expect_trip(
        [&] {
          monitor.observe(synthetic_state(0.100, 0, 0, 30), 1, true, true,
                          false, 0, 0, 10);
        },
        "SLOW_30_DEG_S_100MS_GATE");
  }
  {
    RunStats stats;
    SafetyMonitor monitor(stats);
    monitor.observe(synthetic_state(0.0, 0, 0, 1), 1, true, true, false,
                    0, 6, 10);
    monitor.observe(synthetic_state(0.299, 0, 0, 1), 1, true, true, false,
                    0, 6, 10);
    expect_trip(
        [&] {
          monitor.observe(synthetic_state(0.300, 0, 0, 1), 1, true, true,
                          false, 0, 6, 10);
        },
        "TRACKING_ERROR_OVER_5_DEG_300MS");
  }
  {
    RunStats stats;
    SafetyMonitor monitor(stats);
    monitor.observe(synthetic_state(0.0, 0, 0, 1), 1, true, true, false,
                    0, 6, 10);
    State invalid;
    invalid.monotonic_s = 0.150;
    monitor.observe(invalid, 1, true, true, false, 0, 6, 10);
    // The invalid 150ms gap is excluded: only 150ms of valid threshold time
    // has accumulated at wall-clock t=0.300.
    monitor.observe(synthetic_state(0.300, 0, 0, 1), 1, true, true, false,
                    0, 6, 10);
    expect_trip(
        [&] {
          monitor.observe(synthetic_state(0.450, 0, 0, 1), 1, true, true,
                          false, 0, 6, 10);
        },
        "TRACKING_ERROR_OVER_5_DEG_300MS");
  }
  {
    RunStats stats;
    SafetyMonitor monitor(stats);
    expect_trip(
        [&] {
          monitor.observe(synthetic_state(0.0, 0, 0, 1), 1, true, true,
                          false, 0, 8, 10);
        },
        "TRACKING_ERROR_8_DEG_GATE");
  }
  {
    RunStats stats;
    SafetyMonitor monitor(stats);
    State invalid;
    for (int i = 0; i < 4; ++i) {
      invalid.monotonic_s = i * kDtS;
      monitor.observe(invalid, 1, true, true, false, 0, 0, 10);
    }
    expect_trip(
        [&] {
          invalid.monotonic_s = 0.04;
          monitor.observe(invalid, 1, true, true, false, 0, 0, 10);
        },
        "FIVE_CONSECUTIVE_INVALID_FRAMES");
  }
  {
    RunStats stats;
    SafetyMonitor monitor(stats);
    expect_trip(
        [&] {
          monitor.observe(synthetic_state(0.0, 1.01, 0, 0), 1, true, true,
                          true, 0, 0, 10);
        },
        "FOC_ENTRY_DISPLACEMENT_OVER_1_DEG");
  }
  {
    RunStats stats;
    SafetyMonitor monitor(stats);
    expect_trip(
        [&] {
          monitor.observe(synthetic_state(0.0, 0, 0, 0, 5.0), 1, true,
                          true, false, 0, 0, 10);
        },
        "TAU_FEEDBACK_5_GATE");
  }
  {
    RunStats stats;
    SafetyMonitor monitor(stats);
    expect_trip(
        [&] {
          monitor.observe(synthetic_state(0.0, 0, 0, 0, 0, 70), 1, true,
                          true, false, 0, 0, 10);
        },
        "TEMPERATURE_70C_GATE");
  }
  {
    RunStats stats;
    SafetyMonitor monitor(stats);
    expect_trip(
        [&] {
          monitor.observe(synthetic_state(0.0, 0, 0, 0, 0, -1), 1, true,
                          true, false, 0, 0, 10);
        },
        "NEGATIVE_TEMPERATURE_INVALID");
  }
  {
    RunStats stats;
    SafetyMonitor monitor(stats);
    State state = synthetic_state(0.0, 0, 0, 0, 0, 70);
    state.feedback_valid = false;  // Another numeric field failed validation.
    expect_trip(
        [&] {
          monitor.observe(state, 1, true, true, false, 0, 0, 10);
        },
        "TEMPERATURE_70C_GATE");
  }
  {
    RunStats stats;
    SafetyMonitor monitor(stats);
    State state = synthetic_state(0.0, 0, 0, 0, 5.0, 30);
    state.feedback_valid = false;
    expect_trip(
        [&] {
          monitor.observe(state, 1, true, true, false, 0, 0, 10);
        },
        "TAU_FEEDBACK_5_GATE");
  }
  {
    RunStats stats;
    SafetyMonitor monitor(stats);
    expect_trip(
        [&] {
          monitor.observe(synthetic_state(0.0, 0, 0, 0, 0, 30, 1), 1,
                          true, true, false, 0, 0, 10);
        },
        "MERROR_NONZERO");
  }
  {
    RunStats stats;
    SafetyMonitor monitor(stats);
    bool manual = false;
    const char* reason =
        monitor.observe_brake(synthetic_state(0.0, 20, 0, 0, 0, 30, 0, 0),
                              true, manual);
    if (!manual || reason == nullptr ||
        std::string(reason) != "ABSOLUTE_20_DEG_POSITION_DURING_BRAKE") {
      throw std::runtime_error("BRAKE_POSITION_20_LATCH_SELF_TEST_FAILED");
    }
  }
}

struct RunOutcome {
  bool pass = false;
  bool brake_confirmed = false;
  bool manual_power_off_required = false;
  bool log_sync_ok = true;
  double active_duration_s = std::numeric_limits<double>::quiet_NaN();
  std::string reason = "NOT_RUN";
  RunStats stats;
};

double median(std::vector<double> values) {
  if (values.empty()) throw std::runtime_error("MEDIAN_EMPTY");
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2;
  return values.size() % 2 == 0
             ? 0.5 * (values[middle - 1] + values[middle])
             : values[middle];
}

class Commissioner {
 public:
  Commissioner(GoM8010Driver& driver, ContinuousLog& log)
      : driver_(driver), log_(log), monitor_(outcome_.stats) {}

  ~Commissioner() {
    if (connected_ && armed_) {
      (void)driver_.emergencyBrake();
      driver_.disconnect();
    }
  }

  RunOutcome run() {
    try {
      driver_.connect();
      connected_ = true;
      armed_ = true;
      baseline_and_session_zero();
      foc_entry_hold();

      run_profile("PROFILE_SESSION_ZERO_TO_P5", entry_anchor_deg_, 5.0,
                  outcome_.stats.positive);
      hold_target("HOLD_P5_300MS", 5.0, 0.300,
                  &outcome_.stats.positive);

      LegMetrics ignored_return_one;
      run_profile("PROFILE_P5_TO_ZERO", 5.0, 0.0,
                  ignored_return_one);
      const State zero_one =
          hold_target("HOLD_ZERO1_200MS", 0.0, 0.200, nullptr);
      outcome_.stats.first_return_error_deg =
          std::abs(degrees(zero_one.q_joint));

      run_profile("PROFILE_ZERO_TO_N5", 0.0, -5.0,
                  outcome_.stats.negative);
      hold_target("HOLD_N5_300MS", -5.0, 0.300,
                  &outcome_.stats.negative);

      LegMetrics ignored_return_two;
      run_profile("PROFILE_N5_TO_ZERO", -5.0, 0.0,
                  ignored_return_two);
      const State zero_two =
          hold_target("HOLD_ZERO2_200MS", 0.0, 0.200, nullptr);
      outcome_.stats.final_return_error_deg =
          std::abs(degrees(zero_two.q_joint));

      const BrakeResult brake = terminal_brake("NORMAL_FINAL_BRAKE");
      outcome_.brake_confirmed = brake.confirmed;
      outcome_.log_sync_ok = brake.log_sync_ok;
      outcome_.stats.final_feedback_mode = brake.final_mode;
      outcome_.active_duration_s =
          std::isfinite(brake.active_duration_at_confirmation_s)
              ? brake.active_duration_at_confirmation_s
              : active_elapsed_s();
      outcome_.manual_power_off_required = brake.manual_power_off_required;
      armed_ = false;
      driver_.disconnect();
      connected_ = false;

      if (brake.hard_fault != nullptr) {
        outcome_.reason = brake.hard_fault;
        return outcome_;
      }
      if (!brake.confirmed) {
        outcome_.reason = "FINAL_BRAKE_CONFIRMATION_FAILED";
        outcome_.manual_power_off_required = true;
        return outcome_;
      }
      if (!brake.log_sync_ok) {
        outcome_.reason = "FINAL_LOG_FSYNC_FAILED";
        return outcome_;
      }
      if (outcome_.active_duration_s > kActiveDeadlineS) {
        outcome_.reason = "ACTIVE_SEQUENCE_15S_DEADLINE";
        return outcome_;
      }
      if (at_least(outcome_.stats.max_abs_angle_deg, 18.0) ||
          outcome_.stats.merror_count != 0 ||
          at_least(outcome_.stats.max_abs_tau, 5.0) ||
          outcome_.stats.max_temperature_c >= 70 ||
          at_least(outcome_.stats.max_tracking_deg, 8.0) ||
          outcome_.stats.max_invalid_consecutive >= 5 ||
          outcome_.stats.final_feedback_mode != driver_.brakeMode()) {
        outcome_.reason = "FINAL_PASS_INVARIANT_FAILED";
        return outcome_;
      }
      outcome_.pass = true;
      outcome_.reason = "SOFTWARE_SEQUENCE_PASS_PENDING_OPERATOR";
      return outcome_;
    } catch (const SafetyTrip& trip) {
      return fail(trip.reason.c_str(), trip.manual_power_off);
    } catch (const std::exception& error) {
      return fail(error.what(), false);
    } catch (...) {
      return fail("UNKNOWN_EXCEPTION", false);
    }
  }

 private:
  struct BrakeResult {
    bool confirmed = false;
    bool log_sync_ok = true;
    bool manual_power_off_required = false;
    int final_mode = -1;
    const char* hard_fault = nullptr;
    double active_duration_at_confirmation_s =
        std::numeric_limits<double>::quiet_NaN();
  };

  bool deadline_started() const noexcept { return active_start_.has_value(); }

  double active_elapsed_s() const {
    if (!active_start_) return std::numeric_limits<double>::quiet_NaN();
    return std::chrono::duration<double>(Clock::now() - *active_start_).count();
  }

  void check_abort_and_deadline() const {
    if (g_stop_requested != 0) throw SafetyTrip("PROCESS_SIGNAL_REQUESTED");
    if (deadline_started() && active_elapsed_s() >= kActiveDeadlineS) {
      throw SafetyTrip("ACTIVE_SEQUENCE_15S_DEADLINE");
    }
  }

  void safe_record(const std::string& stage, const State& state,
                   double q_des_deg, double dq_des_deg_s,
                   const std::string& status) {
    log_.record(csv_row(stage, state, driver_.commandState(), q_des_deg,
                        dq_des_deg_s, monitor_, outcome_.stats,
                        reference_defined_, status));
  }

  void snapshot_trigger(const State& state, double q_des_deg,
                        double dq_des_deg_s) noexcept {
    trigger_state_ = state;
    trigger_command_ = driver_.commandState();
    trigger_q_des_deg_ = q_des_deg;
    trigger_dq_des_deg_s_ = dq_des_deg_s;
    trigger_reference_defined_ = reference_defined_;
    trigger_snapshot_valid_ = true;
  }

  State exchange_brake(const std::string& stage, bool reference_defined) {
    check_abort_and_deadline();
    driver_.brake();
    const State state = driver_.readTelemetry();
    latest_ = state;
    try {
      monitor_.observe(state, driver_.brakeMode(), reference_defined, false,
                       false, 0.0,
                       std::numeric_limits<double>::quiet_NaN(), 0.0);
      safe_record(stage, state, std::numeric_limits<double>::quiet_NaN(), 0.0,
                  state.feedback_valid ? "BRAKE" : "INVALID_FROZEN");
    } catch (const SafetyTrip&) {
      // Never put disk I/O between a detected trip and terminal BRAKE.
      snapshot_trigger(state, std::numeric_limits<double>::quiet_NaN(), 0.0);
      throw;
    }
    scheduler_.wait_next();
    return state;
  }

  State exchange_position(const std::string& stage, double q_des_deg,
                          double dq_des_deg_s, double final_target_deg,
                          bool foc_entry, double entry_anchor_deg) {
    check_abort_and_deadline();
    driver_.commandJointState(radians(q_des_deg), radians(dq_des_deg_s));
    const State state = driver_.readTelemetry();
    latest_ = state;
    try {
      monitor_.observe(state, driver_.focMode(), true, true, foc_entry,
                       entry_anchor_deg, q_des_deg, final_target_deg);
      safe_record(stage, state, q_des_deg, dq_des_deg_s,
                  state.feedback_valid ? "FOC" : "INVALID_PROFILE_FROZEN");
    } catch (const SafetyTrip&) {
      // The trigger is recorded only after the first BRAKE send attempt.
      snapshot_trigger(state, q_des_deg, dq_des_deg_s);
      throw;
    }
    scheduler_.wait_next();
    return state;
  }

  void baseline_and_session_zero() {
    scheduler_.reset();
    const auto baseline_start = Clock::now();
    std::vector<double> raw_samples;
    while (std::chrono::duration<double>(Clock::now() - baseline_start).count() <
               0.500 ||
           raw_samples.size() < 50) {
      const State state = exchange_brake("BRAKE_BASELINE_0P5S", false);
      if (state.feedback_valid) raw_samples.push_back(state.q_raw);
    }
    const double raw_zero = median(raw_samples);
    driver_.setSessionZero(raw_zero);
    reference_defined_ = true;
    try {
      std::ostringstream zero_event;
      zero_event << std::setprecision(17)
                 << "SESSION_LOCAL_ZERO_SET_Q_RAW=" << raw_zero;
      log_.event(zero_event.str());
    } catch (...) {
      throw;
    }

    int valid_warmup = 0;
    while (valid_warmup < 11) {
      const State state = exchange_brake("BRAKE_ESTIMATOR_WARMUP", true);
      if (state.feedback_valid) ++valid_warmup;
    }
    if (!latest_.velocity_valid) {
      // The 11th post-reference feedback must make the 11-point authority
      // available before the first FOC command.
      throw SafetyTrip("VELOCITY_ESTIMATOR_WARMUP_FAILED");
    }
  }

  void foc_entry_hold() {
    const double anchor_deg = degrees(latest_.q_joint);
    // The requested first leg is SESSION_LOCAL_ZERO -> +5 deg.  Refuse to
    // energize FOC if BRAKE drifted outside the zero target band after the
    // median reference and estimator warmup.
    require_foc_entry_near_session_zero(anchor_deg);
    entry_anchor_deg_ = anchor_deg;
    driver_.setPositionControl(kKp, kKd, 0.0);
    active_start_ = Clock::now();
    std::optional<double> first_valid_s;
    while (true) {
      const State state = exchange_position("FOC_ENTRY_FIXED_HOLD_300MS",
                                            anchor_deg, 0.0, anchor_deg, true,
                                            anchor_deg);
      if (!state.feedback_valid) {
        first_valid_s.reset();
        continue;
      }
      if (!first_valid_s) first_valid_s = state.monotonic_s;
      if (state.monotonic_s - *first_valid_s >= 0.300 - 1e-9) break;
    }
  }

  static bool in_target_band(double target_deg, double actual_deg) {
    if (target_deg == 5.0) return actual_deg >= 4.0 && actual_deg <= 6.0;
    if (target_deg == -5.0) return actual_deg >= -6.0 && actual_deg <= -4.0;
    if (target_deg == 0.0) return std::abs(actual_deg) < 1.0;
    throw std::runtime_error("UNKNOWN_TARGET_BAND");
  }

  static void update_leg(LegMetrics& metrics, double target_deg,
                         const State& state) {
    if (!state.feedback_valid) return;
    const double actual_deg = degrees(state.q_joint);
    if (!std::isfinite(metrics.peak_deg)) {
      metrics.peak_deg = actual_deg;
    } else if (target_deg > 0.0) {
      metrics.peak_deg = std::max(metrics.peak_deg, actual_deg);
    } else if (target_deg < 0.0) {
      metrics.peak_deg = std::min(metrics.peak_deg, actual_deg);
    }
    if (state.velocity_valid) {
      metrics.max_slow_speed_deg_s =
          std::max(metrics.max_slow_speed_deg_s,
                   std::abs(degrees(state.qdot_joint_slow)));
    }
  }

  void run_profile(const std::string& stage, double q0_deg, double qf_deg,
                   LegMetrics& metrics) {
    active_leg_metrics_ = qf_deg != 0.0 ? &metrics : nullptr;
    active_leg_target_deg_ = qf_deg;
    const auto profile_start = Clock::now();
    double segment_start_deg = q0_deg;
    State terminal_state = latest_;
    while (segment_start_deg != qf_deg) {
      const j1_final::Profile profile = j1_final::makeProfile(
          segment_start_deg, qf_deg, kMaxVelocityDegS,
          kMaxAccelerationDegS2, kDtS);
      bool restart_from_hold = false;
      for (int tick = 1; tick <= profile.total_intervals; ++tick) {
        const j1_final::Sample command = profile.sample(tick);
        const State state = exchange_position(stage, command.q_deg,
                                              command.dq_deg_s, qf_deg,
                                              false, 0.0);
        if (qf_deg != 0.0) update_leg(metrics, qf_deg, state);
        if (!state.feedback_valid) {
          // A static position with nonzero dq is not a coherent held state.
          // Hold the last transmitted q with dq=0 until feedback returns,
          // then create a new rest-to-rest profile from that exact command.
          while (true) {
            const State recovered = exchange_position(
                stage + "_INVALID_HOLD", command.q_deg, 0.0, qf_deg, false,
                0.0);
            if (qf_deg != 0.0) update_leg(metrics, qf_deg, recovered);
            if (recovered.feedback_valid) {
              terminal_state = recovered;
              break;
            }
          }
          segment_start_deg = command.q_deg;
          restart_from_hold = true;
          break;
        }
        terminal_state = state;
      }
      if (!restart_from_hold) segment_start_deg = qf_deg;
    }

    // Actual position may lag, but the reference has already completed its
    // natural deceleration.  Hold qf/dq0 until the first valid in-band frame.
    if (terminal_state.feedback_valid &&
        in_target_band(qf_deg, degrees(terminal_state.q_joint))) {
      metrics.duration_s =
          std::chrono::duration<double>(Clock::now() - profile_start).count();
      active_leg_metrics_ = nullptr;
      return;
    }
    while (true) {
      const State state =
          exchange_position(stage + "_AWAIT_TARGET", qf_deg, 0.0, qf_deg,
                            false, 0.0);
      if (qf_deg != 0.0) update_leg(metrics, qf_deg, state);
      if (state.feedback_valid &&
          in_target_band(qf_deg, degrees(state.q_joint))) {
        metrics.duration_s =
            std::chrono::duration<double>(Clock::now() - profile_start)
                .count();
        active_leg_metrics_ = nullptr;
        return;
      }
    }
  }

  State hold_target(const std::string& stage, double target_deg,
                    double duration_s, LegMetrics* outward_metrics) {
    active_leg_metrics_ = outward_metrics;
    active_leg_target_deg_ = target_deg;
    std::optional<double> continuously_observed_since;
    while (true) {
      const State state = exchange_position(stage, target_deg, 0.0, target_deg,
                                            false, 0.0);
      if (outward_metrics != nullptr) {
        update_leg(*outward_metrics, target_deg, state);
      }
      if (!state.feedback_valid ||
          !in_target_band(target_deg, degrees(state.q_joint))) {
        continuously_observed_since.reset();
        continue;
      }
      if (!continuously_observed_since) {
        continuously_observed_since = state.monotonic_s;
      }
      if (state.monotonic_s - *continuously_observed_since >=
          duration_s - 1e-9) {
        active_leg_metrics_ = nullptr;
        return state;
      }
    }
  }

  BrakeResult terminal_brake(const char* reason) noexcept {
    BrakeResult result;
    if (!connected_) return result;

    // Setting the command is the first operation.  Logging must never delay
    // selection of BRAKE; the first readTelemetry() below transmits it.
    try {
      driver_.brake();
    } catch (...) {
      return result;
    }
    scheduler_.reset();
    const auto started = Clock::now();
    int consecutive = 0;
    bool first_post_command_synced = false;
    bool event_logged = false;
    bool trigger_logged = false;
    while (std::chrono::duration<double>(Clock::now() - started).count() <
           0.500) {
      try {
        // This is deliberately before every log operation: it is the first
        // call that actually transmits the already-selected BRAKE packet.
        const State state = driver_.readTelemetry();
        const auto feedback_received = Clock::now();
        bool manual_power_off = false;
        const char* brake_fault = monitor_.observe_brake(
            state, reference_defined_, manual_power_off);
        result.manual_power_off_required =
            result.manual_power_off_required || manual_power_off;
        if (result.hard_fault == nullptr && brake_fault != nullptr) {
          result.hard_fault = brake_fault;
        }
        if (state.feedback_valid) {
          result.final_mode = static_cast<int>(state.mode);
        }
        const bool brake_feedback =
            state.feedback_valid &&
            state.mode == static_cast<unsigned>(driver_.brakeMode()) &&
            state.merror == 0;
        consecutive = brake_feedback ? consecutive + 1 : 0;
        try {
          if (trigger_snapshot_valid_ && !trigger_logged) {
            log_.record(csv_row(
                "TRIGGER_FRAME", trigger_state_, trigger_command_,
                trigger_q_des_deg_, trigger_dq_des_deg_s_, monitor_,
                outcome_.stats, trigger_reference_defined_,
                "SAFETY_TRIGGER_PRE_BRAKE"));
            trigger_logged = true;
          }
          if (!event_logged) {
            log_.event(reason);
            event_logged = true;
          }
          if (brake_fault != nullptr) log_.event(brake_fault);
          safe_record("TERMINAL_BRAKE_CONFIRM", state,
                      std::numeric_limits<double>::quiet_NaN(), 0.0,
                      brake_feedback ? "BRAKE_FEEDBACK" : "BRAKE_PENDING");
        } catch (...) {
          result.log_sync_ok = false;
        }
        // Preserve the first post-BRAKE attempt even if it is invalid or still
        // reports FOC.  This occurs only after the BRAKE sendRecv attempt.
        if (!first_post_command_synced) {
          try {
            log_.sync();
            first_post_command_synced = true;
          } catch (...) {
            result.log_sync_ok = false;
          }
        }
        if (consecutive >= 5) {
          result.confirmed = true;
          if (active_start_) {
            result.active_duration_at_confirmation_s =
                std::chrono::duration<double>(feedback_received -
                                              *active_start_)
                    .count();
          }
          break;
        }
      } catch (...) {
        consecutive = 0;
        try {
          if (trigger_snapshot_valid_ && !trigger_logged) {
            log_.record(csv_row(
                "TRIGGER_FRAME", trigger_state_, trigger_command_,
                trigger_q_des_deg_, trigger_dq_des_deg_s_, monitor_,
                outcome_.stats, trigger_reference_defined_,
                "SAFETY_TRIGGER_PRE_BRAKE"));
            trigger_logged = true;
          }
          if (!event_logged) {
            log_.event(reason);
            event_logged = true;
          }
        } catch (...) {
          result.log_sync_ok = false;
        }
        if (!first_post_command_synced) {
          try {
            log_.sync();
            first_post_command_synced = true;
          } catch (...) {
            result.log_sync_ok = false;
          }
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    try {
      log_.sync();
    } catch (...) {
      result.log_sync_ok = false;
    }
    return result;
  }

  RunOutcome fail(const char* reason, bool manual_power_off) {
    // No formatting, allocation, or trigger logging is allowed before this
    // function has attempted the first actual BRAKE frame.
    const bool hardware_was_connected = connected_;
    const BrakeResult brake = terminal_brake(reason);
    armed_ = false;
    if (connected_) {
      driver_.disconnect();
      connected_ = false;
    }
    if (active_leg_metrics_ != nullptr) {
      update_leg(*active_leg_metrics_, active_leg_target_deg_, latest_);
      active_leg_metrics_ = nullptr;
    }
    outcome_.pass = false;
    outcome_.reason = reason;
    outcome_.brake_confirmed = brake.confirmed;
    outcome_.log_sync_ok = brake.log_sync_ok;
    outcome_.stats.final_feedback_mode = brake.final_mode;
    outcome_.manual_power_off_required =
        manual_power_off || brake.manual_power_off_required ||
        (hardware_was_connected && !brake.confirmed);
    outcome_.active_duration_s =
        std::isfinite(brake.active_duration_at_confirmation_s)
            ? brake.active_duration_at_confirmation_s
            : active_elapsed_s();
    return outcome_;
  }

  GoM8010Driver& driver_;
  ContinuousLog& log_;
  RunOutcome outcome_;
  SafetyMonitor monitor_;
  Scheduler100Hz scheduler_;
  State latest_;
  std::optional<Clock::time_point> active_start_;
  bool connected_ = false;
  bool armed_ = false;
  bool reference_defined_ = false;
  double entry_anchor_deg_ = 0.0;
  LegMetrics* active_leg_metrics_ = nullptr;
  double active_leg_target_deg_ = 0.0;
  bool trigger_snapshot_valid_ = false;
  bool trigger_reference_defined_ = false;
  State trigger_state_;
  CommandState trigger_command_;
  double trigger_q_des_deg_ = std::numeric_limits<double>::quiet_NaN();
  double trigger_dq_des_deg_s_ = 0.0;
};

void print_outcome(const RunOutcome& result, const std::string& output) {
  const RunStats& stats = result.stats;
  std::cout << std::fixed << std::setprecision(9)
            << "REVISION=" << kRevision << '\n'
            << "RESULT="
            << (result.pass ? "SOFTWARE_PASS_PENDING_OPERATOR" : "FAIL")
            << '\n'
            << "SOFTWARE_SEQUENCE_PASS=" << (result.pass ? "YES" : "NO")
            << '\n'
            << "REASON=" << result.reason << '\n'
            << "KP_LITERAL=" << kKp << '\n'
            << "KD_LITERAL=" << kKd << '\n'
            << "COMMANDED_MAX_VELOCITY_DEG_S=" << kMaxVelocityDegS << '\n'
            << "COMMANDED_ACCELERATION_DEG_S2="
            << kMaxAccelerationDegS2 << '\n'
            << "P10_PEAK_DEG=" << stats.positive.peak_deg << '\n'
            << "P10_MAX_SLOW_SPEED_DEG_S="
            << stats.positive.max_slow_speed_deg_s << '\n'
            << "P10_DURATION_S=" << stats.positive.duration_s << '\n'
            << "FIRST_RETURN_ERROR_DEG=" << stats.first_return_error_deg
            << '\n'
            << "N10_PEAK_DEG=" << stats.negative.peak_deg << '\n'
            << "N10_MAX_SLOW_SPEED_DEG_S="
            << stats.negative.max_slow_speed_deg_s << '\n'
            << "N10_DURATION_S=" << stats.negative.duration_s << '\n'
            << "FINAL_RETURN_ERROR_DEG=" << stats.final_return_error_deg
            << '\n'
            << "MAX_ABS_J1_ANGLE_DEG=" << stats.max_abs_angle_deg << '\n'
            << "MAX_QDOT_FAST_DEG_S=" << stats.max_fast_deg_s << '\n'
            << "MAX_QDOT_SLOW_DEG_S=" << stats.max_slow_deg_s << '\n'
            << "MAX_TRACKING_ERROR_DEG=" << stats.max_tracking_deg << '\n'
            << "MAX_ABS_TAU_FEEDBACK=" << stats.max_abs_tau << '\n'
            << "TEMPERATURE_MAX_C=" << stats.max_temperature_c << '\n'
            << "MERROR_COUNT=" << stats.merror_count << '\n'
            << "INVALID_FRAME_COUNT=" << stats.invalid_frames << '\n'
            << "MAX_INVALID_CONSECUTIVE=" << stats.max_invalid_consecutive
            << '\n'
            << "POSITION_WARNING=" << stats.position_warning << '\n'
            << "SPEED_WARNING=" << stats.speed_warning << '\n'
            << "TORQUE_WARNING=" << stats.torque_warning << '\n'
            << "TEMPERATURE_WARNING=" << stats.temperature_warning << '\n'
            << "ACTIVE_DURATION_S=" << result.active_duration_s << '\n'
            << "BRAKE_CONFIRMED=" << (result.brake_confirmed ? "YES" : "NO")
            << '\n'
            << "FINAL_FEEDBACK_MODE=" << stats.final_feedback_mode << '\n'
            << "LOG_FSYNC_OK=" << (result.log_sync_ok ? "YES" : "NO")
            << '\n'
            << "MANUAL_POWER_OFF_REQUIRED="
            << (result.manual_power_off_required ? "YES" : "NO") << '\n'
            << "POWER_SUPPLY_LIMITING=OPERATOR_OBSERVATION_REQUIRED\n"
            << "COLLISION_VIBRATION_NOISE=OPERATOR_OBSERVATION_REQUIRED\n"
            << "CSV=" << output << '\n';
}

void run_all_offline_self_tests() {
  go_m8010::runFrozenSdkPacketSelfTest();
  j1_final::runTrajectorySelfTests();
  run_safety_self_tests();
  if (!valid_output_path("/tmp/v15_19d_j1_final_20260815T100000Z.csv") ||
      valid_output_path("/tmp/v15_19d_j1_final_bad/name.csv") ||
      valid_output_path("/home/car/v15_19d_j1_final_bad.csv")) {
    throw std::runtime_error("OUTPUT_PATH_SELF_TEST_FAILED");
  }
}

void install_signal_safety() {
  struct sigaction action {};
  action.sa_handler = signal_handler;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;
  if (sigaction(SIGINT, &action, nullptr) != 0 ||
      sigaction(SIGTERM, &action, nullptr) != 0 ||
      sigaction(SIGHUP, &action, nullptr) != 0) {
    throw std::runtime_error("SIGNAL_HANDLER_INSTALL_FAILED");
  }
  if (prctl(PR_SET_PDEATHSIG, SIGTERM) != 0) {
    throw std::runtime_error("PDEATHSIG_INSTALL_FAILED");
  }
  if (getppid() == 1) throw std::runtime_error("PARENT_ALREADY_GONE");
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);

    // These two branches occur before ProcessLock, log, driver construction,
    // SerialPort construction, or any path under /dev is referenced by I/O.
    if (options.self_test) {
      run_all_offline_self_tests();
      std::cout << "OFFLINE_SELF_TEST=PASS\n"
                << "SERIAL_DEVICE_OPENED=NO\n";
      return 0;
    }
    if (!options.execute) {
      const j1_final::Profile profile = j1_final::makeProfile(
          0.0, 5.0, kMaxVelocityDegS, kMaxAccelerationDegS2, kDtS);
      std::cout << std::fixed << std::setprecision(9)
                << "MODE=DRY_RUN\nSERIAL_DEVICE_OPENED=NO\n"
                << "CONFIRMATION_REQUIRED=" << kConfirmation << '\n'
                << "SEQUENCE=0,+5,0,-5,0\n"
                << "PROFILE_INTERVALS_PER_5_DEG="
                << profile.total_intervals << '\n'
                << "REALIZED_PEAK_VELOCITY_DEG_S="
                << profile.peak_velocity_deg_s << '\n'
                << "REALIZED_ACCELERATION_DEG_S2="
                << profile.acceleration_deg_s2 << '\n';
      return 0;
    }

    // The authorized execute path repeats every pure preflight before any
    // lock, log, driver object, SerialPort, or device operation is possible.
    run_all_offline_self_tests();
    install_signal_safety();
    ProcessLock lock;
    ContinuousLog log(options.output);
    go_m8010::DriverConfig config;
    config.port = kPort;
    config.motor_id = 0;
    config.gear_ratio = kGearRatio;
    config.direction_sign = 1;
    config.temperature_limit_c = 70.0;
    config.joint_envelope_rad = radians(20.0);
    GoM8010Driver driver(config);
    Commissioner commissioner(driver, log);
    const RunOutcome outcome = commissioner.run();
    print_outcome(outcome, options.output);
    return outcome.pass ? 0 : 2;
  } catch (const std::exception& error) {
    std::cerr << "PRE_EXECUTION_FAILURE=" << error.what() << '\n';
    return 1;
  } catch (...) {
    std::cerr << "PRE_EXECUTION_FAILURE=UNKNOWN\n";
    return 1;
  }
}
