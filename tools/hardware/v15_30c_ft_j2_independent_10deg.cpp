#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

namespace {

using Clock = std::chrono::steady_clock;

constexpr char kPort[] =
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if01-port0";
constexpr char kLockPath[] = "/tmp/v15_23d_ft_j2_channel1.lock";
constexpr char kOutputA[] =
    "/tmp/v15_30c_ft_j2a_independent_10deg_kp10_kd10_6s.csv";
constexpr char kOutputB[] =
    "/tmp/v15_30c_ft_j2b_independent_10deg_kp10_kd10_6s.csv";
constexpr double kPi = 3.14159265358979323846;
constexpr double kGear = 6.3299999237060547;
constexpr double kKp = 1.00;
constexpr double kKd = 0.10;
constexpr double kTauFf = 0.0;
constexpr double kRateHz = 100.0;
constexpr double kPeriodS = 1.0 / kRateHz;
constexpr double kMoveDurationS = 6.0;
constexpr double kEndpointHoldS = 0.70;
constexpr double kInitialHoldS = 1.0;
constexpr double kFeedbackEnvelopeDeg = 12.0;
constexpr double kOtherMotorEnvelopeDeg = 10.0;
constexpr double kRotorCommandPdHardNm = 0.60;
constexpr double kActiveRotorTorqueFeedbackHardNm = 0.60;
constexpr double kOtherRotorTorqueFeedbackHardNm = 0.35;
constexpr double kPrecisionToleranceDeg = 1.0;
constexpr double kFastVelocityHardDegS = 40.0;
constexpr double kSlowVelocityHardDegS = 30.0;
constexpr double kCycleDeadlineMs = 20.0;
constexpr int kTemperatureLimitC = 60;
constexpr int kBrakeMode = 0;
constexpr int kFocMode = 1;

volatile std::sig_atomic_t g_stop_requested = 0;

void handle_signal(int) { g_stop_requested = 1; }

template <typename T>
void zero_object(T& value) {
  static_assert(std::is_trivially_copyable<T>::value, "frozen SDK ABI");
  std::memset(static_cast<void*>(&value), 0, sizeof(value));
}

double rad_to_deg(double value) { return value * 180.0 / kPi; }
double deg_to_rad(double value) { return value * kPi / 180.0; }

double wrap_pi(double value) {
  while (value > kPi) value -= 2.0 * kPi;
  while (value < -kPi) value += 2.0 * kPi;
  return value;
}

double median(std::vector<double> values) {
  if (values.empty()) throw std::runtime_error("MEDIAN_EMPTY");
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2U;
  return values.size() % 2U == 0U
      ? (values[middle - 1U] + values[middle]) / 2.0
      : values[middle];
}

MotorCmd make_command(int id, int mode, double q, double dq, double kp,
                      double kd, double tau) {
  MotorCmd command;
  zero_object(command);
  command.motorType = MotorType::GO_M8010_6;
  command.id = static_cast<unsigned short>(id);
  command.mode = static_cast<unsigned short>(mode);
  command.q = static_cast<float>(q);
  command.dq = static_cast<float>(dq);
  command.kp = static_cast<float>(kp);
  command.kd = static_cast<float>(kd);
  command.tau = static_cast<float>(tau);
  command.Res.u32 = 0;
  command.modify_data(&command);
  return command;
}

MotorCmd brake_command(int id) {
  return make_command(id, kBrakeMode, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void initialize_feedback(MotorData& data) {
  zero_object(data);
  data.motorType = MotorType::GO_M8010_6;
  data.motor_id = 0xffU;
  data.mode = 0xffU;
  data.temp = std::numeric_limits<int>::min();
  data.merror = -1;
  data.q = std::numeric_limits<float>::quiet_NaN();
  data.dq = std::numeric_limits<float>::quiet_NaN();
  data.tau = std::numeric_limits<float>::quiet_NaN();
  data.correct = false;
}

struct Feedback {
  bool send_recv = false;
  bool valid = false;
  MotorData data;
};

Feedback transact(SerialPort& serial, MotorCmd& command, int expected_id,
                  int expected_mode) {
  Feedback feedback;
  initialize_feedback(feedback.data);
  try {
    feedback.send_recv = serial.sendRecv(&command, &feedback.data);
  } catch (...) {
    feedback.send_recv = false;
  }
  feedback.valid = feedback.send_recv && feedback.data.correct &&
      static_cast<int>(feedback.data.motor_id) == expected_id &&
      static_cast<int>(feedback.data.mode) == expected_mode &&
      feedback.data.merror == 0 && feedback.data.temp >= 0 &&
      feedback.data.temp < kTemperatureLimitC &&
      std::isfinite(feedback.data.q) && std::isfinite(feedback.data.dq) &&
      std::isfinite(feedback.data.tau);
  return feedback;
}

class ProcessLock {
 public:
  ProcessLock() {
    fd_ = ::open(kLockPath, O_RDWR | O_CREAT | O_CLOEXEC, 0600);
    if (fd_ < 0 || ::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      throw std::runtime_error("J2_CHANNEL1_LOCK_FAILED");
    }
  }
  ~ProcessLock() {
    if (fd_ >= 0) {
      (void)::flock(fd_, LOCK_UN);
      (void)::close(fd_);
    }
  }

 private:
  int fd_ = -1;
};

struct Cli {
  std::string motor;
  std::string output;
  std::string mechanical_gate;
  std::string clearance_gate;
  std::string power_gate;
  std::string current_limit_gate;
  std::string execute_gate;
  bool self_test = false;
};

Cli parse_cli(int argc, char** argv) {
  Cli cli;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&]() {
      if (++i >= argc) throw std::runtime_error("CLI_VALUE_MISSING");
      return std::string(argv[i]);
    };
    if (arg == "--self-test") cli.self_test = true;
    else if (arg == "--motor") cli.motor = value();
    else if (arg == "--output") cli.output = value();
    else if (arg == "--mechanically-independent-gate")
      cli.mechanical_gate = value();
    else if (arg == "--clearance-gate") cli.clearance_gate = value();
    else if (arg == "--power-gate") cli.power_gate = value();
    else if (arg == "--current-limit-gate") cli.current_limit_gate = value();
    else if (arg == "--execute-gate") cli.execute_gate = value();
    else throw std::runtime_error("CLI_OPTION_NOT_ALLOWED");
  }
  if (cli.self_test) return cli;
  if (cli.motor != "J2A" && cli.motor != "J2B")
    throw std::runtime_error("MOTOR_NOT_ALLOWED");
  const std::string expected_output = cli.motor == "J2A" ? kOutputA : kOutputB;
  if (cli.output != expected_output)
    throw std::runtime_error("OUTPUT_PATH_NOT_ALLOWED");
  if (cli.mechanical_gate != "J2A_J2B_OUTPUTS_MECHANICALLY_INDEPENDENT=YES")
    throw std::runtime_error("MECHANICAL_GATE_MISSING");
  if (cli.clearance_gate != "BOTH_OUTPUTS_PLUS_MINUS_12DEG_CLEAR=YES")
    throw std::runtime_error("CLEARANCE_GATE_MISSING");
  if (cli.power_gate != "J2_24V_STABLE=YES")
    throw std::runtime_error("POWER_GATE_MISSING");
  if (cli.current_limit_gate != "PSU_CURRENT_LIMIT_3A=YES" &&
      cli.current_limit_gate !=
          "PSU_CAPACITY_CONFIRMED_CURRENT_UNMEASURED=YES")
    throw std::runtime_error("CURRENT_LIMIT_GATE_MISSING");
  const std::string expected_execute = cli.motor + "_INDEPENDENT_10DEG_AUTHORIZED=YES";
  if (cli.execute_gate != expected_execute)
    throw std::runtime_error("EXECUTE_GATE_MISSING");
  return cli;
}

struct PairFeedback {
  Feedback active;
  Feedback other;
  double q_active_deg = 0.0;
  double q_other_deg = 0.0;
  double velocity_fast_deg_s = 0.0;
  double velocity_slow_deg_s = 0.0;
  double logical_tau_nm = 0.0;
  double cycle_ms = 0.0;
};

std::ofstream open_output_exclusive(const std::string& path) {
  const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC,
                        0600);
  if (fd < 0) throw std::runtime_error("OUTPUT_PATH_ALREADY_EXISTS_OR_CREATE_FAILED");
  if (::close(fd) != 0) {
    (void)::unlink(path.c_str());
    throw std::runtime_error("OUTPUT_RESERVATION_CLOSE_FAILED");
  }
  std::ofstream stream(path, std::ios::out | std::ios::app);
  if (!stream) {
    (void)::unlink(path.c_str());
    throw std::runtime_error("OUTPUT_OPEN_FAILED");
  }
  return stream;
}

class Runner {
 public:
  explicit Runner(const Cli& cli)
      : cli_(cli),
        active_id_(cli.motor == "J2A" ? 0 : 1),
        other_id_(cli.motor == "J2A" ? 1 : 0),
        active_sign_(cli.motor == "J2A" ? -1.0 : 1.0),
        other_sign_(cli.motor == "J2A" ? 1.0 : -1.0),
        lock_(),
        serial_(kPort, 16, 4000000, 20000, BlockYN::NO,
                bytesize_t::eightbits, parity_t::parity_none,
                stopbits_t::stopbits_one, flowcontrol_t::flowcontrol_none),
        csv_(open_output_exclusive(cli.output)),
        origin_(Clock::now()) {
    csv_ << "tick,timestamp_s,phase,active_motor,active_id,other_id,q_ref_deg,"
            "dq_ref_deg_s,q_active_deg,q_other_deg,velocity_fast_deg_s,"
            "velocity_slow_deg_s,active_q_raw,other_q_raw,active_dq_raw,"
            "other_dq_raw,active_tau_raw,other_tau_raw,logical_tau_nm,"
            "active_temp_c,other_temp_c,active_merror,other_merror,"
            "active_mode,other_mode,active_valid,other_valid,cycle_ms\n";
  }

  ~Runner() { safe_brake_noexcept(); }

  int run() {
    try {
      capture_brake_reference();
      const auto initial = hold("INITIAL_FOC_HOLD", 0.0, kInitialHoldS);
      const double initial_final = tail_median(initial, 50);
      const double initial_max = max_abs(initial);
      if (initial_max > 1.0 || std::abs(initial_final) > 0.5)
        throw std::runtime_error("INITIAL_HOLD_FAILED");

      profile("PLUS_10_OUTBOUND", 0.0, 10.0);
      const auto plus = hold("PLUS_10_ENDPOINT", 10.0, kEndpointHoldS);
      const double plus_actual = tail_median(plus, 50);
      profile("PLUS_10_RETURN", 10.0, 0.0);
      const auto center1 = hold("FIRST_CENTER", 0.0, kEndpointHoldS);
      const double center1_actual = tail_median(center1, 50);

      profile("MINUS_10_OUTBOUND", 0.0, -10.0);
      const auto minus = hold("MINUS_10_ENDPOINT", -10.0, kEndpointHoldS);
      const double minus_actual = tail_median(minus, 50);
      profile("MINUS_10_RETURN", -10.0, 0.0);
      const auto center2 = hold("FINAL_CENTER", 0.0, kEndpointHoldS);
      const double center2_actual = tail_median(center2, 50);

      const bool final_brake = brake_both(20, true);
      const bool visible = plus_actual >= 5.0 && minus_actual <= -5.0;
      const bool precise =
          std::abs(plus_actual - 10.0) <= kPrecisionToleranceDeg &&
          std::abs(center1_actual) <= kPrecisionToleranceDeg &&
          std::abs(minus_actual + 10.0) <= kPrecisionToleranceDeg &&
          std::abs(center2_actual) <= kPrecisionToleranceDeg;
      std::cout << std::setprecision(17)
                << "RUN=V15_30C_FT_J2_INDEPENDENT_10DEG_KP10_KD10_6S\n"
                << "ACTIVE_MOTOR=" << cli_.motor << '\n'
                << "ROUTE=0_TO_PLUS10_TO_0_TO_MINUS10_TO_0\n"
                << "KP=" << kKp << "\nKD=" << kKd << "\nTFF=" << kTauFf
                << "\nMOVE_DURATION_S=" << kMoveDurationS << '\n'
                << "PLUS_10_ACTUAL_DEG=" << plus_actual << '\n'
                << "FIRST_CENTER_ACTUAL_DEG=" << center1_actual << '\n'
                << "MINUS_10_ACTUAL_DEG=" << minus_actual << '\n'
                << "FINAL_CENTER_ACTUAL_DEG=" << center2_actual << '\n'
                << "MAX_ABS_LOGICAL_TAU_FEEDBACK_NM=" << max_tau_nm_ << '\n'
                << "MAX_ABS_ACTIVE_ROTOR_TAU_FEEDBACK_NM="
                << max_active_rotor_tau_nm_ << '\n'
                << "MAX_ABS_OTHER_ROTOR_TAU_FEEDBACK_NM="
                << max_other_rotor_tau_nm_ << '\n'
                << "MAX_ABS_OTHER_DRIFT_DEG=" << max_other_drift_deg_ << '\n'
                << "MAX_ABS_FAST_VELOCITY_DEG_S=" << max_fast_velocity_deg_s_
                << "\nMAX_ABS_SLOW_VELOCITY_DEG_S="
                << max_slow_velocity_deg_s_ << '\n'
                << "MAX_ACTIVE_TEMP_C=" << max_active_temp_c_ << '\n'
                << "MAX_OTHER_TEMP_C=" << max_other_temp_c_ << '\n'
                << "PSU_GATE=" << cli_.current_limit_gate << '\n'
                << "PSU_CURRENT_MEASUREMENT="
                << (cli_.current_limit_gate == "PSU_CURRENT_LIMIT_3A=YES"
                        ? "LIMIT_SET_3A" : "UNAVAILABLE") << '\n'
                << "FINAL_DUAL_BRAKE=" << (final_brake ? "PASS" : "FAIL")
                << "\nVISIBLE_10DEG_RESULT=" << (visible ? "PASS" : "FAIL")
                << "\nPRECISION_TOLERANCE_DEG=" << kPrecisionToleranceDeg
                << "\nPRECISION_RESULT=" << (precise ? "PASS" : "FAIL")
                << "\nZERO_MODIFIED=NO\nID_MODIFIED=NO\nRID_WRITTEN=NO\n";
      return visible && precise && final_brake ? 0 : 2;
    } catch (...) {
      (void)brake_both(20, false);
      throw;
    }
  }

 private:
  void check_stop() const {
    if (g_stop_requested != 0) throw std::runtime_error("STOP_REQUESTED");
  }

  PairFeedback cycle(const std::string& phase, double q_ref_deg,
                     double dq_ref_deg_s, bool foc) {
    check_stop();
    const auto started = Clock::now();
    MotorCmd other_command = brake_command(other_id_);
    Feedback other = transact(serial_, other_command, other_id_, kBrakeMode);
    if (!other.valid)
      throw std::runtime_error("OTHER_BRAKE_FEEDBACK_INVALID_PRE_ENABLE");
    const double q_other_pre_deg = rad_to_deg(
        other_sign_ * wrap_pi(static_cast<double>(other.data.q) -
                              other_center_raw_) /
        kGear);
    if (std::abs(q_other_pre_deg) > kOtherMotorEnvelopeDeg)
      throw std::runtime_error("OTHER_BRAKED_MOTOR_MOVED_PRE_ENABLE");
    if (std::abs(static_cast<double>(other.data.tau)) >
        kOtherRotorTorqueFeedbackHardNm)
      throw std::runtime_error("OTHER_TORQUE_FEEDBACK_LIMIT_PRE_ENABLE");
    check_stop();
    const double q_command = foc
        ? active_center_raw_ + active_sign_ * kGear * deg_to_rad(q_ref_deg)
        : 0.0;
    const double dq_command = foc
        ? active_sign_ * kGear * deg_to_rad(dq_ref_deg_s)
        : 0.0;
    if (foc && have_previous_) {
      const double predicted_pd_nm = std::abs(
          kKp * kGear * deg_to_rad(q_ref_deg - previous_q_active_deg_) +
          kKd * kGear * deg_to_rad(
              dq_ref_deg_s - velocity_slow_deg_s_));
      if (predicted_pd_nm > kRotorCommandPdHardNm)
        throw std::runtime_error("PREDICTED_PD_TORQUE_LIMIT_EXCEEDED");
    }
    MotorCmd active_command = foc
        ? make_command(active_id_, kFocMode, q_command, dq_command,
                       kKp, kKd, kTauFf)
        : brake_command(active_id_);
    Feedback active = transact(serial_, active_command, active_id_,
                               foc ? kFocMode : kBrakeMode);
    const auto finished = Clock::now();
    const double cycle_ms =
        std::chrono::duration<double, std::milli>(finished - started).count();

    if (!active.valid) throw std::runtime_error("ACTIVE_FEEDBACK_INVALID");
    if (cycle_ms > kCycleDeadlineMs)
      throw std::runtime_error("CONTROL_CYCLE_DEADLINE_EXCEEDED");

    PairFeedback pair;
    pair.active = active;
    pair.other = other;
    pair.cycle_ms = cycle_ms;
    pair.q_active_deg = rad_to_deg(active_sign_ *
        wrap_pi(static_cast<double>(active.data.q) - active_center_raw_) / kGear);
    pair.q_other_deg = rad_to_deg(other_sign_ *
        wrap_pi(static_cast<double>(other.data.q) - other_center_raw_) / kGear);
    pair.logical_tau_nm = active_sign_ * kGear * active.data.tau;

    const double timestamp_s =
        std::chrono::duration<double>(finished - origin_).count();
    if (have_previous_) {
      const double dt = timestamp_s - previous_timestamp_s_;
      if (!(dt > 0.0 && dt < 0.10))
        throw std::runtime_error("VELOCITY_SAMPLE_INTERVAL_INVALID");
      const double raw_velocity =
          (pair.q_active_deg - previous_q_active_deg_) / dt;
      velocity_fast_deg_s_ = 0.35 * raw_velocity + 0.65 * velocity_fast_deg_s_;
      velocity_slow_deg_s_ = 0.10 * raw_velocity + 0.90 * velocity_slow_deg_s_;
    }
    previous_timestamp_s_ = timestamp_s;
    previous_q_active_deg_ = pair.q_active_deg;
    have_previous_ = true;
    pair.velocity_fast_deg_s = velocity_fast_deg_s_;
    pair.velocity_slow_deg_s = velocity_slow_deg_s_;

    max_tau_nm_ = std::max(max_tau_nm_, std::abs(pair.logical_tau_nm));
    max_active_rotor_tau_nm_ =
        std::max(max_active_rotor_tau_nm_,
                 std::abs(static_cast<double>(active.data.tau)));
    max_other_rotor_tau_nm_ =
        std::max(max_other_rotor_tau_nm_,
                 std::abs(static_cast<double>(other.data.tau)));
    max_other_drift_deg_ =
        std::max(max_other_drift_deg_, std::abs(pair.q_other_deg));
    max_fast_velocity_deg_s_ = std::max(max_fast_velocity_deg_s_,
                                        std::abs(pair.velocity_fast_deg_s));
    max_slow_velocity_deg_s_ = std::max(max_slow_velocity_deg_s_,
                                        std::abs(pair.velocity_slow_deg_s));
    max_active_temp_c_ = std::max(max_active_temp_c_, active.data.temp);
    max_other_temp_c_ = std::max(max_other_temp_c_, other.data.temp);

    if (foc && std::abs(pair.q_active_deg) > kFeedbackEnvelopeDeg)
      throw std::runtime_error("ACTIVE_FEEDBACK_ENVELOPE_EXCEEDED");
    if (std::abs(pair.q_other_deg) > kOtherMotorEnvelopeDeg)
      throw std::runtime_error("OTHER_BRAKED_MOTOR_MOVED");
    if (std::abs(active.data.tau) > kActiveRotorTorqueFeedbackHardNm)
      throw std::runtime_error("ACTIVE_TORQUE_FEEDBACK_LIMIT_EXCEEDED");
    if (std::abs(other.data.tau) > kOtherRotorTorqueFeedbackHardNm)
      throw std::runtime_error("OTHER_TORQUE_FEEDBACK_LIMIT_EXCEEDED");
    if (std::abs(pair.velocity_fast_deg_s) > kFastVelocityHardDegS ||
        std::abs(pair.velocity_slow_deg_s) > kSlowVelocityHardDegS)
      throw std::runtime_error("OUTPUT_VELOCITY_LIMIT_EXCEEDED");

    csv_ << tick_++ << ',' << std::setprecision(17) << timestamp_s << ','
         << phase << ',' << cli_.motor << ',' << active_id_ << ',' << other_id_
         << ',' << q_ref_deg << ',' << dq_ref_deg_s << ',' << pair.q_active_deg
         << ',' << pair.q_other_deg << ',' << pair.velocity_fast_deg_s << ','
         << pair.velocity_slow_deg_s << ',' << active.data.q << ','
         << other.data.q << ',' << active.data.dq << ',' << other.data.dq << ','
         << active.data.tau << ',' << other.data.tau << ','
         << pair.logical_tau_nm << ',' << active.data.temp << ','
         << other.data.temp << ',' << active.data.merror << ','
         << other.data.merror << ',' << static_cast<int>(active.data.mode) << ','
         << static_cast<int>(other.data.mode) << ',' << active.valid << ','
         << other.valid << ',' << cycle_ms << '\n';
    csv_.flush();
    return pair;
  }

  void capture_brake_reference() {
    std::vector<double> active_raw;
    std::vector<double> other_raw;
    active_raw.reserve(50U);
    other_raw.reserve(50U);
    for (int frame = 0; frame < 50; ++frame) {
      check_stop();
      MotorCmd active_command = brake_command(active_id_);
      MotorCmd other_command = brake_command(other_id_);
      Feedback active = transact(serial_, active_command, active_id_, kBrakeMode);
      Feedback other = transact(serial_, other_command, other_id_, kBrakeMode);
      if (!active.valid || !other.valid)
        throw std::runtime_error("BRAKE_CAPTURE_INVALID");
      if (std::abs(static_cast<double>(active.data.tau)) >
              kOtherRotorTorqueFeedbackHardNm ||
          std::abs(static_cast<double>(other.data.tau)) >
              kOtherRotorTorqueFeedbackHardNm)
        throw std::runtime_error("BRAKE_CAPTURE_TORQUE_LIMIT_EXCEEDED");
      active_raw.push_back(active.data.q);
      other_raw.push_back(other.data.q);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    active_center_raw_ = median(active_raw);
    other_center_raw_ = median(other_raw);
    previous_timestamp_s_ =
        std::chrono::duration<double>(Clock::now() - origin_).count();
    previous_q_active_deg_ = 0.0;
    velocity_fast_deg_s_ = 0.0;
    velocity_slow_deg_s_ = 0.0;
    have_previous_ = true;
  }

  std::vector<double> hold(const std::string& phase, double q_ref_deg,
                           double duration_s) {
    const int frames = static_cast<int>(std::ceil(duration_s * kRateHz));
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(frames));
    auto next = Clock::now();
    for (int frame = 0; frame < frames; ++frame) {
      next += std::chrono::milliseconds(10);
      values.push_back(cycle(phase, q_ref_deg, 0.0, true).q_active_deg);
      std::this_thread::sleep_until(next);
    }
    return values;
  }

  void profile(const std::string& phase, double start_deg, double end_deg) {
    const int frames = static_cast<int>(std::ceil(kMoveDurationS * kRateHz));
    auto next = Clock::now();
    for (int frame = 1; frame <= frames; ++frame) {
      next += std::chrono::milliseconds(10);
      const double s = static_cast<double>(frame) / static_cast<double>(frames);
      const double s2 = s * s;
      const double s3 = s2 * s;
      const double s4 = s3 * s;
      const double s5 = s4 * s;
      const double blend = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;
      const double blend_dot =
          (30.0 * s2 - 60.0 * s3 + 30.0 * s4) / kMoveDurationS;
      const double delta = end_deg - start_deg;
      const double q_ref = start_deg + delta * blend;
      const double dq_ref = delta * blend_dot;
      (void)cycle(phase, q_ref, dq_ref, true);
      std::this_thread::sleep_until(next);
    }
  }

  bool brake_both(int frames, bool log_rows) noexcept {
    int valid_pairs = 0;
    for (int frame = 0; frame < frames; ++frame) {
      try {
        MotorCmd active_command = brake_command(active_id_);
        MotorCmd other_command = brake_command(other_id_);
        Feedback active = transact(serial_, active_command, active_id_, kBrakeMode);
        Feedback other = transact(serial_, other_command, other_id_, kBrakeMode);
        if (active.valid && other.valid) ++valid_pairs;
        if (log_rows && csv_) {
          const double timestamp_s =
              std::chrono::duration<double>(Clock::now() - origin_).count();
          csv_ << tick_++ << ',' << std::setprecision(17) << timestamp_s
               << ",FINAL_DUAL_BRAKE," << cli_.motor << ',' << active_id_ << ','
               << other_id_ << ",0,0,0,0,0,0," << active.data.q << ','
               << other.data.q << ',' << active.data.dq << ',' << other.data.dq
               << ',' << active.data.tau << ',' << other.data.tau
               << ",0," << active.data.temp << ',' << other.data.temp << ','
               << active.data.merror << ',' << other.data.merror << ','
               << static_cast<int>(active.data.mode) << ','
               << static_cast<int>(other.data.mode) << ',' << active.valid << ','
               << other.valid << ",0\n";
          csv_.flush();
        }
      } catch (...) {
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return valid_pairs == frames;
  }

  void safe_brake_noexcept() noexcept {
    if (!brake_attempted_) {
      brake_attempted_ = true;
      (void)brake_both(5, false);
    }
  }

  static double tail_median(const std::vector<double>& values,
                            std::size_t count) {
    if (values.size() < count) throw std::runtime_error("TAIL_TOO_SHORT");
    return median(std::vector<double>(values.end() -
        static_cast<std::ptrdiff_t>(count), values.end()));
  }

  static double max_abs(const std::vector<double>& values) {
    double result = 0.0;
    for (double value : values) result = std::max(result, std::abs(value));
    return result;
  }

  const Cli& cli_;
  const int active_id_;
  const int other_id_;
  const double active_sign_;
  const double other_sign_;
  ProcessLock lock_;
  SerialPort serial_;
  std::ofstream csv_;
  Clock::time_point origin_;
  std::uint64_t tick_ = 0;
  double active_center_raw_ = 0.0;
  double other_center_raw_ = 0.0;
  double previous_timestamp_s_ = 0.0;
  double previous_q_active_deg_ = 0.0;
  double velocity_fast_deg_s_ = 0.0;
  double velocity_slow_deg_s_ = 0.0;
  bool have_previous_ = false;
  bool brake_attempted_ = false;
  double max_tau_nm_ = 0.0;
  double max_active_rotor_tau_nm_ = 0.0;
  double max_other_rotor_tau_nm_ = 0.0;
  double max_other_drift_deg_ = 0.0;
  double max_fast_velocity_deg_s_ = 0.0;
  double max_slow_velocity_deg_s_ = 0.0;
  int max_active_temp_c_ = 0;
  int max_other_temp_c_ = 0;
};

void self_test() {
  if (queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE) != kBrakeMode ||
      queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC) != kFocMode ||
      std::abs(queryGearRatio(MotorType::GO_M8010_6) - kGear) > 1e-6) {
    throw std::runtime_error("SDK_AUTHORITY_MISMATCH");
  }
  for (int id : {0, 1}) {
    (void)brake_command(id);
    (void)make_command(id, kFocMode, 1.0, 0.0, kKp, kKd, kTauFf);
  }
  std::cout << "SELF_TEST=PASS\nSERIAL_PORT_CONSTRUCTED=NO\n"
             << "ACTIVE_MOTION_USED=NO\nROUTE=0_TO_PLUS10_TO_0_TO_MINUS10_TO_0\n"
            << "KP=" << kKp << "\nKD=" << kKd
            << "\nTFF=" << kTauFf << "\nMOVE_DURATION_S="
            << kMoveDurationS << '\n'
            << "OTHER_MOTOR_COMMAND=CONTINUOUS_BRAKE\n"
            << "OTHER_MOTOR_DRIFT_HARD_DEG=" << kOtherMotorEnvelopeDeg << '\n'
            << "ROTOR_PREDICTED_PD_TORQUE_HARD_NM="
            << kRotorCommandPdHardNm << '\n'
            << "ACTIVE_ROTOR_FEEDBACK_TORQUE_HARD_NM="
            << kActiveRotorTorqueFeedbackHardNm << '\n'
            << "OTHER_ROTOR_FEEDBACK_TORQUE_HARD_NM="
            << kOtherRotorTorqueFeedbackHardNm << '\n'
            << "FIRST_INVALID_OR_MERROR_ABORT=YES\n"
            << "SIGNAL_CHECK_EVERY_ACTIVE_FRAME=YES\n"
            << "ZERO_WRITE_PATH=NO\nID_WRITE_PATH=NO\nRID_WRITE_PATH=NO\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
    const Cli cli = parse_cli(argc, argv);
    if (cli.self_test) {
      self_test();
      return 0;
    }
    if (::access(kPort, R_OK | W_OK) != 0)
      throw std::runtime_error("J2_PORT_NOT_ACCESSIBLE");
    Runner runner(cli);
    return runner.run();
  } catch (const std::exception& error) {
    std::cerr << "V15_30C_FT_J2_INDEPENDENT_10DEG_RESULT=BLOCKED\nREASON="
              << error.what() << '\n';
    return 2;
  }
}
