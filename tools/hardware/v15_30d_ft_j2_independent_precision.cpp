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
    "/tmp/v15_30d_r2_ft_j2a_independent_precision_i025.csv";
constexpr char kOutputB[] =
    "/tmp/v15_30d_r2_ft_j2b_independent_precision_i025.csv";
constexpr double kPi = 3.14159265358979323846;
constexpr double kGear = 6.3299999237060547;
constexpr double kKp = 1.00;
constexpr double kKd = 0.10;
constexpr double kRateHz = 100.0;
constexpr double kPeriodS = 1.0 / kRateHz;
constexpr double kMoveDurationS = 10.0;
constexpr double kEndpointHoldS = 8.0;
constexpr double kInitialHoldS = 2.5;
constexpr double kKpRampStart = 0.20;
constexpr double kKpRampDurationS = 2.0;
constexpr double kFeedbackEnvelopeDeg = 12.0;
constexpr double kOtherMotorEnvelopeDeg = 10.0;
constexpr double kRotorPredictedWorkNm = 0.50;
constexpr double kRotorPredictedHardNm = 153.0 / 256.0;
constexpr double kActiveRotorTorqueFeedbackHardNm = 154.0 / 256.0;
constexpr double kOtherRotorTorqueFeedbackHardNm = 0.35;
constexpr double kIntegralRequestedTorqueHardNm = 0.25;
constexpr double kIntegralWireQuantumNm = 1.0 / 256.0;
constexpr int kIntegralWireQ8Hard = 64;
constexpr double kIntegralTorqueHardNm =
    static_cast<double>(kIntegralWireQ8Hard) * kIntegralWireQuantumNm;
constexpr double kIntegralKiPerRotorRadS = 0.20;
constexpr double kIntegralAccumulatorRateHardNmS = 0.03;
constexpr int kIntegralWireMinFramesBetweenSteps = 14;
constexpr double kIntegralWireAverageSlewCeilingNmS =
    kIntegralWireQuantumNm /
    (static_cast<double>(kIntegralWireMinFramesBetweenSteps) * kPeriodS);
constexpr double kIntegralEnterErrorDeg = 3.0;
constexpr double kIntegralExitErrorDeg = 3.5;
constexpr double kIntegralEnterVelocityDegS = 1.0;
constexpr double kIntegralExitVelocityDegS = 2.0;
constexpr double kIntegralDeadbandDeg = 0.10;
constexpr double kIntegralProfileEndpointProximityDeg = 3.0;
constexpr int kIntegralDwellFrames = 30;
constexpr double kPrecisionToleranceDeg = 1.0;
constexpr double kFastVelocityHardDegS = 40.0;
constexpr double kSlowVelocityHardDegS = 30.0;
constexpr double kCycleDeadlineMs = 20.0;
constexpr double kScheduleLateHardMs = 2.0;
constexpr int kTemperatureLimitC = 60;
constexpr int kTemperatureRiseHardC = 5;
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

bool integral_profile_endpoint_gate(double endpoint_deg,
                                    double desired_reference_deg,
                                    double last_governed_reference_deg) {
  return std::abs(endpoint_deg - desired_reference_deg) <=
             kIntegralProfileEndpointProximityDeg &&
      std::abs(endpoint_deg - last_governed_reference_deg) <=
             kIntegralProfileEndpointProximityDeg;
}

std::int16_t load_i16_le(const std::uint8_t* value) {
  const std::uint16_t raw = static_cast<std::uint16_t>(value[0]) |
      static_cast<std::uint16_t>(
          static_cast<std::uint16_t>(value[1]) << 8U);
  return static_cast<std::int16_t>(raw);
}

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
  const std::string expected_execute =
      cli.motor + "_INDEPENDENT_PRECISION_I025_R2_AUTHORIZED=YES";
  if (cli.execute_gate != expected_execute)
    throw std::runtime_error("EXECUTE_GATE_MISSING");
  return cli;
}

struct PairFeedback {
  Feedback active;
  Feedback other;
  double q_ref_desired_deg = 0.0;
  double dq_ref_desired_deg_s = 0.0;
  double q_ref_governed_deg = 0.0;
  double dq_ref_governed_deg_s = 0.0;
  double q_active_deg = 0.0;
  double q_other_deg = 0.0;
  double velocity_fast_deg_s = 0.0;
  double velocity_slow_deg_s = 0.0;
  double logical_tau_nm = 0.0;
  double kp_command = 0.0;
  double governor_alpha = 0.0;
  double integral_tau_logical_nm = 0.0;
  double integral_residual_nm = 0.0;
  int integral_wire_frames_since_step = 0;
  int integral_wire_q8 = 0;
  double tau_ff_raw_nm = 0.0;
  double predicted_pd_raw_nm = 0.0;
  double predicted_total_raw_nm = 0.0;
  bool integral_enabled = false;
  double cycle_ms = 0.0;
};

struct QuantizedIntegralState {
  double wire_nm = 0.0;
  double residual_nm = 0.0;
  int residual_direction = 0;
  int frames_since_step = kIntegralWireMinFramesBetweenSteps;
  bool stepped = false;
};

QuantizedIntegralState advance_quantized_integral(
    const QuantizedIntegralState& current, double requested_rate_nm_s) {
  if (!std::isfinite(requested_rate_nm_s))
    throw std::runtime_error("INTEGRAL_RATE_NOT_FINITE");
  const int current_q8 = static_cast<int>(std::llround(
      current.wire_nm / kIntegralWireQuantumNm));
  if (std::abs(current.wire_nm -
               static_cast<double>(current_q8) * kIntegralWireQuantumNm) >
          1e-12 ||
      std::abs(current_q8) > kIntegralWireQ8Hard)
    throw std::runtime_error("INTEGRAL_WIRE_STATE_INVALID");

  QuantizedIntegralState next = current;
  next.stepped = false;
  next.frames_since_step = std::min(
      current.frames_since_step + 1, kIntegralWireMinFramesBetweenSteps);
  const double rate_nm_s = std::clamp(
      requested_rate_nm_s, -kIntegralAccumulatorRateHardNmS,
      kIntegralAccumulatorRateHardNmS);
  const int direction = rate_nm_s > 1e-15 ? 1 :
      (rate_nm_s < -1e-15 ? -1 : 0);
  if (direction == 0) {
    next.residual_nm = 0.0;
    next.residual_direction = 0;
    return next;
  }
  if (current.residual_direction != direction)
    next.residual_nm = 0.0;
  next.residual_direction = direction;
  next.residual_nm = std::clamp(
      next.residual_nm + rate_nm_s * kPeriodS,
      -kIntegralWireQuantumNm, kIntegralWireQuantumNm);

  if (next.frames_since_step >= kIntegralWireMinFramesBetweenSteps &&
      std::abs(next.residual_nm) >= kIntegralWireQuantumNm - 1e-12) {
    const int step_direction = next.residual_nm > 0.0 ? 1 : -1;
    const int candidate_q8 = current_q8 + step_direction;
    if (std::abs(candidate_q8) <= kIntegralWireQ8Hard) {
      next.wire_nm =
          static_cast<double>(candidate_q8) * kIntegralWireQuantumNm;
      next.residual_nm -=
          static_cast<double>(step_direction) * kIntegralWireQuantumNm;
      next.frames_since_step = 0;
      next.stepped = true;
    } else {
      next.residual_nm = 0.0;
      next.residual_direction = 0;
    }
  }
  return next;
}

QuantizedIntegralState reject_quantized_integral_step(
    const QuantizedIntegralState& current) {
  QuantizedIntegralState held = current;
  held.residual_nm = 0.0;
  held.residual_direction = 0;
  held.frames_since_step = std::min(
      current.frames_since_step + 1, kIntegralWireMinFramesBetweenSteps);
  held.stepped = false;
  return held;
}

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
    csv_ << "tick,timestamp_s,phase,active_motor,active_id,other_id,"
            "q_ref_desired_deg,dq_ref_desired_deg_s,q_ref_governed_deg,"
            "dq_ref_governed_deg_s,q_active_deg,q_other_deg,velocity_fast_deg_s,"
            "velocity_slow_deg_s,active_q_raw,other_q_raw,active_dq_raw,"
            "other_dq_raw,active_tau_raw,other_tau_raw,logical_tau_nm,"
            "kp_command,governor_alpha,integral_tau_logical_nm,"
            "integral_residual_nm,integral_wire_frames_since_step,"
            "integral_wire_q8,tau_ff_raw_nm,"
            "predicted_pd_raw_nm,predicted_total_raw_nm,integral_enabled,"
            "active_temp_c,other_temp_c,active_merror,other_merror,"
            "active_mode,other_mode,active_valid,other_valid,cycle_ms\n";
  }

  ~Runner() { safe_brake_noexcept(); }

  int run() {
    try {
      capture_brake_reference();
      const auto initial = hold("INITIAL_KP_RAMP_HOLD", 0.0, kInitialHoldS,
                                false);
      const double initial_final = tail_median(initial, 50);
      const double initial_max = max_abs(initial);
      if (initial_max > 1.0 || std::abs(initial_final) > 0.5)
        throw std::runtime_error("INITIAL_HOLD_FAILED");

      profile("PLUS_10_OUTBOUND", 0.0, 10.0);
      const auto plus = hold("PLUS_10_ENDPOINT_I025", 10.0,
                             kEndpointHoldS, true);
      const double plus_actual = tail_median(plus, 50);
      if (std::abs(plus_actual - 10.0) > kPrecisionToleranceDeg)
        throw std::runtime_error("PLUS_ENDPOINT_PRECISION_FAILED");
      unwind_integral("PLUS_10_I_UNWIND", 10.0);
      profile("PLUS_10_RETURN", 10.0, 0.0);
      const auto center1 = hold("FIRST_CENTER_I025", 0.0,
                                kEndpointHoldS, true);
      const double center1_actual = tail_median(center1, 50);
      if (std::abs(center1_actual) > kPrecisionToleranceDeg)
        throw std::runtime_error("FIRST_CENTER_PRECISION_FAILED");
      unwind_integral("FIRST_CENTER_I_UNWIND", 0.0);

      profile("MINUS_10_OUTBOUND", 0.0, -10.0);
      const auto minus = hold("MINUS_10_ENDPOINT_I025", -10.0,
                              kEndpointHoldS, true);
      const double minus_actual = tail_median(minus, 50);
      if (std::abs(minus_actual + 10.0) > kPrecisionToleranceDeg)
        throw std::runtime_error("MINUS_ENDPOINT_PRECISION_FAILED");
      unwind_integral("MINUS_10_I_UNWIND", -10.0);
      profile("MINUS_10_RETURN", -10.0, 0.0);
      const auto center2 = hold("FINAL_CENTER_I025", 0.0,
                                kEndpointHoldS, true);
      const double center2_actual = tail_median(center2, 50);
      if (std::abs(center2_actual) > kPrecisionToleranceDeg)
        throw std::runtime_error("FINAL_CENTER_PRECISION_FAILED");
      unwind_integral("FINAL_CENTER_I_UNWIND", 0.0);

      const bool final_brake = brake_both(20, true);
      const bool visible = plus_actual >= 5.0 && minus_actual <= -5.0;
      const bool precise =
          std::abs(plus_actual - 10.0) <= kPrecisionToleranceDeg &&
          std::abs(center1_actual) <= kPrecisionToleranceDeg &&
          std::abs(minus_actual + 10.0) <= kPrecisionToleranceDeg &&
          std::abs(center2_actual) <= kPrecisionToleranceDeg;
      std::cout << std::setprecision(17)
                << "RUN=V15_30D_R2_FT_J2_INDEPENDENT_PRECISION_I025\n"
                << "ACTIVE_MOTOR=" << cli_.motor << '\n'
                << "ROUTE=0_TO_PLUS10_TO_0_TO_MINUS10_TO_0\n"
                << "KP=" << kKp << "\nKD=" << kKd
                << "\nINTEGRAL_REQUESTED_BOUND_NM="
                << kIntegralRequestedTorqueHardNm
                << "\nINTEGRAL_WIRE_TAU_HARD_NM=" << kIntegralTorqueHardNm
                << "\nINTEGRAL_WIRE_QUANTUM_NM=" << kIntegralWireQuantumNm
                << "\nINTEGRAL_WIRE_MIN_FRAMES_BETWEEN_STEPS="
                << kIntegralWireMinFramesBetweenSteps
                << "\nINTEGRAL_WIRE_AVERAGE_SLEW_CEILING_NM_S="
                << kIntegralWireAverageSlewCeilingNmS
                << "\nENDPOINT_HOLD_S=" << kEndpointHoldS
                << "\nPROFILE_INTEGRAL_ENDPOINT_PROXIMITY_DEG="
                << kIntegralProfileEndpointProximityDeg
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
                << "MAX_ABS_INTEGRAL_TAU_LOGICAL_NM="
                << max_integral_tau_nm_ << '\n'
                << "MIN_REFERENCE_GOVERNOR_ALPHA=" << min_governor_alpha_
                << "\nREFERENCE_GOVERNOR_LIMITED_FRAMES="
                << governor_limited_frames_ << '\n'
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

  void reset_integral() {
    integral_state_ = QuantizedIntegralState{};
    integral_enabled_ = false;
    integral_dwell_frames_ = 0;
  }

  PairFeedback cycle(const std::string& phase, double q_ref_desired_deg,
                     double dq_ref_desired_deg_s, bool foc,
                     bool allow_integral) {
    check_stop();
    if (foc && !have_previous_)
      throw std::runtime_error("ACTIVE_COMMAND_WITHOUT_FRESH_REFERENCE");
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
    if (!temperature_baseline_valid_)
      throw std::runtime_error("TEMPERATURE_BASELINE_NOT_VALID");
    if (other.data.temp - baseline_other_temp_c_ >=
            kTemperatureRiseHardC ||
        previous_active_temp_c_ - baseline_active_temp_c_ >=
            kTemperatureRiseHardC)
      throw std::runtime_error("TEMPERATURE_RISE_LIMIT_PRE_ENABLE");
    check_stop();

    const double kp_command = foc
        ? std::min(kKp, kKpRampStart +
              (kKp - kKpRampStart) * foc_elapsed_s_ / kKpRampDurationS)
        : 0.0;
    double requested_integral_rate_nm_s = 0.0;
    if (!foc) {
      reset_integral();
    } else if (!allow_integral) {
      integral_enabled_ = false;
      integral_dwell_frames_ = 0;
      if (std::abs(integral_state_.wire_nm) > 1e-12)
        requested_integral_rate_nm_s = -std::copysign(
            kIntegralAccumulatorRateHardNmS, integral_state_.wire_nm);
    } else {
      const double error_deg = q_ref_desired_deg - previous_q_active_deg_;
      const bool enter_gate =
          std::abs(error_deg) <= kIntegralEnterErrorDeg &&
          std::abs(velocity_slow_deg_s_) <= kIntegralEnterVelocityDegS &&
          std::abs(dq_ref_desired_deg_s) <= kIntegralEnterVelocityDegS &&
          std::abs(previous_active_tau_raw_nm_) < 0.40;
      const bool exit_gate =
          std::abs(error_deg) > kIntegralExitErrorDeg ||
          std::abs(velocity_slow_deg_s_) > kIntegralExitVelocityDegS ||
          std::abs(dq_ref_desired_deg_s) > kIntegralExitVelocityDegS ||
          std::abs(previous_active_tau_raw_nm_) >= 0.40;
      if (!integral_enabled_) {
        integral_dwell_frames_ = enter_gate ? integral_dwell_frames_ + 1 : 0;
        if (integral_dwell_frames_ >= kIntegralDwellFrames)
          integral_enabled_ = true;
      } else if (exit_gate) {
        integral_enabled_ = false;
        integral_dwell_frames_ = 0;
      }
      if (integral_enabled_ &&
          std::abs(error_deg) > kIntegralDeadbandDeg) {
        const double rotor_error_rad = kGear * deg_to_rad(error_deg);
        requested_integral_rate_nm_s = std::clamp(
            kIntegralKiPerRotorRadS * rotor_error_rad,
            -kIntegralAccumulatorRateHardNmS,
            kIntegralAccumulatorRateHardNmS);
      } else if (std::abs(integral_state_.wire_nm) > 1e-12)
        requested_integral_rate_nm_s = -std::copysign(
            kIntegralAccumulatorRateHardNmS, integral_state_.wire_nm);
    }
    QuantizedIntegralState integral_candidate = foc
        ? advance_quantized_integral(integral_state_,
                                     requested_integral_rate_nm_s)
        : integral_state_;
    double integral_candidate_nm = integral_candidate.wire_nm;

    auto predicted_total = [&](double alpha, double integral_nm,
                               double* pd_raw_nm,
                               double* q_governed_deg,
                               double* dq_governed_deg_s) {
      const double q_governed = last_command_q_ref_deg_ + alpha *
          (q_ref_desired_deg - last_command_q_ref_deg_);
      const double dq_governed = alpha * dq_ref_desired_deg_s;
      const double q_command_raw = active_center_raw_ + active_sign_ *
          kGear * deg_to_rad(q_governed);
      const double dq_command_raw = active_sign_ * kGear *
          deg_to_rad(dq_governed);
      const double pd = kp_command *
          (q_command_raw - previous_active_raw_q_) +
          kKd * (dq_command_raw - previous_active_raw_dq_);
      if (pd_raw_nm != nullptr) *pd_raw_nm = pd;
      if (q_governed_deg != nullptr) *q_governed_deg = q_governed;
      if (dq_governed_deg_s != nullptr) *dq_governed_deg_s = dq_governed;
      return pd + active_sign_ * integral_nm;
    };

    auto solve_governor = [&](double integral_nm) {
      const double at_zero = std::abs(predicted_total(
          0.0, integral_nm, nullptr, nullptr, nullptr));
      if (at_zero > kRotorPredictedWorkNm + 1e-12) return -1.0;
      if (std::abs(predicted_total(
              1.0, integral_nm, nullptr, nullptr, nullptr)) <=
          kRotorPredictedWorkNm + 1e-12)
        return 1.0;
      double low = 0.0;
      double high = 1.0;
      for (int iteration = 0; iteration < 32; ++iteration) {
        const double middle = 0.5 * (low + high);
        if (std::abs(predicted_total(
                middle, integral_nm, nullptr, nullptr, nullptr)) <=
            kRotorPredictedWorkNm)
          low = middle;
        else
          high = middle;
      }
      return low;
    };

    auto integral_feasible_at_frozen_reference = [&](double integral_nm) {
      return std::abs(predicted_total(
          0.0, integral_nm, nullptr, nullptr, nullptr)) <=
          kRotorPredictedWorkNm + 1e-12;
    };

    if (foc && !integral_feasible_at_frozen_reference(
                   integral_candidate_nm)) {
      if (!integral_feasible_at_frozen_reference(integral_state_.wire_nm))
        throw std::runtime_error("INTEGRAL_FEASIBLE_INTERVAL_EMPTY");
      integral_candidate = reject_quantized_integral_step(integral_state_);
      integral_candidate_nm = integral_candidate.wire_nm;
    }
    double governor_alpha = foc ? solve_governor(integral_candidate_nm) : 0.0;
    if (foc && governor_alpha < 0.0)
      throw std::runtime_error("REFERENCE_GOVERNOR_NO_FEASIBLE_WORK_COMMAND");
    if (foc && governor_alpha < 1.0 - 1e-9 &&
        integral_candidate.stepped &&
        std::abs(integral_candidate_nm) >
            std::abs(integral_state_.wire_nm) + 1e-12) {
      integral_candidate = reject_quantized_integral_step(integral_state_);
      integral_candidate_nm = integral_candidate.wire_nm;
      governor_alpha = solve_governor(integral_candidate_nm);
      if (governor_alpha < 0.0)
        throw std::runtime_error("ANTI_WINDUP_NO_FEASIBLE_WORK_COMMAND");
    }

    double predicted_pd_raw_nm = 0.0;
    double q_ref_governed_deg = 0.0;
    double dq_ref_governed_deg_s = 0.0;
    const double predicted_total_raw_nm = foc
        ? predicted_total(governor_alpha, integral_candidate_nm,
                          &predicted_pd_raw_nm, &q_ref_governed_deg,
                          &dq_ref_governed_deg_s)
        : 0.0;
    if (foc && std::abs(predicted_total_raw_nm) >= kRotorPredictedHardNm)
      throw std::runtime_error("PREDICTED_TOTAL_ROTOR_TORQUE_HARD_LIMIT");
    if (foc && std::abs(predicted_total_raw_nm) >
        kRotorPredictedWorkNm + 1e-9)
      throw std::runtime_error("PREDICTED_TOTAL_ROTOR_TORQUE_WORK_LIMIT");

    const double q_command = foc
        ? active_center_raw_ + active_sign_ * kGear *
              deg_to_rad(q_ref_governed_deg)
        : 0.0;
    const double dq_command = foc
        ? active_sign_ * kGear * deg_to_rad(dq_ref_governed_deg_s)
        : 0.0;
    const double tau_ff_raw_nm = foc
        ? active_sign_ * integral_candidate_nm : 0.0;
    MotorCmd active_command = foc
        ? make_command(active_id_, kFocMode, q_command, dq_command,
                       kp_command, kKd, tau_ff_raw_nm)
        : brake_command(active_id_);
    if (foc) {
      const int expected_tau_q8 = static_cast<int>(std::llround(
          tau_ff_raw_nm / kIntegralWireQuantumNm));
      const std::uint8_t* wire = active_command.get_motor_send_data();
      if (wire == nullptr || load_i16_le(wire + 3) != expected_tau_q8 ||
          std::abs(expected_tau_q8) > kIntegralWireQ8Hard)
        throw std::runtime_error("INTEGRAL_WIRE_ENCODING_MISMATCH");
    }
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
    pair.q_ref_desired_deg = q_ref_desired_deg;
    pair.dq_ref_desired_deg_s = dq_ref_desired_deg_s;
    pair.q_ref_governed_deg = q_ref_governed_deg;
    pair.dq_ref_governed_deg_s = dq_ref_governed_deg_s;
    pair.q_active_deg = rad_to_deg(active_sign_ *
        wrap_pi(static_cast<double>(active.data.q) - active_center_raw_) / kGear);
    pair.q_other_deg = rad_to_deg(other_sign_ *
        wrap_pi(static_cast<double>(other.data.q) - other_center_raw_) / kGear);
    pair.logical_tau_nm = active_sign_ * kGear * active.data.tau;
    pair.kp_command = kp_command;
    pair.governor_alpha = governor_alpha;
    pair.integral_tau_logical_nm = integral_candidate_nm;
    pair.integral_residual_nm = integral_candidate.residual_nm;
    pair.integral_wire_frames_since_step =
        integral_candidate.frames_since_step;
    pair.integral_wire_q8 = static_cast<int>(std::llround(
        integral_candidate_nm / kIntegralWireQuantumNm));
    pair.tau_ff_raw_nm = tau_ff_raw_nm;
    pair.predicted_pd_raw_nm = predicted_pd_raw_nm;
    pair.predicted_total_raw_nm = predicted_total_raw_nm;
    pair.integral_enabled = integral_enabled_;

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
    max_integral_tau_nm_ = std::max(
        max_integral_tau_nm_, std::abs(integral_candidate_nm));
    min_governor_alpha_ = std::min(min_governor_alpha_, governor_alpha);
    if (foc && governor_alpha < 1.0 - 1e-9) ++governor_limited_frames_;
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
    if (std::abs(active.data.tau) >= kActiveRotorTorqueFeedbackHardNm)
      throw std::runtime_error("ACTIVE_TORQUE_FEEDBACK_LIMIT_EXCEEDED");
    if (std::abs(other.data.tau) > kOtherRotorTorqueFeedbackHardNm)
      throw std::runtime_error("OTHER_TORQUE_FEEDBACK_LIMIT_EXCEEDED");
    if (std::abs(pair.velocity_fast_deg_s) > kFastVelocityHardDegS ||
        std::abs(pair.velocity_slow_deg_s) > kSlowVelocityHardDegS)
      throw std::runtime_error("OUTPUT_VELOCITY_LIMIT_EXCEEDED");
    if (active.data.temp - baseline_active_temp_c_ >= kTemperatureRiseHardC ||
        other.data.temp - baseline_other_temp_c_ >= kTemperatureRiseHardC)
      throw std::runtime_error("SHORT_TEST_TEMPERATURE_RISE_LIMIT");

    integral_state_ = integral_candidate;
    last_command_q_ref_deg_ = q_ref_governed_deg;
    previous_timestamp_s_ = timestamp_s;
    previous_q_active_deg_ = pair.q_active_deg;
    previous_active_raw_q_ = active.data.q;
    previous_active_raw_dq_ = active.data.dq;
    previous_active_tau_raw_nm_ = active.data.tau;
    previous_active_temp_c_ = active.data.temp;
    have_previous_ = true;
    if (foc) foc_elapsed_s_ += kPeriodS;

    csv_ << tick_++ << ',' << std::setprecision(17) << timestamp_s << ','
         << phase << ',' << cli_.motor << ',' << active_id_ << ',' << other_id_
         << ',' << q_ref_desired_deg << ',' << dq_ref_desired_deg_s << ','
         << q_ref_governed_deg << ',' << dq_ref_governed_deg_s << ','
         << pair.q_active_deg << ',' << pair.q_other_deg << ','
         << pair.velocity_fast_deg_s << ','
         << pair.velocity_slow_deg_s << ',' << active.data.q << ','
         << other.data.q << ',' << active.data.dq << ',' << other.data.dq << ','
         << active.data.tau << ',' << other.data.tau << ','
         << pair.logical_tau_nm << ',' << kp_command << ',' << governor_alpha
         << ',' << integral_candidate_nm << ','
         << integral_candidate.residual_nm << ','
         << integral_candidate.frames_since_step << ','
         << pair.integral_wire_q8 << ',' << tau_ff_raw_nm << ','
         << predicted_pd_raw_nm << ',' << predicted_total_raw_nm << ','
         << static_cast<int>(integral_enabled_) << ',' << active.data.temp << ','
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
    Feedback last_active;
    Feedback last_other;
    for (int frame = 0; frame < 100; ++frame) {
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
      last_active = active;
      last_other = other;
      if (frame >= 50) {
        active_raw.push_back(active.data.q);
        other_raw.push_back(other.data.q);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (active_raw.size() != 50U || other_raw.size() != 50U)
      throw std::runtime_error("BRAKE_CAPTURE_COUNT_INVALID");
    active_center_raw_ = median(active_raw);
    other_center_raw_ = median(other_raw);
    const auto active_bounds = std::minmax_element(active_raw.begin(),
                                                   active_raw.end());
    const auto other_bounds = std::minmax_element(other_raw.begin(),
                                                  other_raw.end());
    const double active_span_deg = rad_to_deg(
        (*active_bounds.second - *active_bounds.first) / kGear);
    const double other_span_deg = rad_to_deg(
        (*other_bounds.second - *other_bounds.first) / kGear);
    const std::vector<double> active_tail(active_raw.end() - 10,
                                          active_raw.end());
    const std::vector<double> other_tail(other_raw.end() - 10,
                                         other_raw.end());
    const double active_tail_offset_deg = std::abs(rad_to_deg(
        (median(active_tail) - active_center_raw_) / kGear));
    const double other_tail_offset_deg = std::abs(rad_to_deg(
        (median(other_tail) - other_center_raw_) / kGear));
    if (active_span_deg > 0.20 || other_span_deg > 0.20 ||
        active_tail_offset_deg > 0.10 || other_tail_offset_deg > 0.10)
      throw std::runtime_error("BRAKE_CAPTURE_STATIC_GATE_FAILED");
    previous_timestamp_s_ =
        std::chrono::duration<double>(Clock::now() - origin_).count();
    previous_q_active_deg_ = rad_to_deg(active_sign_ *
        wrap_pi(last_active.data.q - active_center_raw_) / kGear);
    previous_active_raw_q_ = last_active.data.q;
    previous_active_raw_dq_ = last_active.data.dq;
    previous_active_tau_raw_nm_ = last_active.data.tau;
    previous_active_temp_c_ = last_active.data.temp;
    baseline_active_temp_c_ = last_active.data.temp;
    baseline_other_temp_c_ = last_other.data.temp;
    temperature_baseline_valid_ = true;
    velocity_fast_deg_s_ = 0.0;
    velocity_slow_deg_s_ = 0.0;
    last_command_q_ref_deg_ = 0.0;
    foc_elapsed_s_ = 0.0;
    reset_integral();
    have_previous_ = true;
  }

  std::vector<double> hold(const std::string& phase, double q_ref_deg,
                           double duration_s, bool allow_integral) {
    const int frames = static_cast<int>(std::ceil(duration_s * kRateHz));
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(frames));
    auto next = Clock::now();
    for (int frame = 0; frame < frames; ++frame) {
      check_stop();
      const double schedule_late_ms = std::chrono::duration<double, std::milli>(
          Clock::now() - next).count();
      if (schedule_late_ms > kScheduleLateHardMs)
        throw std::runtime_error("ACTIVE_SCHEDULE_LATE_LIMIT");
      values.push_back(cycle(phase, q_ref_deg, 0.0, true,
                             allow_integral).q_active_deg);
      next += std::chrono::milliseconds(10);
      std::this_thread::sleep_until(next);
    }
    return values;
  }

  void profile(const std::string& phase, double start_deg, double end_deg) {
    if (std::abs(integral_state_.wire_nm) > 1e-12 ||
        std::abs(integral_state_.residual_nm) > 1e-12)
      throw std::runtime_error("PROFILE_STARTED_WITH_NONZERO_INTEGRAL");
    const int frames = static_cast<int>(std::ceil(kMoveDurationS * kRateHz));
    auto next = Clock::now();
    for (int frame = 1; frame <= frames; ++frame) {
      check_stop();
      const double schedule_late_ms = std::chrono::duration<double, std::milli>(
          Clock::now() - next).count();
      if (schedule_late_ms > kScheduleLateHardMs)
        throw std::runtime_error("ACTIVE_SCHEDULE_LATE_LIMIT");
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
      (void)cycle(phase, q_ref, dq_ref, true,
                  integral_profile_endpoint_gate(
                      end_deg, q_ref, last_command_q_ref_deg_));
      next += std::chrono::milliseconds(10);
      std::this_thread::sleep_until(next);
    }
  }

  void unwind_integral(const std::string& phase, double target_deg) {
    const int maximum_frames =
        kIntegralWireQ8Hard * kIntegralWireMinFramesBetweenSteps + 5;
    auto next = Clock::now();
    for (int frame = 0;
         frame < maximum_frames &&
             std::abs(integral_state_.wire_nm) > 1e-12;
         ++frame) {
      check_stop();
      const double schedule_late_ms = std::chrono::duration<double, std::milli>(
          Clock::now() - next).count();
      if (schedule_late_ms > kScheduleLateHardMs)
        throw std::runtime_error("INTEGRAL_UNWIND_SCHEDULE_LATE_LIMIT");
      const PairFeedback pair = cycle(phase, target_deg, 0.0, true, false);
      if (std::abs(pair.q_active_deg - target_deg) >
          kPrecisionToleranceDeg)
        throw std::runtime_error("INTEGRAL_UNWIND_POSITION_LOSS");
      next += std::chrono::milliseconds(10);
      std::this_thread::sleep_until(next);
    }
    if (std::abs(integral_state_.wire_nm) > 1e-12)
      throw std::runtime_error("INTEGRAL_UNWIND_DID_NOT_REACH_ZERO");
    integral_state_.residual_nm = 0.0;
    integral_state_.residual_direction = 0;
  }

  bool brake_both(int frames, bool log_rows) noexcept {
    reset_integral();
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
               << other_id_ << ",0,0,0,0,0,0,0,0," << active.data.q << ','
               << other.data.q << ',' << active.data.dq << ',' << other.data.dq
               << ',' << active.data.tau << ',' << other.data.tau
               << ",0,0,0,0,0,0,0,0,0,0,0," << active.data.temp << ','
               << other.data.temp << ','
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
  double previous_active_raw_q_ = 0.0;
  double previous_active_raw_dq_ = 0.0;
  double previous_active_tau_raw_nm_ = 0.0;
  int previous_active_temp_c_ = 0;
  double velocity_fast_deg_s_ = 0.0;
  double velocity_slow_deg_s_ = 0.0;
  double last_command_q_ref_deg_ = 0.0;
  double foc_elapsed_s_ = 0.0;
  QuantizedIntegralState integral_state_;
  bool integral_enabled_ = false;
  int integral_dwell_frames_ = 0;
  bool have_previous_ = false;
  bool brake_attempted_ = false;
  double max_tau_nm_ = 0.0;
  double max_active_rotor_tau_nm_ = 0.0;
  double max_other_rotor_tau_nm_ = 0.0;
  double max_other_drift_deg_ = 0.0;
  double max_integral_tau_nm_ = 0.0;
  double min_governor_alpha_ = 1.0;
  int governor_limited_frames_ = 0;
  double max_fast_velocity_deg_s_ = 0.0;
  double max_slow_velocity_deg_s_ = 0.0;
  int max_active_temp_c_ = 0;
  int max_other_temp_c_ = 0;
  int baseline_active_temp_c_ = 0;
  int baseline_other_temp_c_ = 0;
  bool temperature_baseline_valid_ = false;
};

void self_test() {
  if (queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE) != kBrakeMode ||
      queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC) != kFocMode ||
      std::abs(queryGearRatio(MotorType::GO_M8010_6) - kGear) > 1e-6) {
    throw std::runtime_error("SDK_AUTHORITY_MISMATCH");
  }
  (void)brake_command(0);
  (void)brake_command(1);
  for (int q8 = -kIntegralWireQ8Hard;
       q8 <= kIntegralWireQ8Hard; ++q8) {
    MotorCmd command = make_command(
        0, kFocMode, 1.0, 0.0, kKp, kKd,
        static_cast<double>(q8) * kIntegralWireQuantumNm);
    const std::uint8_t* raw = command.get_motor_send_data();
    if (raw == nullptr || load_i16_le(raw + 3) != q8)
      throw std::runtime_error("I025_Q8_ENCODING_SELF_TEST_FAILED");
  }
  if (!(kRotorPredictedWorkNm < kRotorPredictedHardNm &&
        kRotorPredictedHardNm < kActiveRotorTorqueFeedbackHardNm &&
        kIntegralTorqueHardNm < kRotorPredictedWorkNm &&
        kIntegralTorqueHardNm <= kIntegralRequestedTorqueHardNm &&
        kIntegralWireAverageSlewCeilingNmS <=
            kIntegralAccumulatorRateHardNmS &&
        kIntegralWireQuantumNm /
                (static_cast<double>(
                    kIntegralWireMinFramesBetweenSteps - 1) * kPeriodS) >
            kIntegralAccumulatorRateHardNmS &&
        kKpRampStart > 0.0 && kKpRampStart < kKp))
    throw std::runtime_error("TORQUE_OR_GAIN_ORDER_SELF_TEST_FAILED");
  const double profile_peak_velocity_deg_s =
      1.875 * 10.0 / kMoveDurationS;
  if (profile_peak_velocity_deg_s > 2.0 + 1e-12)
    throw std::runtime_error("PROFILE_SPEED_SELF_TEST_FAILED");
  if (std::abs(kEndpointHoldS - 8.0) > 1e-12 ||
      integral_profile_endpoint_gate(10.0, 0.0, 0.0) ||
      integral_profile_endpoint_gate(10.0, 8.0, 6.0) ||
      !integral_profile_endpoint_gate(10.0, 8.0, 8.0))
    throw std::runtime_error("PROFILE_ENDPOINT_GATE_SELF_TEST_FAILED");
  QuantizedIntegralState simulated_integral;
  int last_step_frame = -1000000;
  int absolute_frame = 0;
  auto simulate_and_check = [&](double rate_nm_s) {
    const double previous_wire_nm = simulated_integral.wire_nm;
    simulated_integral = advance_quantized_integral(
        simulated_integral, rate_nm_s);
    const double delta_nm = simulated_integral.wire_nm - previous_wire_nm;
    if (std::abs(delta_nm) > kIntegralWireQuantumNm + 1e-12)
      throw std::runtime_error("INTEGRAL_WIRE_STEP_SELF_TEST_FAILED");
    if (simulated_integral.stepped) {
      if (absolute_frame - last_step_frame <
          kIntegralWireMinFramesBetweenSteps)
        throw std::runtime_error("INTEGRAL_WIRE_INTERVAL_SELF_TEST_FAILED");
      last_step_frame = absolute_frame;
    }
    const int q8 = static_cast<int>(std::llround(
        simulated_integral.wire_nm / kIntegralWireQuantumNm));
    MotorCmd command = make_command(
        0, kFocMode, 1.0, 0.0, kKp, kKd,
        simulated_integral.wire_nm);
    const std::uint8_t* raw = command.get_motor_send_data();
    if (raw == nullptr || load_i16_le(raw + 3) != q8 ||
        std::abs(q8) > kIntegralWireQ8Hard)
      throw std::runtime_error("INTEGRAL_WIRE_RUNTIME_SELF_TEST_FAILED");
    ++absolute_frame;
  };
  for (int frame = 0; frame < 1100; ++frame)
    simulate_and_check(kIntegralAccumulatorRateHardNmS);
  if (std::abs(simulated_integral.wire_nm - kIntegralTorqueHardNm) > 1e-12)
    throw std::runtime_error("INTEGRAL_POSITIVE_CLAMP_SELF_TEST_FAILED");
  for (int frame = 0; frame < 1100; ++frame) {
    const double unwind_rate_nm_s =
        std::abs(simulated_integral.wire_nm) > 1e-12
        ? -std::copysign(kIntegralAccumulatorRateHardNmS,
                         simulated_integral.wire_nm)
        : 0.0;
    simulate_and_check(unwind_rate_nm_s);
  }
  if (std::abs(simulated_integral.wire_nm) > 1e-12)
    throw std::runtime_error("INTEGRAL_UNWIND_SELF_TEST_FAILED");

  std::cout << "V15_30D_R2_FT_J2_INDEPENDENT_PRECISION_SELF_TEST=PASS\n"
            << "SERIAL_PORT_CONSTRUCTED=NO\nACTIVE_MOTION_USED=NO\n"
            << "ROUTE=0_TO_PLUS10_TO_0_TO_MINUS10_TO_0\n"
            << "KP_RAMP_START=" << kKpRampStart << "\nKP_FINAL=" << kKp
            << "\nKD=" << kKd << "\nMOVE_DURATION_S=" << kMoveDurationS
            << "\nENDPOINT_HOLD_S=" << kEndpointHoldS
            << "\nPROFILE_PEAK_VELOCITY_DEG_S="
            << profile_peak_velocity_deg_s << '\n'
            << "PROFILE_INTEGRAL_ENDPOINT_PROXIMITY_DEG="
            << kIntegralProfileEndpointProximityDeg << '\n'
            << "PROFILE_INTEGRAL_REQUIRES_DESIRED_AND_GOVERNED_NEAR=YES\n"
            << "OTHER_MOTOR_COMMAND=CONTINUOUS_BRAKE\n"
            << "COUPLED_SYNC_CONTROL_ENABLED=NO\n"
            << "OTHER_MOTOR_DRIFT_HARD_DEG=" << kOtherMotorEnvelopeDeg << '\n'
            << "ROTOR_PREDICTED_WORK_NM=" << kRotorPredictedWorkNm << '\n'
            << "ROTOR_PREDICTED_HARD_NM=" << kRotorPredictedHardNm << '\n'
            << "ACTIVE_ROTOR_FEEDBACK_TORQUE_HARD_NM="
            << kActiveRotorTorqueFeedbackHardNm << '\n'
            << "OTHER_ROTOR_FEEDBACK_TORQUE_HARD_NM="
            << kOtherRotorTorqueFeedbackHardNm << '\n'
            << "INTEGRAL_REQUESTED_BOUND_NM="
            << kIntegralRequestedTorqueHardNm << '\n'
            << "INTEGRAL_WIRE_Q8_HARD=" << kIntegralWireQ8Hard << '\n'
            << "INTEGRAL_WIRE_TAU_HARD_NM=" << kIntegralTorqueHardNm << '\n'
            << "INTEGRAL_WIRE_QUANTUM_NM=" << kIntegralWireQuantumNm << '\n'
            << "INTEGRAL_KI_PER_ROTOR_RAD_S="
            << kIntegralKiPerRotorRadS << '\n'
            << "INTEGRAL_ACCUMULATOR_RATE_HARD_NM_S="
            << kIntegralAccumulatorRateHardNmS << '\n'
            << "INTEGRAL_WIRE_MIN_FRAMES_BETWEEN_STEPS="
            << kIntegralWireMinFramesBetweenSteps << '\n'
            << "INTEGRAL_WIRE_AVERAGE_SLEW_CEILING_NM_S="
            << kIntegralWireAverageSlewCeilingNmS << '\n'
            << "INTEGRAL_WIRE_ADJACENT_STEP_HARD_LSB=1\n"
            << "INTEGRAL_ENTRY_ERROR_DEG=" << kIntegralEnterErrorDeg << '\n'
            << "INTEGRAL_DWELL_FRAMES=" << kIntegralDwellFrames << '\n'
            << "REFERENCE_GOVERNOR=ENABLED\nANTI_WINDUP=ENABLED\n"
            << "MIT_HOST_PREDICTION_NOT_INTERNAL_HARD_GUARANTEE=YES\n"
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
    std::cerr << "V15_30D_R2_FT_J2_INDEPENDENT_PRECISION_RESULT=BLOCKED\nREASON="
              << error.what() << '\n';
    return 2;
  }
}
