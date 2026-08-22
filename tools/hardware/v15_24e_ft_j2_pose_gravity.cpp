#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

namespace {

using Clock = std::chrono::steady_clock;
constexpr char kPort[] =
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if01-port0";
constexpr char kLockPath[] = "/tmp/v15_23d_ft_j2_channel1.lock";
constexpr int kIdA = 0;
constexpr int kIdB = 1;
constexpr int kBrakeMode = 0;
constexpr int kFocMode = 1;
constexpr int kSignA = -1;
constexpr int kSignB = +1;
constexpr double kPi = 3.14159265358979323846;
constexpr double kGear = 6.3299999237060547;
constexpr double kKp = 0.60;
constexpr double kKd = 0.05;
constexpr double kHz = 100.0;
constexpr double kPeriod = 0.01;
constexpr double kVmax = 10.0 * kPi / 180.0;
constexpr double kAccel = 30.0 * kPi / 180.0;
constexpr double kZeroHoldDuration = 0.50;
constexpr double kGravityHoldDuration = 1.00;
constexpr double kBoundaryToCenter = 5.0 * kPi / 180.0;
constexpr double kCenterExcursion = 5.0 * kPi / 180.0;
constexpr double kCommandLowerBoundary = 0.0;
constexpr double kCommandUpperBoundary =
    kBoundaryToCenter + kCenterExcursion;
// B is a measured, session-local real boundary. Commands never go below B,
// but there is deliberately no extra inward margin. The feedback guard is
// reactive and therefore is not proof that the mechanism can never cross B
// between samples.
constexpr double kFeedbackLowerBoundaryAbort = -0.5 * kPi / 180.0;
constexpr double kFeedbackUpperBoundaryAbort = 12.0 * kPi / 180.0;
constexpr std::array<double, 5> kAuditedJ2OffsetsDeg =
    {0.0, 2.5, 5.0, 7.5, 10.0};
constexpr std::array<double, 5> kCenterRelativeRouteDeg =
    {0.0, 5.0, 0.0, -5.0, 0.0};
constexpr std::array<double, 7> kFullBoundaryRelativeRouteDeg =
    {0.0, 5.0, 10.0, 5.0, 0.0, 5.0, 0.0};
constexpr double kSyncHardAbort = 1.0 * kPi / 180.0;
constexpr double kGravityUnexpectedMotionAbort = 1.5 * kPi / 180.0;
constexpr double kFeedbackVelocityHardAbort = 30.0 * kPi / 180.0;
constexpr int kTempLimit = 60;
constexpr char kModelPath[] =
    "/home/car/go-m8010-robot-arm-v15-20a/"
    "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/mujoco_v15_14/"
    "go_m8010_arm_v15_14_kinematic.xml";
constexpr char kModelHash[] =
    "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9";
constexpr char kAnchorPath[] = "/tmp/v15_24e_model_anchor.json";
constexpr char kEligibilityPath[] = "/tmp/v15_24e_next_phase_gate.json";
constexpr char kAnchorGate[] = "MUJOCO_PHYSICAL_POSE_MATCHED=YES";
constexpr char kBoundaryAnchorConfirmationSource[] =
    "USER_CHAT_EXPLICIT_BOUNDARY_B_PLUS_5DEG_STAGE_ROUTE_AUTHORIZATION";
constexpr char kBoundarySafetyGate[] =
    "STABLE_BOUNDARY_AND_BRAKE_FALLBACK_PATH_CLEAR=YES";
constexpr char kRepoRoot[] = "/home/car/go-m8010-robot-arm-v15-20a";
constexpr char kSourceHead[] =
    "4fc673c77fee836a4931fabec731ea85346f25b3";
constexpr char kMassPath[] =
    "/home/car/go-m8010-robot-arm-v15-20a/V15_15_实测质量账本_v1.json";
constexpr char kMassHash[] =
    "658fa0b2aef1da4200d86fccf742e135215288e77f58fa5095a06f513eaba04a";
constexpr char kComPath[] =
    "/home/car/go-m8010-robot-arm-v15-20a/V15_15_COM账本_v2.json";
constexpr char kComHash[] =
    "1cb975890dd121eb41d41bfee414a2c4643f4d917089970fac60009736fc77ae";
constexpr char kInertiaPath[] =
    "/home/car/go-m8010-robot-arm-v15-20a/V15_16_刚体惯量_Engineering_V1.json";
constexpr char kInertiaHash[] =
    "c6c398532d7144ed6aa340a8d8b765b333d01f1fddc2e3280106c90565614401";

struct LevelConfig {
  double alpha = 0.0;
  double cap = 0.0;
  double ramp_duration = 0.0;
  const char* label = "";
};

volatile std::sig_atomic_t g_stop = 0;
void signal_handler(int) { g_stop = 1; }

double smoothstep5(double x) {
  x = std::clamp(x, 0.0, 1.0);
  return x * x * x * (10.0 + x * (-15.0 + 6.0 * x));
}

std::string sha256_file(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("SHA256_FILE_OPEN_FAILED:" + path);
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(
      EVP_MD_CTX_new(), &EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("SHA256_INIT_FAILED");
  std::array<char, 65536> buffer{};
  while (stream) {
    stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const std::streamsize count = stream.gcount();
    if (count > 0 &&
        EVP_DigestUpdate(context.get(), buffer.data(),
                         static_cast<std::size_t>(count)) != 1)
      throw std::runtime_error("SHA256_UPDATE_FAILED");
  }
  if (!stream.eof()) throw std::runtime_error("SHA256_FILE_READ_FAILED:" + path);
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0;
  if (EVP_DigestFinal_ex(context.get(), digest.data(), &digest_size) != 1 ||
      digest_size != 32U)
    throw std::runtime_error("SHA256_FINAL_FAILED");
  std::ostringstream result;
  result << std::hex << std::setfill('0');
  for (unsigned int i = 0; i < digest_size; ++i)
    result << std::setw(2) << static_cast<unsigned int>(digest[i]);
  return result.str();
}

void verify_frozen_file_hashes() {
  const std::array<std::pair<const char*, const char*>, 4> authorities = {{
      {kModelPath, kModelHash},
      {kMassPath, kMassHash},
      {kComPath, kComHash},
      {kInertiaPath, kInertiaHash},
  }};
  for (const auto& authority : authorities) {
    if (sha256_file(authority.first) != authority.second)
      throw std::runtime_error("FROZEN_AUTHORITY_HASH_MISMATCH_BLOCKED:" +
                               std::string(authority.first));
  }
}

template <typename T>
void zero_object(T& value) {
  static_assert(std::is_trivially_copyable<T>::value, "SDK wrapper ABI");
  std::memset(static_cast<void*>(&value), 0, sizeof(value));
}

std::uint16_t load_u16_le(const std::uint8_t* p) {
  return static_cast<std::uint16_t>(p[0]) |
         static_cast<std::uint16_t>(p[1] << 8U);
}

std::uint16_t crc16_kermit(const std::uint8_t* p, std::size_t n) {
  std::uint16_t crc = 0;
  while (n-- != 0U) {
    crc = static_cast<std::uint16_t>(crc ^ *p++);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0U
          ? static_cast<std::uint16_t>((crc >> 1U) ^ 0x8408U)
          : static_cast<std::uint16_t>(crc >> 1U);
    }
  }
  return crc;
}

MotorCmd make_command(int id, int mode, double q, double dq,
                      double kp, double kd, double tau) {
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
  if (command.hex_len != 17) throw std::runtime_error("COMMAND_LENGTH_INVALID");
  const std::uint8_t* raw = command.get_motor_send_data();
  if (raw == nullptr || raw[0] != 0xfeU || raw[1] != 0xeeU ||
      raw[2] != static_cast<std::uint8_t>(id | (mode << 4)) ||
      (raw[2] & 0x80U) != 0U ||
      load_u16_le(raw + 15) != crc16_kermit(raw, 15)) {
    throw std::runtime_error("COMMAND_PACKET_AUDIT_FAILED");
  }
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
  bool correct = false;
  bool crc_ok = false;
  int id = -1;
  int mode = -1;
  int temp = -1;
  int merror = -1;
  double q = std::numeric_limits<double>::quiet_NaN();
  double dq = std::numeric_limits<double>::quiet_NaN();
  double tau = std::numeric_limits<double>::quiet_NaN();
  bool valid = false;
};

class Unwrapper {
 public:
  double update(double raw) {
    if (!initialized_) {
      initialized_ = true;
      previous_ = raw;
      value_ = raw;
      return value_;
    }
    double delta = raw - previous_;
    while (delta > kPi) delta -= 2.0 * kPi;
    while (delta < -kPi) delta += 2.0 * kPi;
    value_ += delta;
    previous_ = raw;
    return value_;
  }
 private:
  bool initialized_ = false;
  double previous_ = 0.0;
  double value_ = 0.0;
};

double median(std::vector<double> values) {
  if (values.empty()) throw std::runtime_error("MEDIAN_EMPTY");
  std::sort(values.begin(), values.end());
  const std::size_t m = values.size() / 2U;
  return values.size() % 2U == 0U
      ? (values[m - 1U] + values[m]) / 2.0 : values[m];
}

struct ProfilePoint { double q = 0.0; double dq = 0.0; };

class Trapezoid {
 public:
  explicit Trapezoid(double displacement)
      : sign_(displacement < 0.0 ? -1.0 : 1.0),
        distance_(std::abs(displacement)) {
    const double threshold = kVmax * kVmax / kAccel;
    if (distance_ <= threshold) {
      t_acc_ = std::sqrt(distance_ / kAccel);
      v_peak_ = kAccel * t_acc_;
      t_cruise_ = 0.0;
    } else {
      t_acc_ = kVmax / kAccel;
      v_peak_ = kVmax;
      t_cruise_ = (distance_ - kAccel * t_acc_ * t_acc_) / kVmax;
    }
    duration_ = 2.0 * t_acc_ + t_cruise_;
  }
  double duration() const { return duration_; }
  ProfilePoint sample(double t) const {
    t = std::clamp(t, 0.0, duration_);
    const double d_acc = 0.5 * kAccel * t_acc_ * t_acc_;
    double q = 0.0;
    double v = 0.0;
    if (t < t_acc_) {
      q = 0.5 * kAccel * t * t;
      v = kAccel * t;
    } else if (t < t_acc_ + t_cruise_) {
      const double u = t - t_acc_;
      q = d_acc + v_peak_ * u;
      v = v_peak_;
    } else if (t < duration_) {
      const double u = duration_ - t;
      q = distance_ - 0.5 * kAccel * u * u;
      v = kAccel * u;
    } else {
      q = distance_;
      v = 0.0;
    }
    return {sign_ * q, sign_ * v};
  }
 private:
  double sign_ = 1.0;
  double distance_ = 0.0;
  double t_acc_ = 0.0;
  double t_cruise_ = 0.0;
  double v_peak_ = 0.0;
  double duration_ = 0.0;
};

struct Cli {
  std::string phase;
  std::string output;
  std::string anchor_file;
  std::string eligibility_file;
  std::string authorization_gate;
  std::string safety_gate;
  std::string power_on_gate;
  std::string boundary_gate;
  bool self_test = false;
  bool anchor_self_test = false;
};

void verify_boundary_gate(const std::string& gate) {
  if (gate != kBoundarySafetyGate)
    throw std::runtime_error("STABLE_BOUNDARY_BRAKE_FALLBACK_GATE_MISSING");
}

Cli parse_cli(int argc, char** argv) {
  Cli cli;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&]() {
      if (++i >= argc) throw std::runtime_error("CLI_VALUE_MISSING");
      return std::string(argv[i]);
    };
    if (arg == "--self-test") cli.self_test = true;
    else if (arg == "--anchor-self-test") cli.anchor_self_test = true;
    else if (arg == "--phase") cli.phase = value();
    else if (arg == "--output") cli.output = value();
    else if (arg == "--anchor-file") cli.anchor_file = value();
    else if (arg == "--eligibility-file") cli.eligibility_file = value();
    else if (arg == "--authorization-gate") cli.authorization_gate = value();
    else if (arg == "--safety-gate") cli.safety_gate = value();
    else if (arg == "--power-on-gate") cli.power_on_gate = value();
    else if (arg == "--boundary-gate") cli.boundary_gate = value();
    else throw std::runtime_error("CLI_OPTION_NOT_ALLOWED");
  }
  if (cli.self_test) return cli;
  if (cli.anchor_self_test) {
    if (cli.anchor_file != kAnchorPath)
      throw std::runtime_error("ANCHOR_PATH_NOT_ALLOWED");
    return cli;
  }
  const bool allowed_phase = cli.phase == "level-a-run" ||
      cli.phase == "level-a-repeat" || cli.phase == "level-b-run" ||
      cli.phase == "level-b-repeat";
  if (!allowed_phase)
    throw std::runtime_error("PHASE_NOT_ALLOWED");
  const std::string expected_output =
      cli.phase == "level-a-run" ?
          "hardware/v15_24e_ft/j2_pose_gravity_025_run.csv" :
      cli.phase == "level-a-repeat" ?
          "hardware/v15_24e_ft/j2_pose_gravity_025_repeat.csv" :
      cli.phase == "level-b-run" ?
          "hardware/v15_24e_ft/j2_pose_gravity_050_run.csv" :
          "hardware/v15_24e_ft/j2_pose_gravity_050_repeat.csv";
  if (cli.output != expected_output)
    throw std::runtime_error("OUTPUT_PATH_NOT_ALLOWED");
  if (cli.anchor_file != kAnchorPath)
    throw std::runtime_error("ANCHOR_PATH_NOT_ALLOWED");
  if (cli.phase == "level-a-run") {
    if (!cli.eligibility_file.empty())
      throw std::runtime_error("LEVEL_A_RUN_MUST_NOT_USE_PHASE_GATE");
  } else if (cli.eligibility_file != kEligibilityPath) {
    throw std::runtime_error("NEXT_PHASE_ELIGIBILITY_GATE_MISSING");
  }
  const std::string expected_auth =
      cli.phase == "level-a-run" ?
          "J2_POSE_GRAVITY_025_RUN_AUTHORIZED=YES" :
      cli.phase == "level-a-repeat" ?
          "J2_POSE_GRAVITY_025_EXACT_REPEAT_AUTHORIZED=YES" :
      cli.phase == "level-b-run" ?
          "J2_POSE_GRAVITY_050_RUN_AUTHORIZED=YES" :
          "J2_POSE_GRAVITY_050_EXACT_REPEAT_AUTHORIZED=YES";
  if (cli.authorization_gate != expected_auth ||
      cli.safety_gate !=
          "BIG_ARM_AND_FOREARM_INSTALLED_ROUTE_CLEAR_OPERATOR_READY=YES" ||
      cli.power_on_gate != "J2_POSE_GRAVITY_24V_POWER_ON_CONFIRMED=YES")
    throw std::runtime_error("DUAL_OPERATOR_GATE_MISSING");
  verify_boundary_gate(cli.boundary_gate);
  return cli;
}

LevelConfig level_config(const Cli& cli) {
  if (cli.phase == "level-a-run" || cli.phase == "level-a-repeat")
    return {0.25, 0.30, 1.00, "LEVEL_A_025"};
  return {0.50, 0.60, 1.50, "LEVEL_B_050"};
}

struct ModelAnchor {
  std::array<double, 5> q{};
  double anchor_tau = 0.0;
  double envelope_min = 0.0;
  double envelope_max = 0.0;
  int gravity_sign = 0;
  int uncertainty_samples = 0;
};

template <std::size_t N>
bool json_number_array_matches(const nlohmann::json& value,
                               const std::array<double, N>& expected) {
  if (!value.is_array() || value.size() != expected.size()) return false;
  for (std::size_t i = 0; i < expected.size(); ++i) {
    if (!value.at(i).is_number()) return false;
    const double actual = value.at(i).get<double>();
    if (!std::isfinite(actual) || std::abs(actual - expected[i]) > 1e-12)
      return false;
  }
  return true;
}

ModelAnchor load_model_anchor(const std::string& path) {
  verify_frozen_file_hashes();
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("MODEL_ANCHOR_OPEN_FAILED");
  nlohmann::json doc;
  stream >> doc;
  if (doc.at("schema").get<std::string>() !=
          "SESSION_LOCAL_GRAVITY_ANCHOR_V1_MODEL" ||
      doc.at("scope").get<std::string>() !=
          "SESSION_ONLY_NOT_PERMANENT_ZERO" ||
      doc.at("operator_gate").get<std::string>() != kAnchorGate ||
      doc.at("operator_gate_entry_method").get<std::string>() !=
          kBoundaryAnchorConfirmationSource ||
      doc.at("anchor_role").get<std::string>() != "STABLE_BOUNDARY_B" ||
      doc.at("model_reference_equation").get<std::string>() !=
          "q_model[J2]=q_boundary_B_model[J2]+q_relative_from_boundary_B;"
          "NO_WRAP_OR_NOMINAL_RANGE_CLIPPING" ||
      doc.at("model").at("sha256").get<std::string>() != kModelHash ||
      doc.at("permanent_zero_modified").get<bool>() ||
      doc.at("cad_zero").get<std::string>() != "PENDING" ||
      doc.at("ros_zero").get<std::string>() != "PENDING")
    throw std::runtime_error("MODEL_ANCHOR_AUTHORITY_MISMATCH");
  const auto& ledgers = doc.at("frozen_ledgers_sha256");
  if (ledgers.at("V15_15_实测质量账本_v1.json").get<std::string>() !=
          "658fa0b2aef1da4200d86fccf742e135215288e77f58fa5095a06f513eaba04a" ||
      ledgers.at("V15_15_COM账本_v2.json").get<std::string>() !=
          "1cb975890dd121eb41d41bfee414a2c4643f4d917089970fac60009736fc77ae" ||
      ledgers.at("V15_16_刚体惯量_Engineering_V1.json").get<std::string>() !=
          "c6c398532d7144ed6aa340a8d8b765b333d01f1fddc2e3280106c90565614401")
    throw std::runtime_error("LEDGER_AUTHORITY_MISMATCH");
  const auto& route = doc.at("route_contract");
  if (route.at("boundary_label").get<std::string>() != "B" ||
      route.at("test_center_label").get<std::string>() != "C" ||
      std::abs(route.at("boundary_to_center_logical_j2_offset_deg")
                   .get<double>() - 5.0) > 1e-12 ||
      !json_number_array_matches(route.at("formal_center_relative_route_deg"),
                                 kCenterRelativeRouteDeg) ||
      !json_number_array_matches(route.at("full_boundary_relative_route_deg"),
                                 kFullBoundaryRelativeRouteDeg) ||
      !json_number_array_matches(route.at("safe_boundary_relative_interval_deg"),
                                 std::array<double, 2>{0.0, 10.0}) ||
      route.at("center_is_model_anchor").get<bool>() ||
      !route.at("boundary_is_model_anchor").get<bool>())
    throw std::runtime_error("BOUNDARY_ROUTE_CONTRACT_MISMATCH");
  const auto& q = doc.at("q_anchor_model_rad");
  ModelAnchor result;
  const std::array<const char*, 5> names = {"J2", "J3", "J4", "J5", "J6"};
  for (std::size_t i = 0; i < names.size(); ++i) {
    result.q[i] = q.at(names[i]).get<double>();
    if (!std::isfinite(result.q[i]))
      throw std::runtime_error("MODEL_ANCHOR_NONFINITE");
  }
  const auto& center_q = doc.at("q_test_center_model_rad");
  for (std::size_t i = 0; i < names.size(); ++i) {
    const double actual = center_q.at(names[i]).get<double>();
    const double expected = result.q[i] +
        (i == 0U ? kBoundaryToCenter : 0.0);
    if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-12)
      throw std::runtime_error("TEST_CENTER_MODEL_DERIVATION_MISMATCH");
  }
  const auto& audit = doc.at("uncertainty_audit");
  if (!audit.at("sign_robust").get<bool>() ||
      !audit.at("coverage_complete").get<bool>() ||
      !audit.at("all_samples_finite").get<bool>() ||
      audit.at("finite_sample_count").get<int>() != 405 ||
      audit.at("nonfinite_sample_count").get<int>() != 0 ||
      audit.at("joint_values_clipped").get<bool>() ||
      audit.at("required_sample_count").get<int>() != 405 ||
      audit.at("sample_count").get<int>() != 405 ||
      audit.at("anchor_role").get<std::string>() != "STABLE_BOUNDARY_B" ||
      audit.at("method").get<std::string>() !=
          "BOUNDARY_B_ABSOLUTE_ROUTE_0_TO_PLUS10_5_POINT_X_"
          "J3_J4_J5_J6_SIMULTANEOUS_PLUS_MINUS_5DEG_CORNERS" ||
      audit.at("reference_pose_label").get<std::string>() !=
          "BOUNDARY_B_MODEL_REFERENCE" ||
      !json_number_array_matches(audit.at("j2_offsets_from_reference_deg"),
                                 kAuditedJ2OffsetsDeg) ||
      !json_number_array_matches(audit.at("j2_route_offsets_deg"),
                                 kAuditedJ2OffsetsDeg) ||
      !json_number_array_matches(
          audit.at("formal_center_relative_route_deg"),
          kCenterRelativeRouteDeg) ||
      !json_number_array_matches(
          audit.at("full_planned_boundary_relative_route_deg"),
          kFullBoundaryRelativeRouteDeg) ||
      std::abs(audit.at("test_center_offset_from_boundary_deg")
                   .get<double>() - 5.0) > 1e-12 ||
      std::abs(audit.at("planned_boundary_relative_min_deg")
                   .get<double>()) > 1e-12 ||
      std::abs(audit.at("planned_boundary_relative_max_deg")
                   .get<double>() - 10.0) > 1e-12 ||
      audit.at("crosses_below_boundary").get<bool>())
    throw std::runtime_error("BOUNDARY_ROUTE_405_AUDIT_MISMATCH_BLOCKED");
  result.anchor_tau = audit.at("anchor_tau_g_j2_nm").get<double>();
  result.envelope_min = audit.at("tau_g_j2_min_nm").get<double>();
  result.envelope_max = audit.at("tau_g_j2_max_nm").get<double>();
  result.uncertainty_samples = audit.at("sample_count").get<int>();
  result.gravity_sign = result.envelope_min > 0.0 ? 1 :
      (result.envelope_max < 0.0 ? -1 : 0);
  if (result.gravity_sign == 0 ||
      result.gravity_sign * result.anchor_tau <= 0.0 ||
      result.uncertainty_samples <= 0)
    throw std::runtime_error("GRAVITY_SIGN_AUTHORITY_INVALID");
  return result;
}

void verify_phase_eligibility(const Cli& cli) {
  const std::string output_absolute = std::string(kRepoRoot) + "/" + cli.output;
  if (std::filesystem::exists(output_absolute))
    throw std::runtime_error("EVIDENCE_OUTPUT_ALREADY_EXISTS_REFUSE_OVERWRITE");
  if (cli.phase == "level-a-run") return;

  std::ifstream stream(cli.eligibility_file);
  if (!stream) throw std::runtime_error("NEXT_PHASE_GATE_OPEN_FAILED");
  nlohmann::json gate;
  stream >> gate;
  const std::string prerequisite = cli.phase == "level-b-repeat"
      ? "hardware/v15_24e_ft/j2_pose_gravity_050_run.csv"
      : "hardware/v15_24e_ft/j2_pose_gravity_025_run.csv";
  const std::string prerequisite_absolute =
      std::string(kRepoRoot) + "/" + prerequisite;
  if (gate.at("schema").get<std::string>() !=
          "V15_24E_NEXT_PHASE_GATE_V1" ||
      !gate.at("eligible").get<bool>() ||
      gate.at("allowed_phase").get<std::string>() != cli.phase ||
      gate.at("prerequisite_csv_path").get<std::string>() != prerequisite ||
      gate.at("prerequisite_csv_sha256").get<std::string>() !=
          sha256_file(prerequisite_absolute) ||
      gate.at("anchor_sha256").get<std::string>() != sha256_file(kAnchorPath) ||
      gate.at("source_head").get<std::string>() != kSourceHead)
    throw std::runtime_error("NEXT_PHASE_GATE_AUTHORITY_MISMATCH");

  if (cli.phase == "level-b-run") {
    if (gate.at("reason").get<std::string>() !=
            "LEVEL_A_SAFE_BIDIRECTIONAL_IMPROVEMENT_POSITION_FAIL" ||
        gate.at("level_a_numeric_result").get<std::string>() != "FAIL" ||
        !gate.at("safety_and_sync_pass").get<bool>() ||
        !gate.at("gravity_sign_robust").get<bool>() ||
        !gate.at("improvement_vs_zero_and_fixed_both_directions").get<bool>() ||
        std::abs(gate.at("meaningful_improvement_threshold_deg").get<double>() -
                 0.05) > 1e-12)
      throw std::runtime_error("LEVEL_B_CONDITIONAL_ELIGIBILITY_NOT_PROVEN");
  } else {
    const std::string expected_reason = cli.phase == "level-a-repeat"
        ? "LEVEL_A_FIRST_RUN_PASS" : "LEVEL_B_FIRST_RUN_PASS";
    if (gate.at("reason").get<std::string>() != expected_reason ||
        gate.at("prerequisite_result").get<std::string>() != "PASS")
      throw std::runtime_error("EXACT_REPEAT_ELIGIBILITY_NOT_PROVEN");
  }
}

ModelAnchor load_hardware_preflight(const Cli& cli) {
  ModelAnchor anchor = load_model_anchor(cli.anchor_file);
  verify_phase_eligibility(cli);
  return anchor;
}

class PoseGravityModel {
 public:
  explicit PoseGravityModel(const ModelAnchor& anchor) : anchor_(anchor) {
    if (std::string(mj_versionString()) != "3.11.0")
      throw std::runtime_error("MUJOCO_VERSION_MISMATCH");
    char error[1024] = {};
    model_.reset(mj_loadXML(kModelPath, nullptr, error, sizeof(error)));
    if (!model_) throw std::runtime_error(std::string("MUJOCO_LOAD_FAILED:") + error);
    data_.reset(mj_makeData(model_.get()));
    if (!data_) throw std::runtime_error("MUJOCO_DATA_ALLOC_FAILED");
    if (model_->nq != 6 || model_->nv != 6)
      throw std::runtime_error("MUJOCO_DOF_CONTRACT_MISMATCH");
    const std::array<const char*, 5> names = {"J2", "J3", "J4", "J5", "J6"};
    for (std::size_t i = 0; i < names.size(); ++i) {
      const int id = mj_name2id(model_.get(), mjOBJ_JOINT, names[i]);
      if (id < 0) throw std::runtime_error("MUJOCO_JOINT_MISSING");
      qadr_[i] = model_->jnt_qposadr[id];
      dadr_[i] = model_->jnt_dofadr[id];
      if (anchor_.q[i] < model_->jnt_range[2 * id] - 1e-12 ||
          anchor_.q[i] > model_->jnt_range[2 * id + 1] + 1e-12)
        throw std::runtime_error("MODEL_ANCHOR_OUTSIDE_JOINT_RANGE");
    }
    model_->opt.gravity[0] = 0.0;
    model_->opt.gravity[1] = 0.0;
    model_->opt.gravity[2] = -9.81;
    const double check = evaluate(0.0);
    if (std::abs(check - anchor_.anchor_tau) > 1e-9)
      throw std::runtime_error("ANCHOR_GRAVITY_REPRODUCTION_MISMATCH");
    // Check all extrema of the one-sided staged route before Runner constructs
    // its serial-port member. The session anchor remains the model pose at B.
    (void)evaluate(kBoundaryToCenter);
    (void)evaluate(kCommandUpperBoundary);
  }

  double evaluate(double q_j2_relative) {
    mju_zero(data_->qpos, model_->nq);
    for (std::size_t i = 0; i < anchor_.q.size(); ++i)
      data_->qpos[qadr_[i]] = anchor_.q[i];
    data_->qpos[qadr_[0]] += q_j2_relative;
    mju_zero(data_->qvel, model_->nv);
    mju_zero(data_->qacc, model_->nv);
    mj_forward(model_.get(), data_.get());
    const double torque = data_->qfrc_bias[dadr_[0]];
    if (!std::isfinite(torque) || anchor_.gravity_sign * torque <= 0.0)
      throw std::runtime_error("RUNTIME_GRAVITY_SIGN_OR_FINITE_ABORT");
    return torque;
  }

  double model_q_j2(double relative) const { return anchor_.q[0] + relative; }
  const ModelAnchor& anchor() const { return anchor_; }

 private:
  struct ModelDeleter { void operator()(mjModel* value) const { mj_deleteModel(value); } };
  struct DataDeleter { void operator()(mjData* value) const { mj_deleteData(value); } };
  ModelAnchor anchor_;
  std::unique_ptr<mjModel, ModelDeleter> model_;
  std::unique_ptr<mjData, DataDeleter> data_;
  std::array<int, 5> qadr_{};
  std::array<int, 5> dadr_{};
};

class ProcessLock {
 public:
  ProcessLock() {
    fd_ = ::open(kLockPath, O_RDWR | O_CREAT | O_CLOEXEC, 0600);
    if (fd_ < 0 || ::flock(fd_, LOCK_EX | LOCK_NB) != 0)
      throw std::runtime_error("J2_CHANNEL1_LOCK_FAILED");
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

struct PairState {
  Feedback a;
  Feedback b;
  double raw_a = std::numeric_limits<double>::quiet_NaN();
  double raw_b = std::numeric_limits<double>::quiet_NaN();
  double q_a = std::numeric_limits<double>::quiet_NaN();
  double q_b = std::numeric_limits<double>::quiet_NaN();
  double q_j2 = std::numeric_limits<double>::quiet_NaN();
  double e_sync = std::numeric_limits<double>::quiet_NaN();
  double q_ref = std::numeric_limits<double>::quiet_NaN();
  double e_common = std::numeric_limits<double>::quiet_NaN();
  double tau_j2_fb = std::numeric_limits<double>::quiet_NaN();
  double dq_a_logical = std::numeric_limits<double>::quiet_NaN();
  double dq_b_logical = std::numeric_limits<double>::quiet_NaN();
  double alpha = 0.0;
  double tau_g_model = std::numeric_limits<double>::quiet_NaN();
  double q_j2_model = std::numeric_limits<double>::quiet_NaN();
  double tff_a_unclamped = 0.0;
  double tff_b_unclamped = 0.0;
  double tff_a_literal = 0.0;
  double tff_b_literal = 0.0;
  std::int16_t tff_a_count = 0;
  std::int16_t tff_b_count = 0;
  bool tff_a_clamped = false;
  bool tff_b_clamped = false;
  bool valid = false;
};

class Runner {
 public:
  explicit Runner(const Cli& cli)
      : cli_(cli), config_(level_config(cli)),
        anchor_(load_hardware_preflight(cli)), gravity_(anchor_), lock_(),
        serial_(kPort, 16, 4000000, 20000, BlockYN::NO,
                bytesize_t::eightbits, parity_t::parity_none,
                stopbits_t::stopbits_one, flowcontrol_t::flowcontrol_none),
        csv_(std::string(kRepoRoot) + "/" + cli.output,
             std::ios::out | std::ios::trunc) {
    if (!csv_) throw std::runtime_error("CSV_OPEN_FAILED");
    csv_ << "tick,timestamp_s,phase,level,alpha,a_mode,b_mode,a_q_cmd,b_q_cmd,"
            "a_dq_cmd,b_dq_cmd,tau_g_model_cmd_nm,qJ2_model_cmd_rad,"
            "a_tff_unclamped,b_tff_unclamped,a_tau_cmd_literal,b_tau_cmd_literal,"
            "a_tau_cmd_count,b_tau_cmd_count,a_tau_cmd_decoded,b_tau_cmd_decoded,"
            "a_tff_clamped,b_tff_clamped,tff_clamp_used,"
            "tau_J2_ff_cmd_decoded,"
            "a_raw,b_raw,qA_logical_rad,qB_logical_rad,"
            "qJ2_logical_rad,qJ2_pre_tff_baseline_rad,qJ2_tff_displacement_rad,"
            "q_ref_rad,e_common_rad,e_sync_rad,a_dq_feedback,b_dq_feedback,"
            "a_dq_logical,b_dq_logical,a_tau,b_tau,tau_J2_feedback,"
            "a_temp,b_temp,a_merror,b_merror,a_received_id,b_received_id,"
            "a_send_recv,b_send_recv,a_correct,b_correct,a_crc_ok,b_crc_ok,"
            "a_valid,b_valid,"
            "cycle_period_ms,cycle_jitter_ms\n";
    origin_ = Clock::now();
  }
  ~Runner() { if (!terminal_brake_done_) safe_brake(); }

  int run() {
    try {
      return run_dual_motion();
    } catch (...) {
      const bool brake = terminal_brake();
      std::cerr << "FINAL_DUAL_5_FRAME_BRAKE="
                << (brake ? "PASS" : "FAIL") << '\n';
      throw;
    }
  }

 private:
  Feedback transact(MotorCmd& command, int expected_id, int expected_mode) {
    MotorData data;
    initialize_feedback(data);
    Feedback f;
    try { f.send_recv = serial_.sendRecv(&command, &data); }
    catch (...) { f.send_recv = false; }
    f.correct = data.correct;
    f.id = static_cast<int>(data.motor_id);
    f.mode = static_cast<int>(data.mode);
    f.temp = data.temp;
    f.merror = data.merror;
    f.q = data.q;
    f.dq = data.dq;
    f.tau = data.tau;
    const std::uint8_t* raw = data.get_motor_recv_data();
    if (raw != nullptr) {
      f.crc_ok = raw[0] == 0xfdU && raw[1] == 0xeeU &&
          (raw[2] & 0x80U) == 0U &&
          load_u16_le(raw + 14) == crc16_kermit(raw, 14);
    }
    f.valid = f.send_recv && f.correct && f.crc_ok &&
        f.id == expected_id && f.mode == expected_mode && f.merror == 0 &&
        f.temp >= 0 && f.temp < kTempLimit && std::isfinite(f.q) &&
        std::isfinite(f.dq) && std::isfinite(f.tau);
    return f;
  }

  PairState transact_pair(int mode, double q_target, double dq_target,
                          double alpha,
                          const std::string& phase, Clock::time_point scheduled,
                          bool ignore_stop = false) {
    if (g_stop != 0 && !ignore_stop)
      throw std::runtime_error("OPERATOR_ABORT");
    if (mode == kFocMode && !reference_set_)
      throw std::runtime_error("REFERENCE_NOT_SET");
    if (!std::isfinite(q_target) || !std::isfinite(dq_target))
      throw std::runtime_error("NONFINITE_COMMAND_TARGET");
    if (q_target < kCommandLowerBoundary ||
        q_target > kCommandUpperBoundary)
      throw std::runtime_error("ASYMMETRIC_COMMAND_BOUNDARY");
    if (mode == kFocMode &&
        q_target <= kCommandLowerBoundary + 1e-12 && dq_target < -1e-12)
      throw std::runtime_error("OUTWARD_VELOCITY_AT_REAL_BOUNDARY_B");
    if (mode == kFocMode &&
        (!std::isfinite(alpha) || alpha < -1e-12 ||
         alpha > config_.alpha + 1e-12))
      throw std::runtime_error("GRAVITY_ALPHA_ENVELOPE");
    if (mode == kFocMode && phase == "ZERO_TFF_BASELINE_HOLD" &&
        std::abs(alpha) > 1e-12)
      throw std::runtime_error("ZERO_TFF_HOLD_ALPHA_NOT_ZERO");
    std::this_thread::sleep_until(scheduled);
    const Clock::time_point begin = Clock::now();
    if (g_stop != 0 && !ignore_stop)
      throw std::runtime_error("OPERATOR_ABORT");
    if (mode == kFocMode && begin > scheduled + std::chrono::milliseconds(2))
      throw std::runtime_error("CONTROL_DEADLINE_MISS_GT_2MS");

    PairState state;
    state.alpha = mode == kFocMode ? alpha : 0.0;
    const double model_relative = latest_q_j2_;
    if (mode == kFocMode) {
      state.tau_g_model = gravity_.evaluate(model_relative);
      state.q_j2_model = gravity_.model_q_j2(model_relative);
      state.tff_a_unclamped =
          static_cast<double>(kSignA) * alpha * state.tau_g_model / (2.0 * kGear);
      state.tff_b_unclamped =
          static_cast<double>(kSignB) * alpha * state.tau_g_model / (2.0 * kGear);
      state.tff_a_literal = std::clamp(
          state.tff_a_unclamped, -config_.cap, config_.cap);
      state.tff_b_literal = std::clamp(
          state.tff_b_unclamped, -config_.cap, config_.cap);
      state.tff_a_clamped =
          std::abs(state.tff_a_literal - state.tff_a_unclamped) > 1e-12;
      state.tff_b_clamped =
          std::abs(state.tff_b_literal - state.tff_b_unclamped) > 1e-12;
      tff_clamp_used_ = tff_clamp_used_ ||
          state.tff_a_clamped || state.tff_b_clamped;
    }

    const double a_q_cmd = mode == kFocMode
        ? ref_a_ + kSignA * kGear * q_target : 0.0;
    const double b_q_cmd = mode == kFocMode
        ? ref_b_ + kSignB * kGear * q_target : 0.0;
    const double a_dq_cmd = mode == kFocMode
        ? kSignA * kGear * dq_target : 0.0;
    const double b_dq_cmd = mode == kFocMode
        ? kSignB * kGear * dq_target : 0.0;
    MotorCmd command_a = mode == kFocMode
        ? make_command(kIdA, mode, a_q_cmd, a_dq_cmd, kKp, kKd,
                       state.tff_a_literal)
        : brake_command(kIdA);
    MotorCmd command_b = mode == kFocMode
        ? make_command(kIdB, mode, b_q_cmd, b_dq_cmd, kKp, kKd,
                       state.tff_b_literal)
        : brake_command(kIdB);
    state.tff_a_count = mode == kFocMode
        ? static_cast<std::int16_t>(load_u16_le(
              command_a.get_motor_send_data() + 3)) : 0;
    state.tff_b_count = mode == kFocMode
        ? static_cast<std::int16_t>(load_u16_le(
              command_b.get_motor_send_data() + 3)) : 0;
    const int cap_count = static_cast<int>(config_.cap * 256.0);
    if (mode == kFocMode &&
        (std::abs(static_cast<int>(state.tff_a_count)) > cap_count ||
         std::abs(static_cast<int>(state.tff_b_count)) > cap_count ||
         state.tff_a_count != -state.tff_b_count))
      throw std::runtime_error("TFF_ENCODED_ENVELOPE_OR_SYMMETRY");
    if (mode == kFocMode && phase == "ZERO_TFF_BASELINE_HOLD" &&
        (state.tff_a_count != 0 || state.tff_b_count != 0))
      throw std::runtime_error("ZERO_TFF_HOLD_COMMAND_NOT_ZERO");

    state.q_ref = mode == kFocMode ? q_target
                                   : std::numeric_limits<double>::quiet_NaN();
    state.a = transact(command_a, kIdA, mode);
    state.b = transact(command_b, kIdB, mode);
    update_invalid(state.a.valid, consecutive_invalid_a_, "J2A_FIVE_CONSECUTIVE_INVALID");
    update_invalid(state.b.valid, consecutive_invalid_b_, "J2B_FIVE_CONSECUTIVE_INVALID");
    if (state.a.valid && state.b.valid) {
      state.raw_a = unwrap_a_.update(state.a.q);
      state.raw_b = unwrap_b_.update(state.b.q);
      if (reference_set_) {
        state.q_a = kSignA * (state.raw_a - ref_a_) / kGear;
        state.q_b = kSignB * (state.raw_b - ref_b_) / kGear;
        state.q_j2 = 0.5 * (state.q_a + state.q_b);
        state.e_common = q_target - state.q_j2;
        state.e_sync = state.q_a - state.q_b;
        state.dq_a_logical = kSignA * state.a.dq / kGear;
        state.dq_b_logical = kSignB * state.b.dq / kGear;
        state.tau_j2_fb = kGear *
            (kSignA * state.a.tau + kSignB * state.b.tau);
        min_feedback_q_a_ = std::min(min_feedback_q_a_, state.q_a);
        min_feedback_q_b_ = std::min(min_feedback_q_b_, state.q_b);
        min_feedback_q_j2_ = std::min(min_feedback_q_j2_, state.q_j2);
        max_feedback_q_a_ = std::max(max_feedback_q_a_, state.q_a);
        max_feedback_q_b_ = std::max(max_feedback_q_b_, state.q_b);
        max_feedback_q_j2_ = std::max(max_feedback_q_j2_, state.q_j2);
        if (state.q_a < kFeedbackLowerBoundaryAbort ||
            state.q_b < kFeedbackLowerBoundaryAbort ||
            state.q_j2 < kFeedbackLowerBoundaryAbort)
          throw std::runtime_error(
              "REAL_BOUNDARY_B_FEEDBACK_UNDERSHOOT_GT_0P5DEG");
        if (state.q_a > kFeedbackUpperBoundaryAbort ||
            state.q_b > kFeedbackUpperBoundaryAbort ||
            state.q_j2 > kFeedbackUpperBoundaryAbort)
          throw std::runtime_error(
              "ONE_SIDED_FEEDBACK_UPPER_ENVELOPE_GT_12DEG");
        if (std::abs(state.dq_a_logical) > kFeedbackVelocityHardAbort ||
            std::abs(state.dq_b_logical) > kFeedbackVelocityHardAbort)
          throw std::runtime_error("UNEXPECTED_RAPID_MOTION_GT_30DEG_S");
      }
      state.valid = true;
      if (mode == kFocMode && reference_set_) {
        latest_q_j2_ = state.q_j2;
        max_logical_tau_feedback_ = std::max(
            max_logical_tau_feedback_, std::abs(state.tau_j2_fb));
      }
    }
    if (mode == kFocMode) {
      min_command_q_j2_ = std::min(min_command_q_j2_, q_target);
      max_command_q_j2_ = std::max(max_command_q_j2_, q_target);
      model_torque_min_ = std::min(model_torque_min_, state.tau_g_model);
      model_torque_max_ = std::max(model_torque_max_, state.tau_g_model);
      tff_a_min_ = std::min(tff_a_min_,
                            static_cast<double>(state.tff_a_count) / 256.0);
      tff_a_max_ = std::max(tff_a_max_,
                            static_cast<double>(state.tff_a_count) / 256.0);
      tff_b_min_ = std::min(tff_b_min_,
                            static_cast<double>(state.tff_b_count) / 256.0);
      tff_b_max_ = std::max(tff_b_max_,
                            static_cast<double>(state.tff_b_count) / 256.0);
    }
    log_row(state, phase, a_q_cmd, b_q_cmd, a_dq_cmd, b_dq_cmd, begin);
    ++tick_;
    if (mode == kFocMode &&
        (state.a.merror != 0 || state.b.merror != 0))
      throw std::runtime_error("MERROR_NONZERO_ACTIVE");
    if (mode == kFocMode && (!state.a.valid || !state.b.valid))
      throw std::runtime_error("ACTIVE_FEEDBACK_LOSS_OR_INVALID");
    if (state.valid && reference_set_ &&
        std::abs(state.e_sync) > kSyncHardAbort)
      throw std::runtime_error("SYNC_HARD_ABORT_GT_1DEG");
    return state;
  }

  static void update_invalid(bool valid, int& consecutive, const char* reason) {
    if (valid) consecutive = 0;
    else if (++consecutive >= 5) throw std::runtime_error(reason);
  }

  std::pair<std::vector<double>, std::vector<double>> capture_references() {
    std::vector<double> a_values;
    std::vector<double> b_values;
    a_values.reserve(50U);
    b_values.reserve(50U);
    Clock::time_point next = Clock::now();
    for (int frame = 0; frame < 50; ++frame) {
      const PairState state = transact_pair(kBrakeMode, 0.0, 0.0,
                                            0.0,
                                            "SESSION_BRAKE_CAPTURE", next);
      if (state.valid) {
        a_values.push_back(state.raw_a);
        b_values.push_back(state.raw_b);
      }
      next += std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(kPeriod));
      if (next < Clock::now()) next = Clock::now();
    }
    if (a_values.size() != 50U || b_values.size() != 50U)
      throw std::runtime_error("DUAL_SESSION_NOT_50_VALID_EACH");
    return {a_values, b_values};
  }

  void establish_references(const std::vector<double>& a_values,
                            const std::vector<double>& b_values) {
    ref_a_ = median(a_values);
    ref_b_ = median(b_values);
    reference_set_ = true;
  }

  std::vector<PairState> run_target_hold(const std::string& phase,
                                         double target, double duration,
                                         bool collect_motion_sync,
                                         double alpha,
                                         double unexpected_motion_reference =
                                             std::numeric_limits<double>::quiet_NaN()) {
    const int count = static_cast<int>(std::ceil(duration * kHz));
    std::vector<PairState> values;
    values.reserve(static_cast<std::size_t>(count));
    Clock::time_point next = active_started_ ? next_active_ : Clock::now();
    active_started_ = true;
    for (int frame = 0; frame < count; ++frame) {
      PairState state = transact_pair(kFocMode, target, 0.0,
                                      alpha, phase, next);
      if (state.valid) {
        values.push_back(state);
        if (std::isfinite(unexpected_motion_reference) &&
            std::abs(state.q_j2 - unexpected_motion_reference) >
                kGravityUnexpectedMotionAbort)
          throw std::runtime_error("GRAVITY_UNEXPECTED_MOTION_GT_1P5DEG");
        if (collect_motion_sync)
          max_motion_sync_ = std::max(max_motion_sync_, std::abs(state.e_sync));
        if (collect_motion_sync)
          max_common_error_ = std::max(max_common_error_, std::abs(state.e_common));
      }
      next += std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(kPeriod));
    }
    next_active_ = next;
    return values;
  }

  void run_profile(const std::string& phase, double start, double displacement,
                   double alpha) {
    Trapezoid profile(displacement);
    const int count = static_cast<int>(std::ceil(profile.duration() * kHz)) + 1;
    Clock::time_point next = active_started_ ? next_active_ : Clock::now();
    active_started_ = true;
    for (int frame = 0; frame < count; ++frame) {
      const double t = std::min(frame * kPeriod, profile.duration());
      const ProfilePoint point = profile.sample(t);
      const PairState state = transact_pair(kFocMode, start + point.q, point.dq,
                                            alpha, phase, next);
      if (state.valid)
        max_motion_sync_ = std::max(max_motion_sync_, std::abs(state.e_sync));
      if (state.valid)
        max_common_error_ = std::max(max_common_error_, std::abs(state.e_common));
      next += std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(kPeriod));
    }
    next_active_ = next;
  }

  bool terminal_brake() {
    bool pass = true;
    Clock::time_point next = Clock::now();
    for (int frame = 0; frame < 5; ++frame) {
      try {
        const PairState state = transact_pair(kBrakeMode, 0.0, 0.0,
                                              0.0,
                                              "FINAL_DUAL_BRAKE", next, true);
        pass = pass && state.a.valid && state.b.valid;
      } catch (...) {
        pass = false;
      }
      next += std::chrono::milliseconds(10);
    }
    final_brake_pass_ = pass;
    csv_.flush();
    if (pass) terminal_brake_done_ = true;
    else safe_brake();
    return pass;
  }

  void safe_brake() noexcept {
    try {
      for (int frame = 0; frame < 20; ++frame) {
        MotorCmd a = brake_command(kIdA);
        MotorCmd b = brake_command(kIdB);
        (void)transact(a, kIdA, kBrakeMode);
        (void)transact(b, kIdB, kBrakeMode);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      terminal_brake_done_ = true;
    } catch (...) {}
  }

  void log_row(const PairState& state, const std::string& phase,
               double a_q_cmd, double b_q_cmd,
               double a_dq_cmd, double b_dq_cmd,
               Clock::time_point begin) {
    const double now_s = std::chrono::duration<double>(begin - origin_).count();
    double period_ms = std::numeric_limits<double>::quiet_NaN();
    double jitter_ms = std::numeric_limits<double>::quiet_NaN();
    if (have_previous_) {
      period_ms = (now_s - previous_s_) * 1000.0;
      jitter_ms = std::abs(period_ms - 10.0);
    }
    previous_s_ = now_s;
    have_previous_ = true;
    csv_ << tick_ << ',' << std::setprecision(17) << now_s << ',' << phase << ','
         << config_.label << ',' << state.alpha << ','
         << state.a.mode << ',' << state.b.mode << ','
         << a_q_cmd << ',' << b_q_cmd << ',' << a_dq_cmd << ',' << b_dq_cmd << ','
         << state.tau_g_model << ',' << state.q_j2_model << ','
         << state.tff_a_unclamped << ',' << state.tff_b_unclamped << ','
         << state.tff_a_literal << ',' << state.tff_b_literal << ','
         << state.tff_a_count << ',' << state.tff_b_count << ','
         << static_cast<double>(state.tff_a_count) / 256.0 << ','
         << static_cast<double>(state.tff_b_count) / 256.0 << ','
         << static_cast<int>(state.tff_a_clamped) << ','
         << static_cast<int>(state.tff_b_clamped) << ','
         << static_cast<int>(state.tff_a_clamped || state.tff_b_clamped) << ','
         << kGear * (kSignA * static_cast<double>(state.tff_a_count) / 256.0 +
                     kSignB * static_cast<double>(state.tff_b_count) / 256.0) << ','
         << state.raw_a << ',' << state.raw_b << ',' << state.q_a << ',' << state.q_b
         << ',' << state.q_j2 << ','
         << (gravity_baseline_set_ ? gravity_baseline_q_
                                       : std::numeric_limits<double>::quiet_NaN()) << ','
         << (gravity_baseline_set_ ? state.q_j2 - gravity_baseline_q_
                                       : std::numeric_limits<double>::quiet_NaN()) << ','
         << state.q_ref << ',' << state.e_common
         << ',' << state.e_sync << ','
         << state.a.dq << ',' << state.b.dq << ','
         << state.dq_a_logical << ',' << state.dq_b_logical << ','
         << state.a.tau << ',' << state.b.tau << ',' << state.tau_j2_fb
         << ',' << state.a.temp << ',' << state.b.temp << ','
         << state.a.merror << ',' << state.b.merror << ','
         << state.a.id << ',' << state.b.id << ','
         << static_cast<int>(state.a.send_recv) << ','
         << static_cast<int>(state.b.send_recv) << ','
         << static_cast<int>(state.a.correct) << ','
         << static_cast<int>(state.b.correct) << ','
         << static_cast<int>(state.a.crc_ok) << ','
         << static_cast<int>(state.b.crc_ok) << ','
         << static_cast<int>(state.a.valid) << ',' << static_cast<int>(state.b.valid)
         << ',' << period_ms << ',' << jitter_ms << '\n';
    csv_.flush();
  }

  static std::vector<double> tail_field(const std::vector<PairState>& values,
                                        bool sync, std::size_t count) {
    if (values.size() < count) throw std::runtime_error("ENDPOINT_FEEDBACK_INSUFFICIENT");
    std::vector<double> result;
    result.reserve(count);
    for (auto it = values.end() - static_cast<std::ptrdiff_t>(count);
         it != values.end(); ++it)
      result.push_back(sync ? it->e_sync : it->q_j2);
    return result;
  }

  std::vector<PairState> run_gravity_ramp(double baseline_q) {
    const int intervals = static_cast<int>(
        std::lround(config_.ramp_duration * kHz));
    std::vector<PairState> values;
    values.reserve(static_cast<std::size_t>(intervals + 1));
    Clock::time_point next = active_started_ ? next_active_ : Clock::now();
    active_started_ = true;
    for (int frame = 0; frame <= intervals; ++frame) {
      const double alpha = config_.alpha * smoothstep5(
          static_cast<double>(frame) / intervals);
      PairState state = transact_pair(kFocMode, 0.0, 0.0,
                                      alpha, "AUTO_GRAVITY_RAMP", next);
      if (state.valid) {
        values.push_back(state);
        max_ramp_sync_ = std::max(max_ramp_sync_, std::abs(state.e_sync));
        max_gravity_displacement_ = std::max(
            max_gravity_displacement_, std::abs(state.q_j2 - baseline_q));
        if (std::abs(state.q_j2 - baseline_q) >
            kGravityUnexpectedMotionAbort)
          throw std::runtime_error("GRAVITY_UNEXPECTED_MOTION_GT_1P5DEG");
      }
      next += std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(kPeriod));
    }
    next_active_ = next;
    return values;
  }

  int run_dual_motion() {
    const auto capture = capture_references();
    establish_references(capture.first, capture.second);
    const auto prehold = run_target_hold("ZERO_TFF_BASELINE_HOLD", 0.0,
                                         kZeroHoldDuration, false, 0.0);
    if (prehold.size() != static_cast<std::size_t>(
            std::ceil(kZeroHoldDuration * kHz)))
      throw std::runtime_error("ZERO_TFF_HOLD_FEEDBACK_INSUFFICIENT");
    const double baseline_q = median(tail_field(prehold, false, 30U));
    const double prehold_sync = std::abs(median(tail_field(prehold, true, 30U)));
    double prehold_max_sync = 0.0;
    double prehold_max_motion = 0.0;
    for (const PairState& state : prehold) {
      prehold_max_sync = std::max(prehold_max_sync, std::abs(state.e_sync));
      prehold_max_motion = std::max(prehold_max_motion, std::abs(state.q_j2));
    }
    const bool zero_hold_pass = prehold_sync <= 0.3 * kPi / 180.0 &&
        prehold_max_sync <= 0.3 * kPi / 180.0 &&
        prehold_max_motion <= 1.0 * kPi / 180.0;
    if (!zero_hold_pass) {
      const bool brake = terminal_brake();
      std::cout << std::setprecision(17)
                << "RUN=" << cli_.phase << '\n'
                << "DUAL_SESSION_A0_RAW_RAD=" << ref_a_ << '\n'
                << "DUAL_SESSION_B0_RAW_RAD=" << ref_b_ << '\n'
                << "ZERO_TFF_BASELINE_QJ2_DEG=" << baseline_q * 180.0 / kPi << '\n'
                << "ZERO_TFF_HOLD_FINAL_ESYNC_DEG=" << prehold_sync * 180.0 / kPi << '\n'
                << "ZERO_TFF_HOLD_MAX_ESYNC_DEG=" << prehold_max_sync * 180.0 / kPi << '\n'
                << "ZERO_TFF_STARTUP_HOLD=FAIL\n"
                << "FINAL_DUAL_5_FRAME_BRAKE=" << (brake ? "PASS" : "FAIL") << '\n';
      return 2;
    }

    gravity_baseline_q_ = baseline_q;
    gravity_baseline_set_ = true;
    const auto ramp = run_gravity_ramp(baseline_q);
    if (ramp.size() != static_cast<std::size_t>(
            std::lround(config_.ramp_duration * kHz) + 1))
      throw std::runtime_error("GRAVITY_RAMP_FEEDBACK_INSUFFICIENT");
    const auto gravity_hold = run_target_hold("AUTO_GRAVITY_HOLD", 0.0,
                                              kGravityHoldDuration, false,
                                              config_.alpha, baseline_q);
    if (gravity_hold.size() != static_cast<std::size_t>(
            std::ceil(kGravityHoldDuration * kHz)))
      throw std::runtime_error("GRAVITY_HOLD_FEEDBACK_INSUFFICIENT");
    double gravity_hold_max_sync = 0.0;
    for (const PairState& state : gravity_hold) {
      gravity_hold_max_sync = std::max(
          gravity_hold_max_sync, std::abs(state.e_sync));
      max_gravity_displacement_ = std::max(
          max_gravity_displacement_, std::abs(state.q_j2 - baseline_q));
    }
    const double gravity_hold_q = median(tail_field(gravity_hold, false, 50U));
    const double gravity_hold_displacement = gravity_hold_q - baseline_q;
    const bool gravity_hold_pass =
        gravity_hold_max_sync <= 0.3 * kPi / 180.0 &&
        max_gravity_displacement_ <= kGravityUnexpectedMotionAbort;
    if (!gravity_hold_pass) {
      const bool brake = terminal_brake();
      std::cout << std::setprecision(17)
                << "RUN=" << cli_.phase << '\n'
                << "DUAL_SESSION_A0_RAW_RAD=" << ref_a_ << '\n'
                << "DUAL_SESSION_B0_RAW_RAD=" << ref_b_ << '\n'
                << "ZERO_TFF_STARTUP_HOLD=PASS\n"
                << "GRAVITY_ALPHA=" << config_.alpha << '\n'
                << "GRAVITY_RAMP_DURATION_S=" << config_.ramp_duration << '\n'
                << "GRAVITY_HOLD_DISPLACEMENT_DEG="
                << gravity_hold_displacement * 180.0 / kPi << '\n'
                << "GRAVITY_HOLD_MAX_ESYNC_DEG="
                << gravity_hold_max_sync * 180.0 / kPi << '\n'
                << "GRAVITY_HOLD_NUMERIC_RESULT=FAIL\n"
                << "FINAL_DUAL_5_FRAME_BRAKE=" << (brake ? "PASS" : "FAIL") << '\n';
      return 2;
    }

    max_motion_sync_ = 0.0;
    max_common_error_ = 0.0;
    run_profile("MOVE_TO_TEST_CENTER_PROFILE", 0.0, kBoundaryToCenter,
                config_.alpha);
    const auto staged_center = run_target_hold(
        "MOVE_TO_TEST_CENTER_ENDPOINT", kBoundaryToCenter,
        0.4, true, config_.alpha);
    const double staged_center_actual =
        median(tail_field(staged_center, false, 30U));
    const double staged_center_error =
        std::abs(staged_center_actual - kBoundaryToCenter);
    const double staged_center_sync =
        std::abs(median(tail_field(staged_center, true, 30U)));

    // The original four center-relative route segments are retained, while
    // every controller target is expressed relative to the measured boundary
    // B. Thus C-relative +5/0/-5/0 are absolute B-relative 10/5/0/5 deg.
    run_profile("PLUS_5_PROFILE", kBoundaryToCenter, kCenterExcursion,
                config_.alpha);
    const auto plus = run_target_hold(
        "PLUS_5_ENDPOINT", kCommandUpperBoundary,
        0.4, true, config_.alpha);
    const double plus_absolute = median(tail_field(plus, false, 30U));
    const double plus_actual = plus_absolute - kBoundaryToCenter;
    const double plus_sync = std::abs(median(tail_field(plus, true, 30U)));

    run_profile("FIRST_CENTER_PROFILE", kCommandUpperBoundary,
                -kCenterExcursion, config_.alpha);
    const auto first = run_target_hold(
        "FIRST_CENTER_ENDPOINT", kBoundaryToCenter, 0.4,
        true, config_.alpha);
    const double first_absolute = median(tail_field(first, false, 30U));
    const double first_error = std::abs(first_absolute - kBoundaryToCenter);

    run_profile("MINUS_5_PROFILE", kBoundaryToCenter, -kCenterExcursion,
                config_.alpha);
    const auto minus = run_target_hold(
        "MINUS_5_ENDPOINT", kCommandLowerBoundary,
        0.4, true, config_.alpha);
    const double minus_absolute = median(tail_field(minus, false, 30U));
    const double minus_actual = minus_absolute - kBoundaryToCenter;
    const double minus_sync = std::abs(median(tail_field(minus, true, 30U)));

    run_profile("FINAL_CENTER_PROFILE", kCommandLowerBoundary,
                kCenterExcursion, config_.alpha);
    const auto final = run_target_hold(
        "FINAL_CENTER_ENDPOINT", kBoundaryToCenter, 0.5,
        true, config_.alpha);
    const double final_absolute = median(tail_field(final, false, 40U));
    const double final_error = std::abs(final_absolute - kBoundaryToCenter);

    run_profile("RETURN_TO_BOUNDARY_PROFILE", kBoundaryToCenter,
                -kBoundaryToCenter, config_.alpha);
    const auto boundary_return = run_target_hold(
        "RETURN_TO_BOUNDARY_ENDPOINT", kCommandLowerBoundary, 0.5,
        true, config_.alpha);
    const double boundary_return_actual =
        median(tail_field(boundary_return, false, 40U));
    const double boundary_return_error =
        std::abs(boundary_return_actual - kCommandLowerBoundary);
    const double boundary_return_sync =
        std::abs(median(tail_field(boundary_return, true, 40U)));

    const double plus_error = std::abs(plus_actual - kCenterExcursion);
    const double minus_error = std::abs(minus_actual + kCenterExcursion);
    const bool pass =
        staged_center_error <= 0.75 * kPi / 180.0 &&
        staged_center_sync <= 0.3 * kPi / 180.0 &&
        plus_error <= 1.0 * kPi / 180.0 &&
        first_error <= 0.75 * kPi / 180.0 &&
        minus_error <= 1.0 * kPi / 180.0 &&
        final_error <= 0.75 * kPi / 180.0 &&
        boundary_return_error <= 0.75 * kPi / 180.0 &&
        boundary_return_sync <= 0.3 * kPi / 180.0 &&
        plus_sync <= 0.3 * kPi / 180.0 &&
        minus_sync <= 0.3 * kPi / 180.0 &&
        max_motion_sync_ <= 0.7 * kPi / 180.0;
    const bool brake = terminal_brake();
    std::cout << std::setprecision(17)
              << "RUN=" << cli_.phase << '\n'
              << "DUAL_SESSION_A0_RAW_RAD=" << ref_a_ << '\n'
              << "DUAL_SESSION_B0_RAW_RAD=" << ref_b_ << '\n'
              << "REAL_BOUNDARY_B_J2A_RAW_RAD=" << ref_a_ << '\n'
              << "REAL_BOUNDARY_B_J2B_RAW_RAD=" << ref_b_ << '\n'
              << "ZERO_TFF_STARTUP_HOLD=PASS\n"
              << "ZERO_TFF_BASELINE_QJ2_DEG=" << baseline_q * 180.0 / kPi << '\n'
              << "ZERO_TFF_HOLD_FINAL_ESYNC_DEG=" << prehold_sync * 180.0 / kPi << '\n'
              << "ZERO_TFF_HOLD_MAX_ESYNC_DEG=" << prehold_max_sync * 180.0 / kPi << '\n'
              << "GRAVITY_LEVEL=" << config_.label << '\n'
              << "GRAVITY_ALPHA=" << config_.alpha << '\n'
              << "TFF_CAP_PER_MOTOR_NM=" << config_.cap << '\n'
              << "GRAVITY_RAMP_DURATION_S=" << config_.ramp_duration << '\n'
              << "GRAVITY_RAMP_MAX_ESYNC_DEG=" << max_ramp_sync_ * 180.0 / kPi << '\n'
              << "GRAVITY_HOLD_DISPLACEMENT_DEG="
              << gravity_hold_displacement * 180.0 / kPi << '\n'
              << "GRAVITY_HOLD_MAX_ESYNC_DEG="
              << gravity_hold_max_sync * 180.0 / kPi << '\n'
              << "GRAVITY_HOLD_MAX_ABS_DISPLACEMENT_DEG="
              << max_gravity_displacement_ * 180.0 / kPi << '\n'
              << "GRAVITY_HOLD_NUMERIC_RESULT=PASS\n"
              << "MODEL_J2_TORQUE_MIN_NM=" << model_torque_min_ << '\n'
              << "MODEL_J2_TORQUE_MAX_NM=" << model_torque_max_ << '\n'
              << "J2A_TFF_DECODED_MIN_NM=" << tff_a_min_ << '\n'
              << "J2A_TFF_DECODED_MAX_NM=" << tff_a_max_ << '\n'
              << "J2B_TFF_DECODED_MIN_NM=" << tff_b_min_ << '\n'
              << "J2B_TFF_DECODED_MAX_NM=" << tff_b_max_ << '\n'
              << "TFF_CLAMP_USED=" << (tff_clamp_used_ ? "YES" : "NO") << '\n'
              << "REAL_BOUNDARY_B_CAPTURED_FROM_STARTUP_BRAKE=YES\n"
              << "BOUNDARY_ROUTE_405_AUDIT_VERIFIED=YES\n"
              << "AUDITED_J2_B_RELATIVE_OFFSETS_DEG=0,2.5,5,7.5,10\n"
              << "TEST_CENTER_C_RELATIVE_TO_B_DEG=5\n"
              << "NO_ADDITIONAL_BOUNDARY_MARGIN=YES\n"
              << "ACTUAL_NO_BOUNDARY_CROSSING_ABSOLUTE_GUARANTEE=NO\n"
              << "COMMAND_ROUTE_B_RELATIVE_DEG=0_TO_5_TO_10_TO_5_TO_0_TO_5_TO_0\n"
              << "DUAL_ROUTE_AT_C=0_TO_PLUS5_TO_0_TO_MINUS5_TO_0\n"
              << "MOVE_TO_TEST_CENTER_ACTUAL_B_RELATIVE_DEG="
              << staged_center_actual * 180.0 / kPi << '\n'
              << "MOVE_TO_TEST_CENTER_ERROR_DEG="
              << staged_center_error * 180.0 / kPi << '\n'
              << "MOVE_TO_TEST_CENTER_ESYNC_DEG="
              << staged_center_sync * 180.0 / kPi << '\n'
              << "PLUS_5_LOGICAL_ACTUAL_DEG=" << plus_actual * 180.0 / kPi << '\n'
              << "PLUS_5_ABSOLUTE_B_RELATIVE_DEG="
              << plus_absolute * 180.0 / kPi << '\n'
              << "PLUS_5_LOGICAL_ERROR_DEG=" << plus_error * 180.0 / kPi << '\n'
              << "PLUS_5_ESYNC_DEG=" << plus_sync * 180.0 / kPi << '\n'
              << "FIRST_CENTER_ERROR_DEG=" << first_error * 180.0 / kPi << '\n'
              << "MINUS_5_LOGICAL_ACTUAL_DEG=" << minus_actual * 180.0 / kPi << '\n'
              << "MINUS_5_ABSOLUTE_B_RELATIVE_DEG="
              << minus_absolute * 180.0 / kPi << '\n'
              << "MINUS_5_LOGICAL_ERROR_DEG=" << minus_error * 180.0 / kPi << '\n'
              << "MINUS_5_ESYNC_DEG=" << minus_sync * 180.0 / kPi << '\n'
              << "FINAL_CENTER_ERROR_DEG=" << final_error * 180.0 / kPi << '\n'
              << "RETURN_TO_BOUNDARY_ACTUAL_B_RELATIVE_DEG="
              << boundary_return_actual * 180.0 / kPi << '\n'
              << "RETURN_TO_BOUNDARY_ERROR_DEG="
              << boundary_return_error * 180.0 / kPi << '\n'
              << "RETURN_TO_BOUNDARY_ESYNC_DEG="
              << boundary_return_sync * 180.0 / kPi << '\n'
              << "MIN_COMMAND_B_RELATIVE_DEG="
              << min_command_q_j2_ * 180.0 / kPi << '\n'
              << "MAX_COMMAND_B_RELATIVE_DEG="
              << max_command_q_j2_ * 180.0 / kPi << '\n'
              << "MIN_QA_FEEDBACK_B_RELATIVE_DEG="
              << min_feedback_q_a_ * 180.0 / kPi << '\n'
              << "MIN_QB_FEEDBACK_B_RELATIVE_DEG="
              << min_feedback_q_b_ * 180.0 / kPi << '\n'
              << "MIN_QJ2_FEEDBACK_B_RELATIVE_DEG="
              << min_feedback_q_j2_ * 180.0 / kPi << '\n'
              << "MAX_QA_FEEDBACK_B_RELATIVE_DEG="
              << max_feedback_q_a_ * 180.0 / kPi << '\n'
              << "MAX_QB_FEEDBACK_B_RELATIVE_DEG="
              << max_feedback_q_b_ * 180.0 / kPi << '\n'
              << "MAX_QJ2_FEEDBACK_B_RELATIVE_DEG="
              << max_feedback_q_j2_ * 180.0 / kPi << '\n'
              << "MODEL_QJ2_SOURCE=B_MODEL_ANCHOR_PLUS_ACTUAL_B_RELATIVE_FEEDBACK\n"
              << "MAX_MOTION_ESYNC_DEG=" << max_motion_sync_ * 180.0 / kPi << '\n'
              << "MAX_COMMON_MODE_ERROR_DEG=" << max_common_error_ * 180.0 / kPi << '\n'
              << "MAX_LOGICAL_PAIRED_TAU_FEEDBACK_NM="
              << max_logical_tau_feedback_ << '\n'
              << "DUAL_MOTION_NUMERIC_RESULT=" << (pass ? "PASS" : "FAIL") << '\n'
              << "FINAL_DUAL_5_FRAME_BRAKE=" << (brake ? "PASS" : "FAIL") << '\n';
    return pass && brake ? 0 : 2;
  }

  const Cli& cli_;
  const LevelConfig config_;
  const ModelAnchor anchor_;
  PoseGravityModel gravity_;
  ProcessLock lock_;
  SerialPort serial_;
  std::ofstream csv_;
  Clock::time_point origin_;
  Unwrapper unwrap_a_;
  Unwrapper unwrap_b_;
  double ref_a_ = 0.0;
  double ref_b_ = 0.0;
  bool reference_set_ = false;
  int consecutive_invalid_a_ = 0;
  int consecutive_invalid_b_ = 0;
  bool terminal_brake_done_ = false;
  bool final_brake_pass_ = false;
  bool have_previous_ = false;
  double previous_s_ = 0.0;
  std::uint64_t tick_ = 0;
  bool active_started_ = false;
  Clock::time_point next_active_{};
  double max_motion_sync_ = 0.0;
  double max_common_error_ = 0.0;
  double max_ramp_sync_ = 0.0;
  double max_gravity_displacement_ = 0.0;
  double max_logical_tau_feedback_ = 0.0;
  double gravity_baseline_q_ = 0.0;
  bool gravity_baseline_set_ = false;
  double latest_q_j2_ = 0.0;
  double min_command_q_j2_ = std::numeric_limits<double>::infinity();
  double max_command_q_j2_ = -std::numeric_limits<double>::infinity();
  double min_feedback_q_a_ = std::numeric_limits<double>::infinity();
  double min_feedback_q_b_ = std::numeric_limits<double>::infinity();
  double min_feedback_q_j2_ = std::numeric_limits<double>::infinity();
  double max_feedback_q_a_ = -std::numeric_limits<double>::infinity();
  double max_feedback_q_b_ = -std::numeric_limits<double>::infinity();
  double max_feedback_q_j2_ = -std::numeric_limits<double>::infinity();
  bool tff_clamp_used_ = false;
  double model_torque_min_ = std::numeric_limits<double>::infinity();
  double model_torque_max_ = -std::numeric_limits<double>::infinity();
  double tff_a_min_ = std::numeric_limits<double>::infinity();
  double tff_a_max_ = -std::numeric_limits<double>::infinity();
  double tff_b_min_ = std::numeric_limits<double>::infinity();
  double tff_b_max_ = -std::numeric_limits<double>::infinity();
};

void anchor_self_test(const std::string& path) {
  const ModelAnchor anchor = load_model_anchor(path);
  PoseGravityModel gravity(anchor);
  const double boundary = gravity.evaluate(kCommandLowerBoundary);
  const double center = gravity.evaluate(kBoundaryToCenter);
  const double upper = gravity.evaluate(kCommandUpperBoundary);
  if (anchor.gravity_sign * boundary <= 0.0 ||
      anchor.gravity_sign * center <= 0.0 ||
      anchor.gravity_sign * upper <= 0.0)
    throw std::runtime_error("ANCHOR_ROUTE_SIGN_SELF_TEST_FAILED");
  std::cout << std::setprecision(17)
            << "V15_24E_MODEL_ANCHOR_SELF_TEST=PASS\n"
            << "SERIAL_PORT_CONSTRUCTED=NO\n"
            << "GRAVITY_SIGN_ROBUST=YES\n"
            << "BOUNDARY_ROUTE_405_AUDIT_VERIFIED=YES\n"
            << "AUDITED_J2_B_RELATIVE_OFFSETS_DEG=0,2.5,5,7.5,10\n"
            << "REAL_BOUNDARY_B_TAU_G_J2_NM=" << boundary << '\n'
            << "TEST_CENTER_C_B_PLUS5_TAU_G_J2_NM=" << center << '\n'
            << "ROUTE_UPPER_B_PLUS10_TAU_G_J2_NM=" << upper << '\n'
            << "MODEL_QJ2_SOURCE=B_MODEL_ANCHOR_PLUS_ACTUAL_B_RELATIVE_FEEDBACK\n"
            << "COMMAND_ROUTE_B_RELATIVE_DEG=0_TO_5_TO_10_TO_5_TO_0_TO_5_TO_0\n"
            << "NO_ADDITIONAL_BOUNDARY_MARGIN=YES\n"
            << "UNCERTAINTY_MIN_NM=" << anchor.envelope_min << '\n'
            << "UNCERTAINTY_MAX_NM=" << anchor.envelope_max << '\n';
  const std::array<const char*, 5> names = {"J2", "J3", "J4", "J5", "J6"};
  for (std::size_t i = 0; i < names.size(); ++i)
    std::cout << "Q_ANCHOR_MODEL_" << names[i] << "_RAD=" << anchor.q[i] << '\n';
}

void self_test() {
  if (queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE) != kBrakeMode ||
      queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC) != kFocMode ||
      std::abs(queryGearRatio(MotorType::GO_M8010_6) - kGear) > 1e-6)
    throw std::runtime_error("SDK_AUTHORITY_MISMATCH");
  MotorCmd brake_a = brake_command(kIdA);
  MotorCmd brake_b = brake_command(kIdB);
  ModelAnchor zero_anchor;
  zero_anchor.q.fill(0.0);
  zero_anchor.anchor_tau = 13.493267323925703;
  zero_anchor.envelope_min = 13.0;
  zero_anchor.envelope_max = 14.0;
  zero_anchor.gravity_sign = 1;
  zero_anchor.uncertainty_samples = 1;
  PoseGravityModel gravity(zero_anchor);
  const double tau_zero = gravity.evaluate(kCommandLowerBoundary);
  const double tau_center = gravity.evaluate(kBoundaryToCenter);
  const double tau_upper = gravity.evaluate(kCommandUpperBoundary);
  if (!(tau_zero > 0.0 && tau_center > 0.0 && tau_upper > 0.0))
    throw std::runtime_error("POSE_GRAVITY_SIGN_SELF_TEST_FAILED");
  constexpr int kBenchmarkSamples = 1000;
  double benchmark_checksum = 0.0;
  const Clock::time_point benchmark_begin = Clock::now();
  for (int i = 0; i < kBenchmarkSamples; ++i) {
    const double phase = static_cast<double>(i) /
        static_cast<double>(kBenchmarkSamples - 1);
    benchmark_checksum += gravity.evaluate(
        phase * kCommandUpperBoundary);
  }
  const double benchmark_average_us =
      std::chrono::duration<double, std::micro>(
          Clock::now() - benchmark_begin).count() / kBenchmarkSamples;
  if (!std::isfinite(benchmark_checksum) || benchmark_average_us > 2000.0)
    throw std::runtime_error("MUJOCO_FORWARD_100HZ_BUDGET_SELF_TEST_FAILED");
  const double tff_a = kSignA * 0.25 * tau_zero / (2.0 * kGear);
  const double tff_b = kSignB * 0.25 * tau_zero / (2.0 * kGear);
  MotorCmd foc_a = make_command(kIdA, kFocMode, 1.0, -0.2,
                                kKp, kKd, tff_a);
  MotorCmd foc_b = make_command(kIdB, kFocMode, 2.0, 0.2,
                                kKp, kKd, tff_b);
  const std::uint8_t* pa = foc_a.get_motor_send_data();
  const std::uint8_t* pb = foc_b.get_motor_send_data();
  if (pa == nullptr || pb == nullptr ||
      pa[2] != static_cast<std::uint8_t>(kIdA | (kFocMode << 4)) ||
      pb[2] != static_cast<std::uint8_t>(kIdB | (kFocMode << 4)) ||
      static_cast<std::int16_t>(load_u16_le(pa + 3)) !=
          -static_cast<std::int16_t>(load_u16_le(pb + 3)) ||
      load_u16_le(pa + 11) != 768U || load_u16_le(pb + 11) != 768U ||
      load_u16_le(pa + 13) != 64U || load_u16_le(pb + 13) != 64U)
    throw std::runtime_error("DUAL_PACKET_SELF_TEST_FAILED");
  (void)brake_a;
  (void)brake_b;
  Trapezoid plus(5.0 * kPi / 180.0);
  Trapezoid minus(-5.0 * kPi / 180.0);
  if (std::abs(plus.sample(plus.duration()).q - 5.0 * kPi / 180.0) > 1e-12 ||
      std::abs(minus.sample(minus.duration()).q + 5.0 * kPi / 180.0) > 1e-12 ||
      static_cast<int>(std::ceil(plus.duration() * kHz)) + 1 != 85 ||
      static_cast<int>(std::ceil(minus.duration() * kHz)) + 1 != 85)
    throw std::runtime_error("PROFILE_SELF_TEST_FAILED");
  for (const double target : kFullBoundaryRelativeRouteDeg) {
    const double target_rad = target * kPi / 180.0;
    if (target_rad < kCommandLowerBoundary ||
        target_rad > kCommandUpperBoundary)
      throw std::runtime_error("STAGED_ROUTE_BOUNDARY_SELF_TEST_FAILED");
  }
  bool missing_boundary_gate_rejected = false;
  try {
    verify_boundary_gate("");
  } catch (const std::runtime_error& error) {
    missing_boundary_gate_rejected =
        std::string(error.what()) ==
            "STABLE_BOUNDARY_BRAKE_FALLBACK_GATE_MISSING";
  }
  if (!missing_boundary_gate_rejected)
    throw std::runtime_error("MISSING_BOUNDARY_GATE_SELF_TEST_FAILED");
  verify_boundary_gate(kBoundarySafetyGate);
  const double q = 5.0 * kPi / 180.0;
  if (!(kSignA * kGear * q < 0.0 && kSignB * kGear * q > 0.0))
    throw std::runtime_error("DUAL_SIGN_SELF_TEST_FAILED");
  for (const LevelConfig config :
       {LevelConfig{0.25, 0.30, 1.0, "LEVEL_A_025"},
        LevelConfig{0.50, 0.60, 1.5, "LEVEL_B_050"}}) {
    std::int16_t previous_a = 0;
    std::int16_t previous_b = 0;
    const int intervals = static_cast<int>(
        std::lround(config.ramp_duration * kHz));
    for (int frame = 0; frame <= intervals; ++frame) {
      const double alpha = config.alpha * smoothstep5(
          static_cast<double>(frame) / intervals);
      const double raw_a = kSignA * alpha * tau_zero / (2.0 * kGear);
      const double raw_b = kSignB * alpha * tau_zero / (2.0 * kGear);
      const double cmd_a = std::clamp(raw_a, -config.cap, config.cap);
      const double cmd_b = std::clamp(raw_b, -config.cap, config.cap);
      MotorCmd ramp_a = make_command(kIdA, kFocMode, 0.0, 0.0,
                                     kKp, kKd, cmd_a);
      MotorCmd ramp_b = make_command(kIdB, kFocMode, 0.0, 0.0,
                                     kKp, kKd, cmd_b);
      const std::int16_t count_a = static_cast<std::int16_t>(
          load_u16_le(ramp_a.get_motor_send_data() + 3));
      const std::int16_t count_b = static_cast<std::int16_t>(
          load_u16_le(ramp_b.get_motor_send_data() + 3));
      if (count_a != -count_b || count_a > previous_a ||
          count_b < previous_b ||
          std::abs(static_cast<int>(count_a)) >
              static_cast<int>(config.cap * 256.0))
        throw std::runtime_error("POSE_GRAVITY_RAMP_SELF_TEST_FAILED");
      previous_a = count_a;
      previous_b = count_b;
    }
  }
  std::cout << "V15_24E_FT_J2_POSE_GRAVITY_SELF_TEST=PASS\n"
            << "SERIAL_PORT_CONSTRUCTED=NO\n"
            << "MUJOCO_VERSION=" << mj_versionString() << '\n'
            << "FROZEN_MODEL_RUNTIME_LOAD=PASS\n"
            << "J2A_ID=0\nJ2B_ID=1\n"
            << "SIGN_A=-1\nSIGN_B=+1\nSIGN_RELATION=OPPOSITE\n"
            << "SEPARATE_RAW_CURRENT_TARGETS=YES\n"
            << "DUAL_KP=0.60\nDUAL_KD=0.05\n"
             << "ZERO_MODEL_J2_GRAVITY_NM=" << std::setprecision(17)
             << tau_zero << '\n'
             << "MUJOCO_FORWARD_AVG_US=" << benchmark_average_us << '\n'
             << "MUJOCO_FORWARD_100HZ_BUDGET=PASS\n"
            << "LEVEL_A_ALPHA=0.25\nLEVEL_A_CAP_NM=0.30\n"
            << "LEVEL_B_ALPHA=0.50\nLEVEL_B_CAP_NM=0.60\n"
            << "POSE_DEPENDENT_TFF_FORMULA=PASS\n"
            << "MODEL_QJ2_SOURCE=B_MODEL_ANCHOR_PLUS_ACTUAL_B_RELATIVE_FEEDBACK\n"
            << "ZERO_TFF_HOLD_DURATION_S=0.5\n"
            << "GRAVITY_HOLD_DURATION_S=1.0\n"
            << "REAL_BOUNDARY_B_CAPTURED_FROM_STARTUP_BRAKE=YES\n"
            << "TEST_CENTER_C_RELATIVE_TO_B_DEG=5\n"
            << "COMMAND_ROUTE_B_RELATIVE_DEG=0_TO_5_TO_10_TO_5_TO_0_TO_5_TO_0\n"
            << "MOTION_ROUTE_AT_C=0_TO_PLUS5_TO_0_TO_MINUS5_TO_0\n"
            << "COMMAND_BOUNDARY_B_RELATIVE_DEG=0_TO_10\n"
            << "FEEDBACK_BOUNDARY_B_RELATIVE_DEG=-0.5_TO_12\n"
            << "NO_ADDITIONAL_BOUNDARY_MARGIN=YES\n"
            << "ACTUAL_NO_BOUNDARY_CROSSING_ABSOLUTE_GUARANTEE=NO\n"
            << "MISSING_BOUNDARY_GATE_REJECTED=YES\n"
            << "BOUNDARY_GATE_VALUE=" << kBoundarySafetyGate << '\n'
            << "MOVE_TO_TEST_CENTER_PROFILE_EXPECTED_ROWS=85\n"
            << "MOVE_TO_TEST_CENTER_ENDPOINT_EXPECTED_ROWS=40\n"
            << "RETURN_TO_BOUNDARY_PROFILE_EXPECTED_ROWS=85\n"
            << "RETURN_TO_BOUNDARY_ENDPOINT_EXPECTED_ROWS=50\n"
            << "LEVEL_A_EXPECTED_TOTAL_ROWS=1076\n"
            << "LEVEL_B_EXPECTED_TOTAL_ROWS=1126\n"
            << "SYNC_HOLD_ACCEPT_DEG=0.3\nSYNC_MOTION_ACCEPT_DEG=0.7\n"
            << "SYNC_HARD_ABORT_DEG=1.0\n"
            << "CALIBRATE_PATH=NO\nID_WRITE_PATH=NO\nZERO_WRITE_PATH=NO\n"
            << "MASS_COM_INERTIA_MODIFIED=NO\n"
            << "PERMANENT_ZERO_MODIFIED=NO\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  std::signal(SIGHUP, signal_handler);
  std::signal(SIGQUIT, signal_handler);
  try {
    const Cli cli = parse_cli(argc, argv);
    if (cli.self_test) {
      self_test();
      return 0;
    }
    if (cli.anchor_self_test) {
      anchor_self_test(cli.anchor_file);
      return 0;
    }
    Runner runner(cli);
    return runner.run();
  } catch (const std::exception& error) {
    std::cerr << "V15_24E_FT_J2_POSE_GRAVITY_RESULT=FAIL\nREASON="
              << error.what() << '\n';
    return 2;
  }
}
