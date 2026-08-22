#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <nlohmann/json.hpp>
#include <sys/file.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

namespace {
using Clock = std::chrono::steady_clock;
constexpr char kGate[] = "V15_30A_GUI_GO_CONTROL_AUTHORIZED=YES";
constexpr double kPi = 3.14159265358979323846;
constexpr double kGear = 6.329999923706055;
constexpr double kPeriod = 0.01;
constexpr double kTargetLimit = 10.0 * kPi / 180.0;
constexpr double kFeedbackLimit = 12.0 * kPi / 180.0;
constexpr double kJ2SyncLimit = 1.0 * kPi / 180.0;
constexpr double kArrivalTolerance = 0.5 * kPi / 180.0;
constexpr double kBrakeStationaritySpan = 0.20 * kPi / 180.0;
constexpr double kTargetTimeoutSeconds = 15.0;
constexpr double kLeaseSeconds = 0.5;
constexpr int kBrakeMode = 0;
constexpr int kFocMode = 1;
constexpr int kTemperatureLimit = 60;
std::atomic<bool> g_stop{false};

void signal_handler(int) { g_stop.store(true); }

template <typename T> void zero_object(T& value) {
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
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc & 1U) != 0U
                ? static_cast<std::uint16_t>((crc >> 1U) ^ 0x8408U)
                : static_cast<std::uint16_t>(crc >> 1U);
  }
  return crc;
}

MotorCmd make_command(int id, int mode, double q, double dq, double kp,
                      double kd) {
  MotorCmd command;
  zero_object(command);
  command.motorType = MotorType::GO_M8010_6;
  command.id = static_cast<unsigned short>(id);
  command.mode = static_cast<unsigned short>(mode);
  command.q = static_cast<float>(q);
  command.dq = static_cast<float>(dq);
  command.kp = static_cast<float>(kp);
  command.kd = static_cast<float>(kd);
  command.tau = 0.0F;
  command.Res.u32 = 0;
  command.modify_data(&command);
  if (command.hex_len != 17) throw std::runtime_error("COMMAND_LENGTH_INVALID");
  const auto* raw = command.get_motor_send_data();
  if (raw == nullptr || raw[0] != 0xfeU || raw[1] != 0xeeU ||
      raw[2] != static_cast<std::uint8_t>(id | (mode << 4)) ||
      (raw[2] & 0x80U) != 0U ||
      load_u16_le(raw + 15) != crc16_kermit(raw, 15))
    throw std::runtime_error("COMMAND_PACKET_AUDIT_FAILED");
  return command;
}

struct Options {
  bool execute = false;
  std::string confirm;
  std::string bus;
  int feedback_port = 15300;
};

Options parse_options(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto next = [&]() {
      if (++i >= argc) throw std::runtime_error("CLI_VALUE_MISSING");
      return std::string(argv[i]);
    };
    if (arg == "--execute") options.execute = true;
    else if (arg == "--confirm") options.confirm = next();
    else if (arg == "--bus") options.bus = next();
    else if (arg == "--feedback-port") options.feedback_port = std::stoi(next());
    else throw std::runtime_error("CLI_OPTION_NOT_ALLOWED");
  }
  if (!options.execute) return options;
  if (options.confirm != kGate) throw std::runtime_error("ACTIVE_CONTROL_GATE_MISSING");
  if (options.bus != "j1" && options.bus != "j2" && options.bus != "j345")
    throw std::runtime_error("BUS_NOT_ALLOWED");
  if (options.feedback_port < 1024 || options.feedback_port > 65535)
    throw std::runtime_error("FEEDBACK_PORT_OUT_OF_RANGE");
  return options;
}

struct BusDefinition {
  const char* port;
  std::vector<const char*> locks;
  int command_port;
};

BusDefinition bus_definition(const std::string& bus) {
  if (bus == "j1") return {
      "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if03-port0",
      {"/tmp/v15_30a_gui_j1.lock", "/tmp/go_m8010_ftasqa6f_channel3.lock"}, 15310};
  if (bus == "j2") return {
      "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if01-port0",
      {"/tmp/v15_30a_gui_j2.lock", "/tmp/v15_23d_ft_j2_channel1.lock"}, 15312};
  return {
      "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if02-port0",
      {"/tmp/v15_30a_gui_j345.lock", "/tmp/v15_23c_j3_bus.lock",
       "/tmp/v15_22b_j4_bus.lock", "/tmp/v15_22a_j5_bus.lock"}, 15313};
}

class ProcessLock {
 public:
  explicit ProcessLock(const char* path) {
    fd_ = ::open(path, O_RDWR | O_CREAT | O_CLOEXEC, 0600);
    if (fd_ < 0 || ::flock(fd_, LOCK_EX | LOCK_NB) != 0)
      throw std::runtime_error("PHYSICAL_BUS_ALREADY_OWNED");
  }
  ~ProcessLock() { if (fd_ >= 0) { (void)::flock(fd_, LOCK_UN); (void)::close(fd_); } }
 private:
  int fd_ = -1;
};

class AngleUnwrapper {
 public:
  double update(double raw) {
    if (!initialized_) { initialized_ = true; previous_ = value_ = raw; return raw; }
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

struct MotorRuntime {
  MotorRuntime(std::string motor_name, int motor_id, int logical_joint_index,
               int motor_sign, double maximum_kp, double maximum_kd)
      : name(std::move(motor_name)), id(motor_id),
        joint_index(logical_joint_index), sign(motor_sign),
        kp_limit(maximum_kp), kd_limit(maximum_kd) {}

  std::string name;
  int id;
  int joint_index;
  int sign;
  double kp_limit;
  double kd_limit;
  AngleUnwrapper unwrap;
  std::deque<double> history;
  double unwrapped = 0.0;
  double reference = 0.0;
  double last_q = 0.0;
  double last_dq = 0.0;
  double last_tau = 0.0;
  int temperature = 0;
  int merror = -1;
  int returned_mode = -1;
  bool reference_ready = false;
  bool valid = false;
  bool fault_latched = false;
  int consecutive_invalid = 0;
  bool speed_ready = false;
  double previous_scaled_position = 0.0;
  Clock::time_point previous_feedback_at{};
  int fast_speed_count = 0;
  int slow_speed_count = 0;
};

std::vector<MotorRuntime> make_motors(const std::string& bus) {
  if (bus == "j1") return {{"J1", 0, 0, +1, 0.50, 0.05}};
  if (bus == "j2") return {
      {"J2A", 0, 1, -1, 1.00, 0.05}, {"J2B", 1, 1, +1, 1.00, 0.05}};
  return {
      {"J3", 3, 2, +1, 0.60, 0.05},
      {"J4", 4, 3, +1, 0.50, 0.05},
      {"J5", 5, 4, +1, 0.50, 0.05}};
}

struct Feedback {
  bool transport_ok = false;
  bool identity_ok = false;
  bool valid = false;
  MotorData data;
};

Feedback transact(SerialPort& serial, MotorCmd& command, int expected_id,
                  int expected_mode) {
  Feedback result;
  zero_object(result.data);
  result.data.motorType = MotorType::GO_M8010_6;
  result.data.motor_id = 0xffU;
  result.data.mode = 0xffU;
  result.data.temp = std::numeric_limits<int>::min();
  result.data.merror = -1;
  result.data.q = result.data.dq = result.data.tau =
      std::numeric_limits<float>::quiet_NaN();
  result.data.correct = false;
  bool sent = false;
  try { sent = serial.sendRecv(&command, &result.data); } catch (...) { sent = false; }
  result.transport_ok = sent && result.data.correct;
  result.identity_ok = result.transport_ok &&
      static_cast<int>(result.data.motor_id) == expected_id;
  result.valid = result.identity_ok &&
      static_cast<int>(result.data.mode) == expected_mode &&
      result.data.merror == 0 && result.data.temp >= 0 &&
      result.data.temp < kTemperatureLimit && std::isfinite(result.data.q) &&
      std::isfinite(result.data.dq) && std::isfinite(result.data.tau);
  return result;
}

double median(const std::deque<double>& values) {
  std::vector<double> sorted(values.begin(), values.end());
  std::sort(sorted.begin(), sorted.end());
  const std::size_t middle = sorted.size() / 2U;
  return sorted.size() % 2U ? sorted[middle]
                            : 0.5 * (sorted[middle - 1U] + sorted[middle]);
}

bool apply_velocity_guards(const std::string& bus, MotorRuntime& motor,
                           double logical_velocity) {
  const double speed_degrees = std::abs(logical_velocity) * 180.0 / kPi;
  bool tripped = false;
  if (bus == "j1") {
    motor.fast_speed_count = speed_degrees > 35.0 ? motor.fast_speed_count + 1 : 0;
    motor.slow_speed_count = speed_degrees > 30.0 ? motor.slow_speed_count + 1 : 0;
    if (speed_degrees >= 40.0 || motor.fast_speed_count >= 2 ||
        motor.slow_speed_count >= 10 || std::abs(motor.last_tau) >= 5.0)
      tripped = true;
  } else if (bus == "j2" || motor.name == "J3") {
    if (speed_degrees > 25.0) tripped = true;
  } else {
    motor.slow_speed_count = speed_degrees > 30.0 ? motor.slow_speed_count + 1 : 0;
    if (speed_degrees >= 40.0 || motor.slow_speed_count >= 3)
      tripped = true;
  }
  if (tripped) motor.fault_latched = true;
  return tripped;
}

struct GuiCommand {
  std::string mode = "brake";
  std::array<double, 6> targets{};
  std::array<bool, 6> active_joint_mask{};
  std::uint64_t activation_epoch = 0;
  std::array<double, 6> kp{{0.5, 1.0, 0.6, 0.5, 0.5, 0.0}};
  std::array<double, 6> kd{{0.05, 0.05, 0.05, 0.05, 0.05, 0.0}};
  double vmax = 5.0 * kPi / 180.0;
  double amax = 20.0 * kPi / 180.0;
  Clock::time_point received_at{};
  bool received = false;
};

void parse_command(const std::string& text, GuiCommand& command) {
  const auto value = nlohmann::json::parse(text);
  const std::string schema = value.at("schema").get<std::string>();
  if (schema != "go-m8010-gui-command/1.0" &&
      schema != "go-m8010-gui-command/1.1")
    throw std::runtime_error("COMMAND_SCHEMA_MISMATCH");
  const std::string mode = value.at("mode").get<std::string>();
  if (mode != "brake" && mode != "drag" && mode != "hold" && mode != "position")
    throw std::runtime_error("COMMAND_MODE_INVALID");
  if (schema == "go-m8010-gui-command/1.0" && mode != "brake")
    throw std::runtime_error("COMMAND_LEGACY_ACTIVE_REJECTED");
  GuiCommand candidate = command;
  const auto targets = value.at("targets_rad").get<std::vector<double>>();
  const auto kp = value.at("kp").get<std::vector<double>>();
  const auto kd = value.at("kd").get<std::vector<double>>();
  const nlohmann::json active_joint_mask =
      schema == "go-m8010-gui-command/1.0"
          ? nlohmann::json::array({false, false, false, false, false, false})
          : value.at("active_joint_mask");
  if (targets.size() != 6U || kp.size() != 6U || kd.size() != 6U)
    throw std::runtime_error("COMMAND_VECTOR_SIZE_INVALID");
  if (!active_joint_mask.is_array() || active_joint_mask.size() != 6U)
    throw std::runtime_error("COMMAND_ACTIVE_MASK_SIZE_INVALID");
  const std::array<double, 6> kp_limits{{0.5, 1.0, 0.6, 0.5, 0.5, 0.0}};
  const std::array<double, 6> kd_limits{{0.05, 0.05, 0.05, 0.05, 0.05, 0.0}};
  for (std::size_t i = 0; i < 6U; ++i) {
    if (!active_joint_mask.at(i).is_boolean())
      throw std::runtime_error("COMMAND_ACTIVE_MASK_TYPE_INVALID");
    if (!std::isfinite(targets[i]) || std::abs(targets[i]) > kTargetLimit + 1e-12)
      throw std::runtime_error("COMMAND_TARGET_ENVELOPE");
    if (!std::isfinite(kp[i]) || kp[i] < 0.0 || kp[i] > kp_limits[i] + 1e-12 ||
        !std::isfinite(kd[i]) || kd[i] < 0.0 || kd[i] > kd_limits[i] + 1e-12)
      throw std::runtime_error("COMMAND_GAIN_ENVELOPE");
    candidate.targets[i] = targets[i];
    candidate.active_joint_mask[i] = active_joint_mask.at(i).get<bool>();
    candidate.kp[i] = kp[i];
    candidate.kd[i] = kd[i];
  }
  const double requested_vmax = value.at("maximum_velocity_rad_s").get<double>();
  const double requested_amax = value.at("maximum_acceleration_rad_s2").get<double>();
  if (!std::isfinite(requested_vmax) || requested_vmax <= 0.0 ||
      !std::isfinite(requested_amax) || requested_amax <= 0.0)
    throw std::runtime_error("COMMAND_PROFILE_INVALID");
  candidate.vmax = std::min(requested_vmax, 5.0 * kPi / 180.0);
  candidate.amax = std::min(requested_amax, 20.0 * kPi / 180.0);
  if (mode == "brake" &&
      std::any_of(candidate.active_joint_mask.begin(), candidate.active_joint_mask.end(),
                  [](bool active) { return active; }))
    throw std::runtime_error("COMMAND_BRAKE_MASK_NOT_EMPTY");
  if (schema == "go-m8010-gui-command/1.0") {
    candidate.activation_epoch = 0;
  } else {
    const auto& activation_epoch = value.at("activation_epoch");
    if (!activation_epoch.is_number_unsigned())
      throw std::runtime_error("COMMAND_ACTIVATION_EPOCH_INVALID");
    candidate.activation_epoch = activation_epoch.get<std::uint64_t>();
    if (candidate.activation_epoch >
        static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()))
      throw std::runtime_error("COMMAND_ACTIVATION_EPOCH_INVALID");
  }
  if (mode != "brake" &&
      std::any_of(candidate.active_joint_mask.begin(), candidate.active_joint_mask.end(),
                  [](bool active) { return active; }) &&
      candidate.activation_epoch == 0)
    throw std::runtime_error("COMMAND_ACTIVE_EPOCH_ZERO");
  candidate.mode = mode;
  candidate.received_at = Clock::now();
  candidate.received = true;
  command = std::move(candidate);
}

std::uint64_t minimum_epoch_after_lease(
    std::uint64_t current_minimum, bool lease_fresh,
    bool active_requested, std::uint64_t command_epoch) {
  if (lease_fresh || !active_requested) return current_minimum;
  return std::max(current_minimum, command_epoch + 1U);
}

struct CommandSafetyState {
  std::uint64_t minimum_activation_epoch = 0;
  std::uint64_t last_seen_activation_epoch = 0;
};

bool command_epoch_is_acceptable(
    const GuiCommand& command, const CommandSafetyState& safety) {
  return command.mode == "brake" ||
      command.activation_epoch >= safety.last_seen_activation_epoch;
}

void observe_valid_command(
    const GuiCommand& command, CommandSafetyState& safety) {
  safety.last_seen_activation_epoch = std::max(
      safety.last_seen_activation_epoch, command.activation_epoch);
  if (command.mode == "brake")
    safety.minimum_activation_epoch = std::max(
        safety.minimum_activation_epoch,
        safety.last_seen_activation_epoch + 1U);
}

void observe_interarrival_lease(
    const GuiCommand& current, Clock::time_point received_at,
    CommandSafetyState& safety) {
  const bool active_requested = current.received &&
      (current.mode == "drag" || current.mode == "hold" ||
       current.mode == "position") &&
      std::any_of(
          current.active_joint_mask.begin(), current.active_joint_mask.end(),
          [](bool active) { return active; });
  const bool lease_fresh = current.received &&
      std::chrono::duration<double>(received_at - current.received_at).count() <=
          kLeaseSeconds;
  safety.minimum_activation_epoch = minimum_epoch_after_lease(
      safety.minimum_activation_epoch, lease_fresh, active_requested,
      current.activation_epoch);
}

void command_mask_self_test() {
  GuiCommand command;
  parse_command(R"({"schema":"go-m8010-gui-command/1.1","sequence":1,
      "mode":"position","targets_rad":[0,0,0,0,0,0],
      "active_joint_mask":[false,false,false,true,false,false],
      "activation_epoch":7,"maximum_velocity_rad_s":0.08,
      "maximum_acceleration_rad_s2":0.3,
      "kp":[0.5,1.0,0.6,0.5,0.5,0.0],
      "kd":[0.05,0.05,0.05,0.05,0.05,0.0]})", command);
  const std::uint64_t minimum = minimum_epoch_after_lease(
      0, false, command.active_joint_mask[3], command.activation_epoch);
  if (!command.active_joint_mask[3] || command.active_joint_mask[2] ||
      command.active_joint_mask[4] || command.activation_epoch != 7U ||
      minimum != 8U || command.activation_epoch >= minimum)
    throw std::runtime_error("COMMAND_MASK_SELF_TEST_FAILED");
  CommandSafetyState safety;
  observe_valid_command(command, safety);
  GuiCommand replay = command;
  command.activation_epoch = 8U;
  observe_valid_command(command, safety);
  if (command_epoch_is_acceptable(replay, safety))
    throw std::runtime_error("COMMAND_REPLAY_SELF_TEST_FAILED");
  observe_interarrival_lease(
      command, command.received_at + std::chrono::milliseconds(501), safety);
  command.mode = "brake";
  observe_valid_command(command, safety);
  if (safety.minimum_activation_epoch != 9U)
    throw std::runtime_error("COMMAND_BRAKE_LATCH_SELF_TEST_FAILED");
  command.activation_epoch = static_cast<std::uint64_t>(
      std::numeric_limits<std::int64_t>::max());
  observe_valid_command(command, safety);
  if (safety.minimum_activation_epoch != command.activation_epoch + 1U)
    throw std::runtime_error("COMMAND_HARD_STOP_FENCE_SELF_TEST_FAILED");
}

void receive_latest(
    int socket_fd, GuiCommand& command, CommandSafetyState& safety) {
  std::array<char, 8192> buffer{};
  while (true) {
    const ssize_t received = ::recv(socket_fd, buffer.data(), buffer.size(), 0);
    if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
    if (received <= 0) return;
    try {
      observe_interarrival_lease(command, Clock::now(), safety);
      GuiCommand candidate;
      parse_command(
          std::string(buffer.data(), static_cast<std::size_t>(received)), candidate);
      if (!command_epoch_is_acceptable(candidate, safety))
        throw std::runtime_error("COMMAND_ACTIVATION_EPOCH_REPLAY");
      observe_valid_command(candidate, safety);
      command = std::move(candidate);
    }
    catch (const std::exception& error) {
      std::cerr << "GUI_COMMAND_REJECTED=" << error.what() << '\n';
    }
  }
}

void update_profile(int joint, const GuiCommand& command,
                    std::array<double, 6>& q_command,
                    std::array<double, 6>& dq_command) {
  const double error = command.targets[static_cast<std::size_t>(joint)] -
                       q_command[static_cast<std::size_t>(joint)];
  const double stopping_speed = std::sqrt(2.0 * command.amax * std::abs(error));
  const double wanted = std::copysign(std::min(command.vmax, stopping_speed), error);
  const double previous_speed = std::abs(dq_command[static_cast<std::size_t>(joint)]);
  const double delta_v = std::clamp(
      wanted - dq_command[static_cast<std::size_t>(joint)],
      -command.amax * kPeriod, command.amax * kPeriod);
  dq_command[static_cast<std::size_t>(joint)] += delta_v;
  const double current_speed = std::abs(dq_command[static_cast<std::size_t>(joint)]);
  const double snap_distance = current_speed * kPeriod +
                               0.5 * command.amax * kPeriod * kPeriod;
  if (previous_speed <= command.amax * kPeriod &&
      current_speed <= command.amax * kPeriod && std::abs(error) <= snap_distance) {
    q_command[static_cast<std::size_t>(joint)] = command.targets[static_cast<std::size_t>(joint)];
    dq_command[static_cast<std::size_t>(joint)] = 0.0;
  } else {
    q_command[static_cast<std::size_t>(joint)] +=
        dq_command[static_cast<std::size_t>(joint)] * kPeriod;
  }
}

std::uint64_t monotonic_ns() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      Clock::now().time_since_epoch()).count());
}

std::string feedback_payload(const std::vector<MotorRuntime>& motors,
                             std::uint64_t stamp, const std::string& mode,
                             bool j2_sync_fault, bool domain_fault) {
  nlohmann::json samples = nlohmann::json::array();
  nlohmann::json controller_mode_by_motor = nlohmann::json::object();
  for (const auto& motor : motors) {
    samples.push_back({
        {"motor", motor.name}, {"position_rad", motor.last_q},
        {"velocity_rad_s", motor.last_dq}, {"temperature_c", motor.temperature},
        {"merror", motor.merror}, {"communication_ok", motor.valid && !motor.fault_latched}});
    controller_mode_by_motor[motor.name] =
        mode != "brake" && motor.returned_mode == kFocMode ? mode : "brake";
  }
  return nlohmann::json({
      {"schema", "go-m8010-motor-feedback/1.0"},
      {"source_monotonic_ns", stamp}, {"samples", samples},
      {"controller_mode", mode},
      {"controller_mode_by_motor", controller_mode_by_motor},
      {"j2_sync_fault", j2_sync_fault},
      {"domain_fault", domain_fault}}).dump();
}

bool send_terminal_brake(SerialPort& serial, std::vector<MotorRuntime>& motors,
                         const std::string& bus) noexcept {
  try {
    std::vector<std::deque<double>> position_windows(motors.size());
    for (int frame = 0; frame < 20; ++frame) {
      bool frame_safe = true;
      std::vector<double> frame_positions(motors.size(), 0.0);
      for (std::size_t index = 0; index < motors.size(); ++index) {
        auto& motor = motors[index];
        MotorCmd brake = make_command(motor.id, kBrakeMode, 0.0, 0.0, 0.0, 0.0);
        const Feedback feedback = transact(serial, brake, motor.id, kBrakeMode);
        bool sample_safe = feedback.valid;
        if (feedback.valid) {
          motor.last_q = feedback.data.q;
          motor.last_dq = feedback.data.dq;
          motor.last_tau = feedback.data.tau;
          motor.unwrapped = motor.unwrap.update(motor.last_q);
          frame_positions[index] = motor.unwrapped;
          const auto feedback_at = Clock::now();
          bool velocity_valid = false;
          bool velocity_tripped = false;
          const double scaled_position = motor.sign * motor.unwrapped / kGear;
          if (motor.speed_ready) {
            const double dt = std::chrono::duration<double>(
                feedback_at - motor.previous_feedback_at).count();
            if (dt > 1e-4 && dt < 0.5) {
              velocity_valid = true;
              velocity_tripped = apply_velocity_guards(
                  bus, motor,
                  (scaled_position - motor.previous_scaled_position) / dt);
            }
          }
          motor.previous_scaled_position = scaled_position;
          motor.previous_feedback_at = feedback_at;
          motor.speed_ready = true;
          const bool torque_safe =
              bus != "j1" || std::abs(motor.last_tau) < 5.0;
          sample_safe = velocity_valid && !velocity_tripped && torque_safe;
        }
        frame_safe = frame_safe && sample_safe;
      }
      if (frame_safe) {
        for (std::size_t index = 0; index < motors.size(); ++index) {
          auto& window = position_windows[index];
          window.push_back(frame_positions[index]);
          while (window.size() > 5U) window.pop_front();
        }
      } else {
        for (auto& window : position_windows) window.clear();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return std::all_of(position_windows.begin(), position_windows.end(),
                       [](const std::deque<double>& window) {
      if (window.size() != 5U) return false;
      const auto minmax = std::minmax_element(window.begin(), window.end());
      return *minmax.second - *minmax.first <=
             kGear * kBrakeStationaritySpan + 1e-12;
    });
  } catch (...) {
    return false;
  }
}

class TerminalBrakeGuard {
 public:
  TerminalBrakeGuard(SerialPort& serial, std::vector<MotorRuntime>& motors,
                     const std::string& bus)
      : serial_(serial), motors_(motors), bus_(bus) {}
  ~TerminalBrakeGuard() {
    if (!armed_) return;
    const bool confirmed = send_terminal_brake(serial_, motors_, bus_);
    const char* report = confirmed
        ? "TERMINAL_PATH=EXCEPTION\nFINAL_MODE=BRAKE\nFINAL_BRAKE=PASS\n"
        : "TERMINAL_PATH=EXCEPTION\nFINAL_MODE=UNCONFIRMED\nFINAL_BRAKE=FAIL\n";
    std::size_t remaining = std::strlen(report);
    while (remaining > 0U) {
      const ssize_t written = ::write(STDERR_FILENO, report, remaining);
      if (written > 0) {
        report += static_cast<std::size_t>(written);
        remaining -= static_cast<std::size_t>(written);
      } else if (written < 0 && errno == EINTR) {
        continue;
      } else {
        break;
      }
    }
  }
  bool brake_and_disarm() noexcept {
    const bool confirmed = !armed_ || send_terminal_brake(serial_, motors_, bus_);
    armed_ = false;
    return confirmed;
  }
 private:
  SerialPort& serial_;
  std::vector<MotorRuntime>& motors_;
  const std::string& bus_;
  bool armed_ = true;
};

int run(const Options& options) {
  if (!options.execute) {
    command_mask_self_test();
    std::cout << "DRY_RUN=YES\nSERIAL_OPENED=NO\nDEFAULT_MODE=BRAKE\n"
                 "CONTROL_LOOP_HZ=100\nTARGET_LIMIT_DEG=10\nTFF=0\n"
                 "MOTOR_INTERNAL_ZERO_WRITE=NO\n";
    return 0;
  }
  if (::prctl(PR_SET_PDEATHSIG, SIGTERM) != 0 || ::getppid() == 1)
    throw std::runtime_error("PARENT_DEATH_GUARD_FAILED");
  const BusDefinition definition = bus_definition(options.bus);
  if (::access(definition.port, R_OK | W_OK) != 0)
    throw std::runtime_error("STABLE_PORT_NOT_ACCESSIBLE");
  if (queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE) != kBrakeMode ||
      queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC) != kFocMode ||
      std::abs(queryGearRatio(MotorType::GO_M8010_6) - kGear) > 1e-6)
    throw std::runtime_error("SDK_AUTHORITY_MISMATCH");
  std::vector<std::unique_ptr<ProcessLock>> locks;
  for (const char* path : definition.locks)
    locks.push_back(std::make_unique<ProcessLock>(path));
  SerialPort serial(definition.port, 16, 4000000, 20000, BlockYN::NO,
                    bytesize_t::eightbits, parity_t::parity_none,
                    stopbits_t::stopbits_one, flowcontrol_t::flowcontrol_none);
  auto motors = make_motors(options.bus);
  TerminalBrakeGuard brake_guard(serial, motors, options.bus);

  const int command_socket = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  const int feedback_socket = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (command_socket < 0 || feedback_socket < 0)
    throw std::runtime_error("UDP_SOCKET_FAILED");
  const int flags = ::fcntl(command_socket, F_GETFL, 0);
  if (flags < 0 || ::fcntl(command_socket, F_SETFL, flags | O_NONBLOCK) < 0)
    throw std::runtime_error("UDP_NONBLOCK_FAILED");
  sockaddr_in bind_address{};
  bind_address.sin_family = AF_INET;
  bind_address.sin_port = htons(static_cast<std::uint16_t>(definition.command_port));
  bind_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::bind(command_socket, reinterpret_cast<const sockaddr*>(&bind_address),
             sizeof(bind_address)) != 0)
    throw std::runtime_error("UDP_COMMAND_BIND_FAILED");
  sockaddr_in feedback_address{};
  feedback_address.sin_family = AF_INET;
  feedback_address.sin_port = htons(static_cast<std::uint16_t>(options.feedback_port));
  feedback_address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

  GuiCommand command;
  std::array<double, 6> q_command{};
  std::array<double, 6> dq_command{};
  std::string previous_mode = "brake";
  std::array<bool, 6> previous_active_joint_mask{};
  CommandSafetyState command_safety;
  bool j2_sync_fault = false;
  bool domain_fault = false;
  std::array<bool, 6> position_tracking{};
  std::array<double, 6> last_position_targets{};
  std::array<Clock::time_point, 6> position_started_at{};
  position_started_at.fill(Clock::now());
  auto next = Clock::now();
  std::uint64_t cycles = 0;
  bool previous_j2_pair_ready = false;
  while (!g_stop.load()) {
    const auto loop_started = Clock::now();
    if (cycles > 0U && loop_started > next + std::chrono::milliseconds(2))
      domain_fault = true;
    receive_latest(command_socket, command, command_safety);
    const bool command_lease_fresh = command.received && !g_stop.load() &&
        std::chrono::duration<double>(Clock::now() - command.received_at).count() <=
            kLeaseSeconds;
    const bool command_requests_active_owned_joint = command.received &&
        (command.mode == "drag" || command.mode == "hold" ||
         command.mode == "position") &&
        std::any_of(motors.begin(), motors.end(), [&](const MotorRuntime& motor) {
          return command.active_joint_mask[
              static_cast<std::size_t>(motor.joint_index)];
        });
    command_safety.minimum_activation_epoch = minimum_epoch_after_lease(
        command_safety.minimum_activation_epoch, command_lease_fresh,
        command_requests_active_owned_joint, command.activation_epoch);
    std::string effective_mode = command.mode;
    if (!command_lease_fresh)
      effective_mode = "brake";
    if (domain_fault) effective_mode = "brake";
    if (effective_mode != "brake" &&
        command.activation_epoch < command_safety.minimum_activation_epoch)
      effective_mode = "brake";
    if (effective_mode == "drag" || effective_mode == "hold" ||
        effective_mode == "position") {
      const bool owned_joint_selected =
          std::any_of(motors.begin(), motors.end(), [&](const MotorRuntime& motor) {
            return command.active_joint_mask[
                static_cast<std::size_t>(motor.joint_index)];
          });
      if (!owned_joint_selected) effective_mode = "brake";
    }
    if (effective_mode == "position") {
      std::set<int> owned_joints;
      for (const auto& motor : motors) owned_joints.insert(motor.joint_index);
      for (const int joint : owned_joints) {
        const std::size_t index = static_cast<std::size_t>(joint);
        if (!command.active_joint_mask[index]) {
          position_tracking[index] = false;
          continue;
        }
        if (!position_tracking[index] ||
            std::abs(command.targets[index] - last_position_targets[index]) > 1e-9)
          position_started_at[index] = Clock::now();
        last_position_targets[index] = command.targets[index];
        position_tracking[index] = true;
      }
    } else {
      position_tracking.fill(false);
    }
    bool j2_pair_ready = options.bus != "j2" ||
        (motors.size() == 2U &&
         std::all_of(motors.begin(), motors.end(), [](const MotorRuntime& motor) {
           return motor.reference_ready && motor.valid && !motor.fault_latched;
         }));
    const bool position_control_requested =
        effective_mode == "hold" || effective_mode == "position";
    const bool active_transition = position_control_requested &&
        (effective_mode != previous_mode ||
         std::any_of(motors.begin(), motors.end(), [&](const MotorRuntime& motor) {
           const std::size_t index = static_cast<std::size_t>(motor.joint_index);
           return command.active_joint_mask[index] && !previous_active_joint_mask[index];
         }));
    if (options.bus == "j2" && j2_pair_ready &&
        (active_transition || !previous_j2_pair_ready)) {
      const double q_a = -1.0 * (motors[0].unwrapped - motors[0].reference) / kGear;
      const double q_b = +1.0 * (motors[1].unwrapped - motors[1].reference) / kGear;
      q_command[1] = 0.5 * (q_a + q_b);
      dq_command[1] = 0.0;
    } else if (options.bus != "j2" && position_control_requested) {
      for (const auto& motor : motors) {
        const std::size_t index = static_cast<std::size_t>(motor.joint_index);
        const bool motor_transition = command.active_joint_mask[index] &&
            (effective_mode != previous_mode || !previous_active_joint_mask[index]);
        if (!motor.reference_ready || !motor_transition) continue;
        q_command[index] =
            motor.sign * (motor.unwrapped - motor.reference) / kGear;
        dq_command[index] = 0.0;
      }
    }
    std::set<int> planned;
    if (effective_mode == "position" && j2_pair_ready) {
      for (const auto& motor : motors)
        if (motor.reference_ready && !motor.fault_latched &&
            command.active_joint_mask[static_cast<std::size_t>(motor.joint_index)] &&
            planned.insert(motor.joint_index).second)
          update_profile(motor.joint_index, command, q_command, dq_command);
    } else {
      dq_command.fill(0.0);
    }

    bool any_foc_sent = false;
    for (auto& motor : motors) {
      int send_mode = kBrakeMode;
      double q = 0.0, dq = 0.0, kp = 0.0, kd = 0.0;
      const bool active_allowed = motor.reference_ready && !motor.fault_latched &&
          !(options.bus == "j2" && j2_sync_fault) && j2_pair_ready &&
          command_lease_fresh &&
          command.active_joint_mask[static_cast<std::size_t>(motor.joint_index)];
      if (effective_mode == "drag" && active_allowed) {
        send_mode = kFocMode;
      } else if ((effective_mode == "hold" || effective_mode == "position") && active_allowed) {
        send_mode = kFocMode;
        const std::size_t joint = static_cast<std::size_t>(motor.joint_index);
        q = motor.reference + motor.sign * kGear * q_command[joint];
        dq = motor.sign * kGear * dq_command[joint];
        kp = std::min(command.kp[joint], motor.kp_limit);
        kd = std::min(command.kd[joint], motor.kd_limit);
      }
      MotorCmd packet = make_command(motor.id, send_mode, q, dq, kp, kd);
      any_foc_sent = any_foc_sent || send_mode == kFocMode;
      const Feedback feedback = transact(serial, packet, motor.id, send_mode);
      motor.valid = feedback.valid;
      if (options.bus == "j2" && !feedback.valid) j2_pair_ready = false;
      if (feedback.identity_ok) {
        motor.temperature = feedback.data.temp;
        motor.merror = feedback.data.merror;
        motor.returned_mode = feedback.data.mode;
        if (feedback.data.merror != 0 || feedback.data.temp < 0 ||
            feedback.data.temp >= kTemperatureLimit ||
            static_cast<int>(feedback.data.mode) != send_mode)
          motor.fault_latched = true;
      }
      if (feedback.valid) {
        motor.consecutive_invalid = 0;
        motor.last_q = feedback.data.q;
        motor.last_dq = feedback.data.dq;
        motor.last_tau = feedback.data.tau;
        motor.unwrapped = motor.unwrap.update(motor.last_q);
        const auto feedback_at = Clock::now();
        const double scaled_position = motor.sign * motor.unwrapped / kGear;
        if (motor.speed_ready) {
          const double dt = std::chrono::duration<double>(
              feedback_at - motor.previous_feedback_at).count();
          if (dt > 1e-4 && dt < 0.5)
            apply_velocity_guards(
                options.bus, motor,
                (scaled_position - motor.previous_scaled_position) / dt);
        }
        motor.previous_scaled_position = scaled_position;
        motor.previous_feedback_at = feedback_at;
        motor.speed_ready = true;
        if (!motor.reference_ready && send_mode == kBrakeMode) {
          motor.history.push_back(motor.unwrapped);
          while (motor.history.size() > 50U) motor.history.pop_front();
          if (motor.history.size() == 50U) {
            const auto minmax = std::minmax_element(motor.history.begin(), motor.history.end());
            std::deque<double> tail(motor.history.end() - 10, motor.history.end());
            const double reference = median(motor.history);
            if ((*minmax.second - *minmax.first) / kGear <= 0.20 * kPi / 180.0 &&
                std::abs(median(tail) - reference) / kGear <= 0.10 * kPi / 180.0) {
              motor.reference = reference;
              motor.reference_ready = true;
            }
          }
        }
        if (motor.reference_ready) {
          const double logical = motor.sign * (motor.unwrapped - motor.reference) / kGear;
          if (std::abs(logical) > kFeedbackLimit) motor.fault_latched = true;
        }
      } else if (++motor.consecutive_invalid >= (options.bus == "j345" ? 1 : 5)) {
        motor.fault_latched = true;
      }
      if (options.bus == "j2" && motor.fault_latched) j2_pair_ready = false;
      if (motor.fault_latched && send_mode != kBrakeMode) {
        MotorCmd brake = make_command(motor.id, kBrakeMode, 0.0, 0.0, 0.0, 0.0);
        (void)transact(serial, brake, motor.id, kBrakeMode);
      }
    }

    if (options.bus == "j2" &&
        std::any_of(motors.begin(), motors.end(), [](const MotorRuntime& motor) {
          return motor.fault_latched || !motor.valid;
        })) {
      domain_fault = true;
    }
    if (options.bus == "j2" && motors.size() == 2U &&
        motors[0].reference_ready && motors[1].reference_ready &&
        motors[0].valid && motors[1].valid) {
      const double q_a = -1.0 * (motors[0].unwrapped - motors[0].reference) / kGear;
      const double q_b = +1.0 * (motors[1].unwrapped - motors[1].reference) / kGear;
      if (std::abs(q_a - q_b) > kJ2SyncLimit) {
        j2_sync_fault = true;
        for (auto& motor : motors) {
          MotorCmd brake = make_command(motor.id, kBrakeMode, 0.0, 0.0, 0.0, 0.0);
          (void)transact(serial, brake, motor.id, kBrakeMode);
        }
      }
    }
    if (options.bus == "j2" && position_tracking[1] &&
        std::chrono::duration<double>(Clock::now() - position_started_at[1]).count() >=
            kTargetTimeoutSeconds && motors.size() == 2U &&
          motors[0].reference_ready && motors[1].reference_ready) {
        const double q_a = -1.0 * (motors[0].unwrapped - motors[0].reference) / kGear;
        const double q_b = +1.0 * (motors[1].unwrapped - motors[1].reference) / kGear;
        if (std::abs(0.5 * (q_a + q_b) - command.targets[1]) > kArrivalTolerance)
          domain_fault = true;
    } else if (options.bus != "j2") {
      for (auto& motor : motors) {
        const std::size_t index = static_cast<std::size_t>(motor.joint_index);
        if (!position_tracking[index] ||
            std::chrono::duration<double>(Clock::now() - position_started_at[index]).count() <
                kTargetTimeoutSeconds ||
            !motor.reference_ready || motor.fault_latched)
          continue;
        const double logical = motor.sign * (motor.unwrapped - motor.reference) / kGear;
        if (std::abs(logical - command.targets[index]) > kArrivalTolerance) {
          if (options.bus == "j345") {
            motor.fault_latched = true;
            MotorCmd brake = make_command(motor.id, kBrakeMode, 0.0, 0.0, 0.0, 0.0);
            (void)transact(serial, brake, motor.id, kBrakeMode);
          } else {
            domain_fault = true;
          }
        }
      }
    }
    if (domain_fault || (options.bus == "j2" && j2_sync_fault)) {
      for (auto& motor : motors) {
        MotorCmd brake = make_command(motor.id, kBrakeMode, 0.0, 0.0, 0.0, 0.0);
        (void)transact(serial, brake, motor.id, kBrakeMode);
      }
    }

    const std::uint64_t stamp = monotonic_ns();
    const bool active_requested =
        (effective_mode == "drag" || effective_mode == "hold" ||
         effective_mode == "position") &&
        std::any_of(motors.begin(), motors.end(), [&](const MotorRuntime& motor) {
          return command.active_joint_mask[
              static_cast<std::size_t>(motor.joint_index)];
        });
    const std::string reported_mode =
        (domain_fault || j2_sync_fault || (active_requested && !any_foc_sent))
            ? "brake" : effective_mode;
    const std::string payload = feedback_payload(
        motors, stamp, reported_mode, j2_sync_fault, domain_fault);
    (void)::sendto(feedback_socket, payload.data(), payload.size(), 0,
                   reinterpret_cast<const sockaddr*>(&feedback_address),
                   sizeof(feedback_address));
    previous_mode = effective_mode;
    previous_active_joint_mask = active_requested
        ? command.active_joint_mask : std::array<bool, 6>{};
    previous_j2_pair_ready = j2_pair_ready;
    ++cycles;
    if (options.bus == "j345" &&
        std::any_of(motors.begin(), motors.end(), [](const MotorRuntime& motor) {
          return motor.fault_latched;
        }))
      next = Clock::now();
    next += std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(kPeriod));
    std::this_thread::sleep_until(next);
  }
  const bool final_brake_confirmed = brake_guard.brake_and_disarm();
  ::close(command_socket);
  ::close(feedback_socket);
  std::cout << "GUI_GO_CONTROLLER_BUS=" << options.bus << "\n"
            << "COMPLETED_CYCLES=" << cycles << "\n"
            << "FINAL_MODE=" << (final_brake_confirmed ? "BRAKE" : "UNCONFIRMED") << "\n"
            << "FINAL_BRAKE=" << (final_brake_confirmed ? "PASS" : "FAIL") << "\n"
            << "MOTOR_INTERNAL_ZERO_WRITE=NO\n";
  return final_brake_confirmed ? 0 : 2;
}
}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  try { return run(parse_options(argc, argv)); }
  catch (const std::exception& error) {
    std::cerr << "V15_30A_GUI_GO_CONTROLLER=BLOCKED\nREASON=" << error.what() << '\n';
    return 2;
  }
}
