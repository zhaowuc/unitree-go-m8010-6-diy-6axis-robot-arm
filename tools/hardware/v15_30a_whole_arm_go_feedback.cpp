#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
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

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

namespace {
constexpr char kGate[] = "V15_30A_STATE_ONLY_BRAKE_FEEDBACK=YES";
constexpr char kJ1Port[] = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if03-port0";
constexpr char kJ2Port[] = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if01-port0";
constexpr char kJ345Port[] = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if02-port0";
constexpr char kLockPath[] = "/tmp/v15_30a_whole_arm_go_feedback.lock";
constexpr int kBrakeMode = 0;
constexpr int kTemperatureLimitC = 70;
std::atomic<bool> stop_requested{false};

template <typename T> void zero_object(T& value) {
  static_assert(std::is_trivially_copyable<T>::value, "frozen SDK ABI");
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

MotorCmd brake_command(int id) {
  MotorCmd command;
  zero_object(command);
  command.motorType = MotorType::GO_M8010_6;
  command.id = static_cast<unsigned short>(id);
  command.mode = static_cast<unsigned short>(kBrakeMode);
  command.Res.u32 = 0;
  command.modify_data(&command);
  if (command.hex_len != 17) throw std::runtime_error("COMMAND_LENGTH_INVALID");
  const std::uint8_t* raw = command.get_motor_send_data();
  if (raw == nullptr || raw[0] != 0xfeU || raw[1] != 0xeeU ||
      raw[2] != static_cast<std::uint8_t>(id) || (raw[2] & 0x80U) != 0U ||
      load_u16_le(raw + 15) != crc16_kermit(raw, 15))
    throw std::runtime_error("BRAKE_PACKET_AUDIT_FAILED");
  return command;
}

void initialize_feedback(MotorData& data) {
  zero_object(data);
  data.motorType = MotorType::GO_M8010_6;
  data.motor_id = 0xffU;
  data.mode = 0xffU;
  data.temp = std::numeric_limits<int>::min();
  data.merror = -1;
  data.q = data.dq = data.tau = std::numeric_limits<float>::quiet_NaN();
  data.correct = false;
}

struct Sample {
  const char* name = "";
  int id = -1;
  bool send_recv = false;
  bool valid = false;
  MotorData data;
};

Sample transact(SerialPort& serial, const char* name, int id) {
  MotorCmd command = brake_command(id);
  Sample sample;
  sample.name = name;
  sample.id = id;
  initialize_feedback(sample.data);
  try { sample.send_recv = serial.sendRecv(&command, &sample.data); }
  catch (...) { sample.send_recv = false; }
  sample.valid = sample.send_recv && sample.data.correct &&
      static_cast<int>(sample.data.motor_id) == id &&
      static_cast<int>(sample.data.mode) == kBrakeMode &&
      sample.data.merror == 0 && sample.data.temp >= 0 &&
      sample.data.temp < kTemperatureLimitC && std::isfinite(sample.data.q) &&
      std::isfinite(sample.data.dq) && std::isfinite(sample.data.tau);
  return sample;
}

struct Options {
  bool execute = false;
  std::string confirm;
  std::string output;
  double seconds = 30.0;
  double hz = 50.0;
  int udp_port = 15300;
};

Options parse_options(int argc, char** argv) {
  Options value;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() { if (++i >= argc) throw std::runtime_error("CLI_VALUE_MISSING"); return std::string(argv[i]); };
    if (arg == "--execute") value.execute = true;
    else if (arg == "--confirm") value.confirm = next();
    else if (arg == "--output") value.output = next();
    else if (arg == "--seconds") value.seconds = std::stod(next());
    else if (arg == "--hz") value.hz = std::stod(next());
    else if (arg == "--udp-port") value.udp_port = std::stoi(next());
    else throw std::runtime_error("CLI_OPTION_NOT_ALLOWED");
  }
  if (!value.execute) return value;
  if (value.confirm != kGate) throw std::runtime_error("OPERATOR_GATE_MISSING");
  if (value.output.empty()) throw std::runtime_error("OUTPUT_REQUIRED");
  if (!std::isfinite(value.seconds) || value.seconds < 1.0 || value.seconds > 3600.0)
    throw std::runtime_error("SECONDS_OUT_OF_RANGE");
  if (!std::isfinite(value.hz) || value.hz < 50.0 || value.hz > 100.0)
    throw std::runtime_error("HZ_OUT_OF_RANGE");
  if (value.udp_port < 1024 || value.udp_port > 65535)
    throw std::runtime_error("UDP_PORT_OUT_OF_RANGE");
  return value;
}

class ProcessLock {
 public:
  ProcessLock() {
    fd = ::open(kLockPath, O_RDWR | O_CREAT | O_CLOEXEC, 0600);
    if (fd < 0 || ::flock(fd, LOCK_EX | LOCK_NB) != 0)
      throw std::runtime_error("WHOLE_ARM_GO_FEEDBACK_LOCKED");
  }
  ~ProcessLock() { if (fd >= 0) { (void)::flock(fd, LOCK_UN); (void)::close(fd); } }
 private: int fd = -1;
};

std::uint64_t monotonic_ns() {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

std::string json_payload(const std::array<Sample, 6>& samples, std::uint64_t stamp) {
  std::ostringstream out;
  out << std::setprecision(17) << "{\"schema\":\"go-m8010-motor-feedback/1.0\",\"source_monotonic_ns\":"
      << stamp << ",\"samples\":[";
  for (std::size_t i = 0; i < samples.size(); ++i) {
    const auto& s = samples[i];
    if (i != 0U) out << ',';
    out << "{\"motor\":\"" << s.name << "\",\"position_rad\":" << s.data.q
        << ",\"velocity_rad_s\":" << s.data.dq << ",\"temperature_c\":" << s.data.temp
        << ",\"merror\":" << s.data.merror << ",\"communication_ok\":"
        << (s.valid ? "true" : "false") << '}';
  }
  out << "]}";
  return out.str();
}

void signal_handler(int) { stop_requested.store(true); }

int run(const Options& options) {
  if (!options.execute) {
    std::cout << "DRY_RUN=YES\nSERIAL_OPENED=NO\nBRAKE_ONLY_PATH=YES\nFOC_PATH=NO\n";
    return 0;
  }
  for (const char* path : {kJ1Port, kJ2Port, kJ345Port})
    if (::access(path, R_OK | W_OK) != 0) throw std::runtime_error("STABLE_PORT_NOT_ACCESSIBLE");
  if (queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE) != kBrakeMode ||
      std::abs(queryGearRatio(MotorType::GO_M8010_6) - 6.329999923706055) > 1e-6)
    throw std::runtime_error("SDK_AUTHORITY_MISMATCH");
  ProcessLock lock;
  SerialPort j1(kJ1Port, 16, 4000000, 20000, BlockYN::NO, bytesize_t::eightbits,
                parity_t::parity_none, stopbits_t::stopbits_one, flowcontrol_t::flowcontrol_none);
  SerialPort j2(kJ2Port, 16, 4000000, 20000, BlockYN::NO, bytesize_t::eightbits,
                parity_t::parity_none, stopbits_t::stopbits_one, flowcontrol_t::flowcontrol_none);
  SerialPort j345(kJ345Port, 16, 4000000, 20000, BlockYN::NO, bytesize_t::eightbits,
                  parity_t::parity_none, stopbits_t::stopbits_one, flowcontrol_t::flowcontrol_none);
  std::ofstream csv(options.output, std::ios::out | std::ios::trunc);
  if (!csv) throw std::runtime_error("OUTPUT_OPEN_FAILED");
  csv << "cycle,source_monotonic_ns,motor,id,send_recv,correct,returned_id,mode,q_raw_rad,dq_raw_rad_s,tau_raw,temperature_c,merror,valid\n";
  csv << std::setprecision(17);
  const int sock = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (sock < 0) throw std::runtime_error("UDP_SOCKET_FAILED");
  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port = htons(static_cast<std::uint16_t>(options.udp_port));
  destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  std::array<int, 6> consecutive_invalid{};
  std::array<int, 6> valid_counts{};
  const int cycles = static_cast<int>(std::llround(options.seconds * options.hz));
  const auto period = std::chrono::duration<double>(1.0 / options.hz);
  auto next = std::chrono::steady_clock::now();
  int completed = 0;
  for (int cycle = 0; cycle < cycles && !stop_requested.load(); ++cycle) {
    std::array<Sample, 6> samples{{
      transact(j1, "J1", 0), transact(j2, "J2A", 0), transact(j2, "J2B", 1),
      transact(j345, "J3", 3), transact(j345, "J4", 4), transact(j345, "J5", 5)}};
    const auto stamp = monotonic_ns();
    bool all_valid = true;
    for (std::size_t i = 0; i < samples.size(); ++i) {
      const auto& s = samples[i];
      all_valid = all_valid && s.valid;
      if (s.valid) { ++valid_counts[i]; consecutive_invalid[i] = 0; }
      else if (++consecutive_invalid[i] >= 5) {
        ::close(sock);
        throw std::runtime_error(std::string(s.name) + "_FIVE_CONSECUTIVE_INVALID");
      }
      csv << cycle << ',' << stamp << ',' << s.name << ',' << s.id << ',' << s.send_recv << ','
          << s.data.correct << ',' << static_cast<int>(s.data.motor_id) << ','
          << static_cast<int>(s.data.mode) << ',' << s.data.q << ',' << s.data.dq << ','
          << s.data.tau << ',' << s.data.temp << ',' << s.data.merror << ',' << s.valid << '\n';
    }
    if (all_valid) {
      const std::string payload = json_payload(samples, stamp);
      if (::sendto(sock, payload.data(), payload.size(), 0,
                   reinterpret_cast<const sockaddr*>(&destination), sizeof(destination)) < 0) {
        ::close(sock);
        throw std::runtime_error("UDP_SEND_FAILED");
      }
    }
    ++completed;
    if (cycle % 50 == 0) csv.flush();
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next);
  }
  ::close(sock);
  csv.flush();
  std::cout << "GO_FEEDBACK_COMPLETED_CYCLES=" << completed << '\n';
  for (std::size_t i = 0; i < valid_counts.size(); ++i)
    std::cout << std::array<const char*, 6>{{"J1","J2A","J2B","J3","J4","J5"}}[i]
              << "_VALID=" << valid_counts[i] << '/' << completed << '\n';
  std::cout << "GO_ACTIVE_MOTION_USED=NO\nGO_COMMAND_MODE=BRAKE\n";
  return completed > 0 ? 0 : 2;
}
}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  try { return run(parse_options(argc, argv)); }
  catch (const std::exception& error) {
    std::cerr << "V15_30A_GO_FEEDBACK_RESULT=BLOCKED\nREASON=" << error.what() << '\n';
    return 2;
  }
}
