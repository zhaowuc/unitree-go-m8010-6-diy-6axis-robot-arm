#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
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

constexpr char kPort[] =
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if01-port0";
constexpr char kDiscoveryOutput[] = "/tmp/v15_23e_ft_j2_dual_discovery.csv";
constexpr char kLockPath[] = "/tmp/v15_23d_ft_j2_channel1.lock";
constexpr int kBrakeMode = 0;
constexpr int kTemperatureLimitC = 60;

template <typename T>
void zero_object(T& value) {
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
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0U
                ? static_cast<std::uint16_t>((crc >> 1U) ^ 0x8408U)
                : static_cast<std::uint16_t>(crc >> 1U);
    }
  }
  return crc;
}

MotorCmd brake_command(int id) {
  MotorCmd command;
  zero_object(command);
  command.motorType = MotorType::GO_M8010_6;
  command.id = static_cast<unsigned short>(id);
  command.mode = static_cast<unsigned short>(kBrakeMode);
  command.q = 0.0F;
  command.dq = 0.0F;
  command.kp = 0.0F;
  command.kd = 0.0F;
  command.tau = 0.0F;
  command.Res.u32 = 0;
  command.modify_data(&command);
  if (command.hex_len != 17) throw std::runtime_error("COMMAND_LENGTH_INVALID");
  const std::uint8_t* raw = command.get_motor_send_data();
  if (raw == nullptr || raw[0] != 0xfeU || raw[1] != 0xeeU ||
      raw[2] != static_cast<std::uint8_t>(id) ||
      (raw[2] & 0x80U) != 0U ||
      load_u16_le(raw + 15) != crc16_kermit(raw, 15)) {
    throw std::runtime_error("BRAKE_PACKET_AUDIT_FAILED");
  }
  return command;
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

struct Options {
  std::string phase = "discovery";
  std::string label;
  std::string output;
  std::string isolation_gate;
  std::string other_isolated_gate;
  std::string execute_gate;
  std::string dual_connected_gate;
  std::string power_on_gate;
  int motor_id = -1;
  bool self_test = false;
};

Options parse_options(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&]() {
      if (++i >= argc) throw std::runtime_error("CLI_VALUE_MISSING");
      return std::string(argv[i]);
    };
    if (arg == "--self-test") options.self_test = true;
    else if (arg == "--phase") options.phase = value();
    else if (arg == "--motor-label") options.label = value();
    else if (arg == "--motor-id") options.motor_id = std::stoi(value());
    else if (arg == "--output") options.output = value();
    else if (arg == "--only-bus-connected") options.isolation_gate = value();
    else if (arg == "--other-motor-isolated") options.other_isolated_gate = value();
    else if (arg == "--execute-brake-discovery") options.execute_gate = value();
    else if (arg == "--execute-brake-baseline") options.execute_gate = value();
    else if (arg == "--dual-connected-gate") options.dual_connected_gate = value();
    else if (arg == "--power-on-gate") options.power_on_gate = value();
    else throw std::runtime_error("CLI_OPTION_NOT_ALLOWED");
  }
  if (options.self_test) return options;
  if (options.phase != "dual-discovery")
    throw std::runtime_error("PHASE_NOT_ALLOWED");
  if (options.output != kDiscoveryOutput) {
    throw std::runtime_error("OUTPUT_PATH_NOT_ALLOWED");
  }
  if (options.dual_connected_gate !=
          "J2A_ID0_AND_J2B_ID1_DUAL_BUS_CONNECTED_WITH_24V_OFF=YES" ||
      options.power_on_gate != "J2_DUAL_24V_POWER_ON_CONFIRMED=YES") {
    throw std::runtime_error("DISCOVERY_OPERATOR_GATE_MISSING");
  }
  return options;
}

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

struct Sample {
  bool send_recv = false;
  bool valid = false;
  MotorData data;
};

Sample transact(SerialPort& serial, int id) {
  MotorCmd command = brake_command(id);
  Sample sample;
  initialize_feedback(sample.data);
  try {
    sample.send_recv = serial.sendRecv(&command, &sample.data);
  } catch (...) {
    sample.send_recv = false;
  }
  sample.valid = sample.send_recv && sample.data.correct &&
      static_cast<int>(sample.data.motor_id) == id &&
      static_cast<int>(sample.data.mode) == kBrakeMode &&
      sample.data.merror == 0 && sample.data.temp >= 0 &&
      sample.data.temp < kTemperatureLimitC &&
      std::isfinite(sample.data.q) && std::isfinite(sample.data.dq) &&
      std::isfinite(sample.data.tau);
  return sample;
}

void write_row(std::ofstream& csv, const std::string& label, int id,
               int attempt, const Sample& sample, const char* phase) {
  csv << phase << ',' << label << ',' << id << ',' << attempt << ','
      << static_cast<int>(sample.send_recv) << ','
      << static_cast<int>(sample.data.correct) << ','
      << static_cast<int>(sample.data.motor_id) << ','
      << static_cast<int>(sample.data.mode) << ','
      << sample.data.q << ',' << sample.data.dq << ',' << sample.data.tau << ','
      << sample.data.temp << ',' << sample.data.merror << ','
      << static_cast<int>(sample.valid) << '\n';
  csv.flush();
}

void self_test() {
  if (queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE) != kBrakeMode ||
      std::abs(queryGearRatio(MotorType::GO_M8010_6) -
               6.3299999237060547) > 1e-6) {
    throw std::runtime_error("SDK_AUTHORITY_MISMATCH");
  }
  for (int id = 0; id <= 14; ++id) (void)brake_command(id);
    std::cout << "V15_23E_FT_J2_BRAKE_DISCOVERY_SELF_TEST=PASS\n"
            << "SERIAL_PORT_CONSTRUCTED=NO\n"
            << "SCAN_ID_RANGE=0..14\n"
            << "BRAKE_BASELINE_FRAMES=50\n"
            << "SESSION_REFERENCE_FRAMES=100\n"
            << "BRAKE_ONLY_PATH=YES\nFOC_PATH=NO\n"
            << "CALIBRATE_PATH=NO\nID_WRITE_PATH=NO\nZERO_WRITE_PATH=NO\n"
            << "BROADCAST_PATH=NO\n";
}

int run_discovery(const Options& options, SerialPort& serial,
                  std::ofstream& csv) {
  std::set<int> discovered;
  std::array<int, 15> valid_counts{};
  for (int id = 0; id <= 14; ++id) {
    for (int attempt = 1; attempt <= 3; ++attempt) {
      const Sample sample = transact(serial, id);
      write_row(csv, options.label, id, attempt, sample, "DISCOVERY");
      if (sample.valid) {
        discovered.insert(id);
        ++valid_counts.at(static_cast<std::size_t>(id));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  if (discovered.size() != 1U)
    throw std::runtime_error(discovered.empty() ?
        "DISCOVERY_ZERO_MOTORS" : "DISCOVERY_MULTIPLE_MOTORS");
  const int found = *discovered.begin();
  if (valid_counts.at(static_cast<std::size_t>(found)) < 2)
    throw std::runtime_error("DISCOVERY_NOT_STABLE");

  bool final_brake = true;
  for (int attempt = 1; attempt <= 5; ++attempt) {
    const Sample sample = transact(serial, found);
    write_row(csv, options.label, found, attempt, sample, "FINAL_BRAKE");
    final_brake = final_brake && sample.valid;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!final_brake) throw std::runtime_error("FINAL_5_FRAME_BRAKE_FAILED");
  std::cout << "DISCOVERY_RESULT=PASS\nPHYSICAL_LABEL=" << options.label
            << "\nDISCOVERED_COUNT=1\nDISCOVERED_ID=" << found
            << "\nDISCOVERED_VALID_ATTEMPTS="
            << valid_counts.at(static_cast<std::size_t>(found))
            << "\nFINAL_5_FRAME_BRAKE=PASS\nFOC_USED=NO\n"
            << "ID_MODIFIED=NO\nZERO_MODIFIED=NO\n";
  return 0;
}

int run_dual_discovery(SerialPort& serial, std::ofstream& csv) {
  int valid_a = 0;
  int valid_b = 0;
  int consecutive_a = 0;
  int consecutive_b = 0;
  for (int frame = 1; frame <= 100; ++frame) {
    const Sample a = transact(serial, 0);
    write_row(csv, "J2A", 0, frame, a, "DUAL_BRAKE_DISCOVERY");
    if (a.valid) { ++valid_a; consecutive_a = 0; }
    else if (++consecutive_a >= 5)
      throw std::runtime_error("J2A_FIVE_CONSECUTIVE_INVALID");
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    const Sample b = transact(serial, 1);
    write_row(csv, "J2B", 1, frame, b, "DUAL_BRAKE_DISCOVERY");
    if (b.valid) { ++valid_b; consecutive_b = 0; }
    else if (++consecutive_b >= 5)
      throw std::runtime_error("J2B_FIVE_CONSECUTIVE_INVALID");
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  bool final_brake = true;
  for (int frame = 1; frame <= 5; ++frame) {
    const Sample a = transact(serial, 0);
    write_row(csv, "J2A", 0, frame, a, "FINAL_BRAKE");
    const Sample b = transact(serial, 1);
    write_row(csv, "J2B", 1, frame, b, "FINAL_BRAKE");
    final_brake = final_brake && a.valid && b.valid;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (valid_a != 100 || valid_b != 100)
    throw std::runtime_error("DUAL_BRAKE_NOT_100_VALID_EACH");
  if (!final_brake) throw std::runtime_error("FINAL_DUAL_BRAKE_FAILED");
  std::cout << "DUAL_BRAKE_DISCOVERY_RESULT=PASS\n"
            << "J2A_ID=0\nJ2A_VALID=100/100\n"
            << "J2B_ID=1\nJ2B_VALID=100/100\n"
            << "BOTH_MERROR_ZERO=YES\nFINAL_DUAL_5_FRAME_BRAKE=PASS\n"
            << "FOC_USED=NO\nZERO_MODIFIED=NO\n";
  return 0;
}

int run_brake_baseline(const Options& options, SerialPort& serial,
                       std::ofstream& csv) {
  int valid = 0;
  for (int frame = 1; frame <= 50; ++frame) {
    const Sample sample = transact(serial, options.motor_id);
    write_row(csv, options.label, options.motor_id, frame, sample,
              "BRAKE_BASELINE");
    if (sample.valid) ++valid;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  bool final_brake = true;
  for (int frame = 1; frame <= 5; ++frame) {
    const Sample sample = transact(serial, options.motor_id);
    write_row(csv, options.label, options.motor_id, frame, sample,
              "FINAL_BRAKE");
    final_brake = final_brake && sample.valid;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (valid != 50) throw std::runtime_error("BRAKE_BASELINE_NOT_50_VALID");
  if (!final_brake) throw std::runtime_error("FINAL_5_FRAME_BRAKE_FAILED");
  std::cout << "BRAKE_BASELINE_RESULT=PASS\nPHYSICAL_LABEL=" << options.label
            << "\nMOTOR_ID=" << options.motor_id
            << "\nVALID_FRAMES=50/50\nFINAL_5_FRAME_BRAKE=PASS\n"
            << "FOC_USED=NO\nID_MODIFIED=NO\nZERO_MODIFIED=NO\n";
  return 0;
}

double mean(const std::vector<double>& values) {
  double total = 0.0;
  for (double value : values) total += value;
  return total / static_cast<double>(values.size());
}

double median(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2U;
  return values.size() % 2U == 0U ?
      (values[middle - 1U] + values[middle]) / 2.0 : values[middle];
}

double stddev(const std::vector<double>& values) {
  const double average = mean(values);
  double sum = 0.0;
  for (double value : values) {
    const double delta = value - average;
    sum += delta * delta;
  }
  return std::sqrt(sum / static_cast<double>(values.size()));
}

int run_session(const Options& options, SerialPort& serial,
                std::ofstream& csv) {
  std::vector<double> values;
  values.reserve(100U);
  for (int frame = 1; frame <= 100; ++frame) {
    const Sample sample = transact(serial, options.motor_id);
    write_row(csv, options.label, options.motor_id, frame, sample,
              "SESSION_REFERENCE");
    if (sample.valid) values.push_back(sample.data.q);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  bool final_brake = true;
  for (int frame = 1; frame <= 5; ++frame) {
    const Sample sample = transact(serial, options.motor_id);
    write_row(csv, options.label, options.motor_id, frame, sample,
              "FINAL_BRAKE");
    final_brake = final_brake && sample.valid;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (values.size() != 100U)
    throw std::runtime_error("SESSION_REFERENCE_NOT_100_VALID");
  if (!final_brake) throw std::runtime_error("FINAL_5_FRAME_BRAKE_FAILED");
  std::cout << std::setprecision(17)
            << "SESSION_REFERENCE_RESULT=PASS\nPHYSICAL_LABEL=" << options.label
            << "\nMOTOR_ID=" << options.motor_id
            << "\nSESSION_VALID=100/100\nSESSION_RAW_MEDIAN_RAD="
            << median(values) << "\nSESSION_RAW_MEAN_RAD=" << mean(values)
            << "\nSESSION_RAW_STD_RAD=" << stddev(values)
            << "\nSESSION_RAW_MIN_RAD="
            << *std::min_element(values.begin(), values.end())
            << "\nSESSION_RAW_MAX_RAD="
            << *std::max_element(values.begin(), values.end())
            << "\nFINAL_5_FRAME_BRAKE=PASS\nFOC_USED=NO\n"
            << "ID_MODIFIED=NO\nZERO_MODIFIED=NO\n";
  return 0;
}

int run(const Options& options) {
  if (::access(kPort, R_OK | W_OK) != 0)
    throw std::runtime_error("CHANNEL1_STABLE_PORT_NOT_ACCESSIBLE");
  ProcessLock lock;
  SerialPort serial(kPort, 16, 4000000, 20000, BlockYN::NO,
                    bytesize_t::eightbits, parity_t::parity_none,
                    stopbits_t::stopbits_one,
                    flowcontrol_t::flowcontrol_none);
  std::ofstream csv(options.output, std::ios::out | std::ios::trunc);
  if (!csv) throw std::runtime_error("OUTPUT_OPEN_FAILED");
  csv << "phase,physical_label,commanded_id,attempt,send_recv,data_correct,"
         "returned_id,mode,q_raw,dq_raw,tau_raw,temperature_c,merror,valid\n";
  if (options.phase == "dual-discovery")
    return run_dual_discovery(serial, csv);
  if (options.phase == "discovery") return run_discovery(options, serial, csv);
  if (options.phase == "brake-baseline")
    return run_brake_baseline(options, serial, csv);
  return run_session(options, serial, csv);
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    if (options.self_test) {
      self_test();
      return 0;
    }
    return run(options);
  } catch (const std::exception& error) {
    std::cerr << "V15_23E_FT_J2_DUAL_BRAKE_DISCOVERY_RESULT=BLOCKED\nREASON="
              << error.what() << '\n';
    return 2;
  }
}
