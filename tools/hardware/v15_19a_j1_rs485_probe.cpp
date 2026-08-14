#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <unistd.h>

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

namespace {

struct Options {
  std::string port;
  unsigned short id = 0;
  int count = 1;
  double hz = 10.0;
  bool execute = false;
};

std::string utc_timestamp() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t value = std::chrono::system_clock::to_time_t(now);
  std::tm tm_value{};
  gmtime_r(&value, &tm_value);
  std::ostringstream stream;
  stream << std::put_time(&tm_value, "%Y-%m-%dT%H:%M:%SZ");
  return stream.str();
}

bool starts_with(const std::string& value, const std::string& prefix) {
  return value.compare(0, prefix.size(), prefix) == 0;
}

void usage(const char* program) {
  std::cout << "Usage: " << program
            << " --port /dev/serial/by-id/... [--id 0] [--count 1]"
               " [--hz 10] [--execute]\n"
            << "Without --execute this is a no-I/O dry-run.\n";
}

Options parse_options(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    const auto value = [&](const char* name) {
      if (index + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + name);
      }
      return std::string(argv[++index]);
    };
    if (argument == "--port") {
      options.port = value("--port");
    } else if (argument == "--id") {
      const long id = std::stol(value("--id"));
      if (id < 0 || id > 255) {
        throw std::runtime_error("--id must be in [0,255]");
      }
      options.id = static_cast<unsigned short>(id);
    } else if (argument == "--count") {
      options.count = std::stoi(value("--count"));
    } else if (argument == "--hz") {
      options.hz = std::stod(value("--hz"));
    } else if (argument == "--execute") {
      options.execute = true;
    } else if (argument == "-h" || argument == "--help") {
      usage(argv[0]);
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + argument);
    }
  }
  if (options.port.empty()) {
    throw std::runtime_error("--port is required");
  }
  if (!starts_with(options.port, "/dev/serial/by-id/") &&
      !starts_with(options.port, "/dev/serial/by-path/")) {
    throw std::runtime_error("--port must use a stable by-id or by-path name");
  }
  if (options.count < 1 || options.count > 10000) {
    throw std::runtime_error("--count must be in [1,10000]");
  }
  if (!std::isfinite(options.hz) || options.hz <= 0.0 ||
      options.hz > 1000.0) {
    throw std::runtime_error("--hz must be finite and in (0,1000]");
  }
  return options;
}

void print_frame(int index, bool send_recv, const MotorData& data) {
  std::cout << std::setprecision(17)
            << "FRAME_INDEX=" << index << "\n"
            << "SENDRECV=" << (send_recv ? "true" : "false") << "\n"
            << "DATA_CORRECT=" << (data.correct ? "true" : "false") << "\n"
            << "DATA_MOTOR_ID=" << static_cast<unsigned>(data.motor_id) << "\n"
            << "DATA_MODE=" << static_cast<unsigned>(data.mode) << "\n"
            << "DATA_Q=" << data.q << "\n"
            << "DATA_DQ=" << data.dq << "\n"
            << "DATA_TAU=" << data.tau << "\n"
            << "DATA_TEMP=" << data.temp << "\n"
            << "DATA_MERROR=" << data.merror << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    const int brake_mode =
        queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE);
    std::cout << std::setprecision(17)
              << "PORT=" << options.port << "\n"
              << "MOTOR_TYPE=GO_M8010_6\n"
              << "MOTOR_ID=" << options.id << "\n"
              << "MODE=BRAKE\n"
              << "MODE_PROTOCOL_VALUE=" << brake_mode << "\n"
              << "KP=0\nKD=0\nQ=0\nDQ=0\nTAU=0\n"
              << "COUNT=" << options.count << "\n"
              << "HZ=" << options.hz << "\n"
              << "BAUDRATE=4000000\nTIMEOUT_US=20000\n"
              << "SERIAL_FORMAT=8N1_NO_FLOW_CONTROL\n"
              << "FOC_USED=NO\nCALIBRATION_USED=NO\nID_MODIFIED=NO\n"
              << "EXECUTE=" << (options.execute ? "YES" : "NO") << "\n";
    if (!options.execute) {
      std::cout << "DRY_RUN=YES\nFRAME_SENT=NO\n"
                   "OFFICIAL_EXAMPLE_DIRECT_EXECUTION=FORBIDDEN\n";
      return 0;
    }
    if (access(options.port.c_str(), R_OK | W_OK) != 0) {
      throw std::runtime_error("port is not readable and writable");
    }

    MotorCmd command{};
    command.motorType = MotorType::GO_M8010_6;
    command.mode = static_cast<unsigned short>(brake_mode);
    command.id = options.id;
    command.kp = 0.0F;
    command.kd = 0.0F;
    command.q = 0.0F;
    command.dq = 0.0F;
    command.tau = 0.0F;
    SerialPort serial(options.port, 16, 4000000, 20000, BlockYN::NO,
                      bytesize_t::eightbits, parity_t::parity_none,
                      stopbits_t::stopbits_one,
                      flowcontrol_t::flowcontrol_none);

    int sent = 0, received = 0, valid = 0, invalid = 0, timeout = 0;
    int bad_correct = 0, wrong_id = 0, merror_nonzero = 0;
    bool have_q = false;
    float q_first = 0.0F, q_last = 0.0F;
    float q_min = std::numeric_limits<float>::infinity();
    float q_max = -std::numeric_limits<float>::infinity();
    float max_step = 0.0F;
    int temp_min = std::numeric_limits<int>::max();
    int temp_max = std::numeric_limits<int>::min();
    using Clock = std::chrono::steady_clock;
    auto next_cycle = Clock::now();
    const auto period = std::chrono::duration<double>(1.0 / options.hz);

    for (int index = 1; index <= options.count; ++index) {
      MotorData data{};
      data.motorType = MotorType::GO_M8010_6;
      data.hex_len = 0;
      data.motor_id = 0;
      data.mode = 0;
      data.temp = 0;
      data.merror = 0;
      data.tau = std::numeric_limits<float>::quiet_NaN();
      data.dq = std::numeric_limits<float>::quiet_NaN();
      data.q = std::numeric_limits<float>::quiet_NaN();
      data.correct = false;
      ++sent;
      const bool send_recv = serial.sendRecv(&command, &data);
      received += send_recv ? 1 : 0;
      timeout += send_recv ? 0 : 1;
      print_frame(index, send_recv, data);
      const bool finite = std::isfinite(data.q) && std::isfinite(data.dq) &&
                          std::isfinite(data.tau);
      const bool id_matches = static_cast<unsigned>(data.motor_id) == options.id;
      const bool frame_valid =
          send_recv && data.correct && id_matches && finite;
      bad_correct += send_recv && !data.correct ? 1 : 0;
      wrong_id += send_recv && data.correct && !id_matches ? 1 : 0;
      merror_nonzero += send_recv && data.correct && data.merror != 0 ? 1 : 0;
      if (!frame_valid) {
        ++invalid;
      } else {
        ++valid;
        if (!have_q) {
          have_q = true;
          q_first = q_last = q_min = q_max = data.q;
          temp_min = temp_max = data.temp;
          std::cout << "FIRST_VALID_J1_FEEDBACK\n"
                    << "FIRST_VALID_PORT=" << options.port << "\n"
                    << "FIRST_VALID_ID=" << static_cast<unsigned>(data.motor_id)
                    << "\nFIRST_VALID_Q=" << data.q
                    << "\nFIRST_VALID_DQ=" << data.dq
                    << "\nFIRST_VALID_TAU=" << data.tau
                    << "\nFIRST_VALID_TEMPERATURE=" << data.temp
                    << "\nFIRST_VALID_MERROR=" << data.merror
                    << "\nFIRST_VALID_MODE=" << static_cast<unsigned>(data.mode)
                    << "\nFIRST_VALID_TIMESTAMP=" << utc_timestamp()
                    << "\nPOST_FIRST_FRAME_OBSERVATION_SECONDS=2\n";
          std::this_thread::sleep_for(std::chrono::seconds(2));
          next_cycle = Clock::now();
        } else {
          max_step = std::max(max_step, std::fabs(data.q - q_last));
          q_last = data.q;
          q_min = std::min(q_min, data.q);
          q_max = std::max(q_max, data.q);
          temp_min = std::min(temp_min, data.temp);
          temp_max = std::max(temp_max, data.temp);
        }
      }
      if (index < options.count) {
        next_cycle += std::chrono::duration_cast<Clock::duration>(period);
        std::this_thread::sleep_until(next_cycle);
      }
    }

    const double rate = static_cast<double>(valid) / sent;
    std::cout << std::setprecision(17)
              << "SUMMARY_SENT=" << sent << "\nSUMMARY_RECEIVED=" << received
              << "\nSUMMARY_VALID=" << valid << "\nSUMMARY_INVALID=" << invalid
              << "\nSUMMARY_TIMEOUT=" << timeout
              << "\nSUMMARY_BAD_CORRECT=" << bad_correct
              << "\nSUMMARY_WRONG_ID=" << wrong_id
              << "\nSUMMARY_MERROR_NONZERO=" << merror_nonzero
              << "\nSUMMARY_VALID_RESPONSE_RATE=" << rate << "\n";
    if (have_q) {
      std::cout << "Q_FIRST=" << q_first << "\nQ_LAST=" << q_last
                << "\nQ_MIN=" << q_min << "\nQ_MAX=" << q_max
                << "\nQ_MAX_SINGLE_STEP_CHANGE=" << max_step
                << "\nTEMP_MIN=" << temp_min << "\nTEMP_MAX=" << temp_max
                << "\n";
    }
    if (valid == 0) {
      std::cout << "J1_ID0_NO_RESPONSE=YES\nCOMMUNICATION_FAIL=YES\n";
      return 2;
    }
    if (max_step > 0.05F) {
      std::cout << "ENCODER_FEEDBACK_SUSPECT=YES\n";
      return 3;
    }
    if (rate < 0.90) {
      std::cout << "COMMUNICATION_FAIL=YES\n";
      return 4;
    }
    if (rate < 0.99) {
      std::cout << "COMMUNICATION_UNSTABLE=YES\n";
      return 5;
    }
    std::cout << "COMMUNICATION_SAMPLE=PASS\n"
                 "RAW_MOTOR_POSITION_ONLY=YES\n"
                 "MOTOR_MOTION_CONTROL=NOT_STARTED\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "PROBE_ERROR=" << error.what()
              << "\nCOMMUNICATION_SAMPLE=FAIL\n";
    return 64;
  }
}
