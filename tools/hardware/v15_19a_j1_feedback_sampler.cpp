#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <fstream>
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
  std::string output;
  double hz = 50.0;
  double seconds = 10.0;
  bool execute = false;
};

bool starts_with(const std::string& value, const std::string& prefix) {
  return value.compare(0, prefix.size(), prefix) == 0;
}

std::string wall_timestamp_utc() {
  using Clock = std::chrono::system_clock;
  const auto now = Clock::now();
  const auto epoch_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
          now.time_since_epoch())
          .count();
  const std::time_t seconds = static_cast<std::time_t>(epoch_us / 1000000);
  const long microseconds = static_cast<long>(epoch_us % 1000000);
  std::tm value{};
  gmtime_r(&seconds, &value);
  std::ostringstream stream;
  stream << std::put_time(&value, "%Y-%m-%dT%H:%M:%S") << '.'
         << std::setw(6) << std::setfill('0') << microseconds << 'Z';
  return stream.str();
}

void usage(const char* program) {
  std::cout << "Usage: " << program
            << " --port /dev/serial/by-id/... --output samples.csv"
               " [--hz 50] [--seconds 10] [--execute]\n"
            << "Without --execute, no serial port or output file is opened.\n";
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
    } else if (argument == "--output") {
      options.output = value("--output");
    } else if (argument == "--hz") {
      options.hz = std::stod(value("--hz"));
    } else if (argument == "--seconds") {
      options.seconds = std::stod(value("--seconds"));
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
  if (options.output.empty()) {
    throw std::runtime_error("--output is required");
  }
  if (!starts_with(options.port, "/dev/serial/by-id/") &&
      !starts_with(options.port, "/dev/serial/by-path/")) {
    throw std::runtime_error("--port must use a stable by-id or by-path name");
  }
  if (!std::isfinite(options.hz) || options.hz <= 0.0 ||
      options.hz > 1000.0) {
    throw std::runtime_error("--hz must be finite and in (0,1000]");
  }
  if (!std::isfinite(options.seconds) || options.seconds <= 0.0 ||
      options.seconds > 3600.0) {
    throw std::runtime_error("--seconds must be finite and in (0,3600]");
  }
  return options;
}

void initialize_data(MotorData& data) {
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
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    const int brake_mode =
        queryMotorMode(MotorType::GO_M8010_6, MotorMode::BRAKE);
    const long long requested_count =
        std::llround(options.hz * options.seconds);
    if (requested_count < 1 || requested_count > 10000000) {
      throw std::runtime_error("hz * seconds produced an unsafe sample count");
    }

    std::cout << std::setprecision(17)
              << "PORT=" << options.port << "\n"
              << "OUTPUT=" << options.output << "\n"
              << "MOTOR_TYPE=GO_M8010_6\nMOTOR_ID=0\n"
              << "MODE=BRAKE\nMODE_PROTOCOL_VALUE=" << brake_mode << "\n"
              << "KP=0\nKD=0\nQ=0\nDQ=0\nTAU=0\n"
              << "HZ=" << options.hz << "\nSECONDS=" << options.seconds
              << "\nREQUESTED_COUNT=" << requested_count
              << "\nBAUDRATE=4000000\nTIMEOUT_US=20000\n"
              << "SERIAL_FORMAT=8N1_NO_FLOW_CONTROL\n"
              << "FOC_USED=NO\nCALIBRATION_USED=NO\nID_MODIFIED=NO\n"
              << "EXECUTE=" << (options.execute ? "YES" : "NO") << "\n";
    if (!options.execute) {
      std::cout << "DRY_RUN=YES\nFRAME_SENT=NO\nOUTPUT_CREATED=NO\n";
      return 0;
    }
    if (access(options.port.c_str(), R_OK | W_OK) != 0) {
      throw std::runtime_error("port is not readable and writable");
    }

    std::ofstream output(options.output, std::ios::out | std::ios::trunc);
    if (!output) {
      throw std::runtime_error("could not open output file");
    }
    output << "index,monotonic_seconds,wall_timestamp_utc,send_recv,"
              "data_correct,motor_id,mode,q_raw,dq_raw,tau_raw,"
              "temperature,merror\n";
    output << std::setprecision(17);

    MotorCmd command{};
    command.motorType = MotorType::GO_M8010_6;
    command.mode = static_cast<unsigned short>(brake_mode);
    command.id = 0;
    command.kp = 0.0F;
    command.kd = 0.0F;
    command.q = 0.0F;
    command.dq = 0.0F;
    command.tau = 0.0F;
    SerialPort serial(options.port, 16, 4000000, 20000, BlockYN::NO,
                      bytesize_t::eightbits, parity_t::parity_none,
                      stopbits_t::stopbits_one,
                      flowcontrol_t::flowcontrol_none);

    int sent = 0;
    int received = 0;
    int valid = 0;
    int invalid = 0;
    int timeout = 0;
    int bad_correct = 0;
    int wrong_id = 0;
    int nonfinite = 0;
    int merror_nonzero = 0;
    using SteadyClock = std::chrono::steady_clock;
    const auto start = SteadyClock::now();
    auto next_cycle = start;
    const auto period = std::chrono::duration<double>(1.0 / options.hz);

    for (long long index = 0; index < requested_count; ++index) {
      MotorData data{};
      initialize_data(data);
      ++sent;
      const bool send_recv = serial.sendRecv(&command, &data);
      const auto sample_time = SteadyClock::now();
      const double monotonic_seconds =
          std::chrono::duration<double>(sample_time - start).count();
      const bool finite = std::isfinite(data.q) && std::isfinite(data.dq) &&
                          std::isfinite(data.tau);
      const bool id_matches = static_cast<unsigned>(data.motor_id) == 0U;
      const bool frame_valid =
          send_recv && data.correct && id_matches && finite;
      received += send_recv ? 1 : 0;
      timeout += send_recv ? 0 : 1;
      bad_correct += send_recv && !data.correct ? 1 : 0;
      wrong_id += send_recv && data.correct && !id_matches ? 1 : 0;
      nonfinite += send_recv && data.correct && !finite ? 1 : 0;
      merror_nonzero +=
          send_recv && data.correct && data.merror != 0 ? 1 : 0;
      valid += frame_valid ? 1 : 0;
      invalid += frame_valid ? 0 : 1;
      output << index << ',' << monotonic_seconds << ','
             << wall_timestamp_utc() << ',' << (send_recv ? 1 : 0) << ','
             << (data.correct ? 1 : 0) << ','
             << static_cast<unsigned>(data.motor_id) << ','
             << static_cast<unsigned>(data.mode) << ',' << data.q << ','
             << data.dq << ',' << data.tau << ',' << data.temp << ','
             << data.merror << '\n';
      next_cycle +=
          std::chrono::duration_cast<SteadyClock::duration>(period);
      if (index + 1 < requested_count) {
        std::this_thread::sleep_until(next_cycle);
      }
    }
    output.close();
    const double valid_rate = static_cast<double>(valid) / sent;
    std::cout << std::setprecision(17)
              << "SUMMARY_SENT=" << sent << "\n"
              << "SUMMARY_RECEIVED=" << received << "\n"
              << "SUMMARY_VALID=" << valid << "\n"
              << "SUMMARY_INVALID=" << invalid << "\n"
              << "SUMMARY_TIMEOUT=" << timeout << "\n"
              << "SUMMARY_BAD_CORRECT=" << bad_correct << "\n"
              << "SUMMARY_WRONG_ID=" << wrong_id << "\n"
              << "SUMMARY_NONFINITE=" << nonfinite << "\n"
              << "SUMMARY_MERROR_NONZERO=" << merror_nonzero << "\n"
              << "SUMMARY_VALID_RESPONSE_RATE=" << valid_rate << "\n"
              << "OUTPUT_CREATED=YES\n"
              << "MOTOR_MOTION_CONTROL=NOT_STARTED\n";
    if (valid_rate < 0.999) {
      std::cout << "SAMPLER_VALIDITY_GATE=FAIL\n";
      return 2;
    }
    std::cout << "SAMPLER_VALIDITY_GATE=PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "SAMPLER_ERROR=" << error.what() << "\n";
    return 64;
  }
}
