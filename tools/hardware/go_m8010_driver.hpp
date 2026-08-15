#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace go_m8010 {

struct DriverConfig {
  std::string port;
  int motor_id = 0;
  double gear_ratio = 6.3299999237060547;
  int direction_sign = 1;
  double temperature_limit_c = 55.0;
  double joint_envelope_rad = 0.19198621771937624;  // 11 deg
};

struct State {
  std::uint64_t sequence = 0;
  double monotonic_s = 0.0;
  bool send_recv = false;
  bool correct = false;
  unsigned motor_id = 255;
  unsigned mode = 255;
  double q_raw = 0.0;
  double q_raw_unwrapped = 0.0;
  double q_joint = 0.0;
  double dq_sdk = 0.0;
  double tau_sdk = 0.0;
  int temperature_c = 0;
  int merror = 0;
  double qdot_joint_single = 0.0;
  double qdot_joint_fast = 0.0;
  double qdot_joint_slow = 0.0;
  double qdot_rotor_fd = 0.0;
  double dt_s = 0.0;
  bool velocity_valid = false;
  bool feedback_valid = false;
  bool joint_envelope_ok = true;
};

struct CommandState {
  int mode = 0;
  double tau_rotor_nm = 0.0;
  double dq_rotor_rad_s = 0.0;
  double q_rotor_rad = 0.0;
  double kp = 0.0;
  double kw = 0.0;
  int encoded_tau_count = 0;
  int encoded_kpos = 0;
  int encoded_kspd = 0;
};

// Exercises the frozen Unitree SDK serializer entirely in memory.  This
// function never constructs a SerialPort and performs no device I/O.  It
// returns normally on success and throws std::runtime_error on any ABI,
// packet, reserved-bit, field-encoding, or CRC mismatch.
void runFrozenSdkPacketSelfTest();

class GoM8010Driver {
 public:
  explicit GoM8010Driver(DriverConfig config);
  ~GoM8010Driver();
  GoM8010Driver(const GoM8010Driver&) = delete;
  GoM8010Driver& operator=(const GoM8010Driver&) = delete;

  void connect();
  void disconnect() noexcept;
  State readTelemetry();
  State readState();
  void brake();
  void enterFoc();
  void enterPositionMode(double kp, double kd);
  void commandTorqueRotorNm(double torque_nm);
  void commandRotorState(double torque_nm, double dq_rotor_rad_s,
                         double q_rotor_rad, double kp, double kw);
  void setPositionControl(double kp, double kw, double torque_ff_rotor_nm);
  void commandJointPositionRad(double q_joint_rad);
  void commandJointState(double position_rad, double velocity_rad_s);
  void setSessionReference(double q_raw_reference);
  void setSessionZero(double q_raw_reference);
  bool emergencyBrake() noexcept;
  bool healthy() const noexcept;
  bool referenceSet() const noexcept;
  int brakeMode() const noexcept;
  int focMode() const noexcept;
  double sessionReferenceRaw() const;
  const CommandState& commandState() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace go_m8010
