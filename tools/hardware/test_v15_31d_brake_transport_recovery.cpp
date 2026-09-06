// Offline recovery check: real packet codec, scripted transport, no device open.
// Build like the production controller, using this file as the translation unit.
#include <cassert>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

class RecoveryTestSerial {
 public:
  template <typename... Args>
  explicit RecoveryTestSerial(Args&&...) : generation(++opens) {}

  bool sendRecv(MotorCmd* command, MotorData* data) {
    ++reads;
    assert(command->mode == 0 && command->q == 0 && command->dq == 0);
    assert(command->kp == 0 && command->kd == 0 && command->tau == 0);
    if (after_read) after_read();
    // The original fd and the first replacement both disappear. Recovery
    // must reopen a second time instead of polling a dead replacement forever.
    if (generation < 3) return false;
    data->correct = true;
    data->motor_id = command->id;
    data->mode = 0;
    data->q = raw_position;
    data->dq = 0;
    data->tau = 0;
    data->temp = 30;
    data->merror = 0;
    return true;
  }

  static inline int opens = 0;
  static inline int reads = 0;
  static inline float raw_position = 100;
  static inline void (*after_read)() = nullptr;
  int generation;
};

#define SerialPort RecoveryTestSerial
#define main production_controller_main_not_called
#include "v15_30a_gui_go_controller.cpp"
#undef main
#undef SerialPort

int main() {
  alarm(10);  // A missing reopen must fail this test instead of hanging it.
  g_thermal_policy = self_test_thermal_policy();
  command_mask_self_test();  // Existing wire guard rejects FOC in BRAKE-only.
  for (const std::string bus : {"j1", "j2", "j345"}) {
    RecoveryTestSerial::opens = RecoveryTestSerial::reads = 0;
    auto serial = std::make_unique<RecoveryTestSerial>();
    auto motors = make_motors(bus);
    for (auto& motor : motors) {
      motor.reference = 7.0;
      motor.reference_ready = true;
      motor.persistent_reference = 8.0;
      motor.session_reference = 9.0;
      motor.transport_fault_latched = motor.fault_latched = true;
      motor.non_transport_fault_latched = motor.thermal_fault_latched = true;
    }
    TxAudit audit(true);
    std::uint64_t attempts = 0, successes = 0;
    assert(recover_go_transport_in_brake(
        bus_definition(bus), serial, motors, bus, 0.0, audit,
        attempts, successes));
    assert(attempts == 1 && successes == 1 && RecoveryTestSerial::opens == 3);
    assert(audit.foc_serial_send_call_count == 0);
    assert(audit.brake_only_guard_block_count == 0);
    for (const auto& motor : motors) {
      assert(motor.valid && motor.last_frame_valid);
      assert(!motor.transport_fault_latched && motor.consecutive_invalid == 0);
      assert(motor.non_transport_fault_latched && motor.thermal_fault_latched);
      assert(motor.fault_latched && motor.reference_ready);
      assert(motor.reference == 7.0 && motor.persistent_reference == 8.0);
      assert(motor.session_reference == 9.0 && motor.history.empty());
    }
  }

  // Raw acquisition must not relax the active session's pose/phase gate.
  RecoveryTestSerial::opens = 2;
  RecoveryTestSerial::reads = 0;
  RecoveryTestSerial::after_read = [] {
    if (RecoveryTestSerial::reads >= 30) g_stop.store(true);
  };
  std::unique_ptr<RecoveryTestSerial> serial;
  auto motors = make_motors("j1");
  TxAudit active_audit(false);
  std::uint64_t attempts = 0, successes = 0;
  assert(!recover_go_transport_in_brake(
      bus_definition("j1"), serial, motors, "j1", 0.0, active_audit,
      attempts, successes));
  assert(successes == 0 && active_audit.foc_serial_send_call_count == 0);
  std::cout << "BRAKE_TRANSPORT_RECOVERY=PASS\nSERIAL_OPENED=NO\n";
}
