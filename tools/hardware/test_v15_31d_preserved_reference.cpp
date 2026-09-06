// Offline consumer for issuer/driver cross-checks; never enters production run().
#define main production_controller_main_not_called
#include "v15_30a_gui_go_controller.cpp"
#undef main

int main() {
  try {
    nlohmann::json document;
    std::cin >> document;
    const bool j2 = document.at("schema") == "go-m8010-j2-power-session-reference/1.0";
    const auto zero = document.at("parent_persistent_zero_sha256").get<std::string>();
    const auto hints = document.at("preserved_inputs").at("recovery_branch_hints_sha256").get<std::string>();
    const auto session = document.at("raw_capture").at("power_session_id").get<std::string>();
    const auto boot = document.at("raw_capture").at("host_boot_id").get<std::string>();
    auto motors = make_motors(j2 ? "j2" : "j1");
    if (!j2) {
      auto wrist = make_motors("j345");
      motors.insert(motors.end(), wrist.begin(), wrist.end());
    }
    for (auto& motor : motors) motor.recovery_hint_configured = true;
    if (j2)
      apply_j2_session_reference_document(document, zero, hints, session, boot, motors);
    else
      apply_go_aux_session_reference_document(document, zero, hints, session, boot, motors);
    nlohmann::json result = {{"serial_opened", false}, {"motors", nlohmann::json::object()}};
    for (const auto& motor : motors) {
      const double reference = reference_for_j2_session(motor.session_capture_raw_position, motor);
      bool drift_rejected = false;
      if (motor.supported_near_vertical_recovery) {
        try {
          (void)reference_for_j2_session(motor.session_capture_raw_position +
              motor.sign * kGear * 2.01 * kPi / 180.0, motor);
        } catch (const std::runtime_error&) { drift_rejected = true; }
      }
      result["motors"][motor.name] = {
          {"reference", motor.session_reference},
          {"logical_hint", motor.session_logical_position},
          {"startup_logical", motor.session_startup_logical_position},
          {"runtime_logical", motor.sign * (motor.session_capture_raw_position - reference) / kGear},
          {"runtime_two_degree_drift_rejected", drift_rejected}};
    }
    std::cout << result.dump() << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
