// Offline wire-consumer check: use the production parser and sample clock.
// Never call the production entry point, open a socket, or construct SerialPort.
#define main production_controller_main_not_called
#include "v15_30a_gui_go_controller.cpp"
#undef main

int main() {
  try {
    nlohmann::json input;
    std::cin >> input;
    const auto& binding = input.at("expected_binding");
    g_expected_gravity_authority_binding = {
        binding.at("authority_class").get<std::string>(),
        binding.at("empirical_envelope_id").get<std::string>(),
        binding.at("empirical_envelope_sha256").get<std::string>(),
        binding.at("anchor_sha256").get<std::string>(),
        binding.at("session_id").get<std::string>(),
        binding.at("state_instance_id").get<std::string>()};
    const auto received_ns = input.at("received_ns").get<std::uint64_t>();
    GuiCommand command;
    parse_command(input.at("packet").get<std::string>(), command,
                  Clock::time_point(std::chrono::nanoseconds(received_ns)));
    nlohmann::json result = {{"serial_opened", false}, {"mode", command.mode}};
    if (command.quintic.present) {
      const auto& trajectory = command.quintic;
      std::array<double, 6> q = trajectory.start_rad;
      std::array<double, 6> dq{};
      nlohmann::json samples = nlohmann::json::array();
      for (const auto index : std::array<std::uint64_t, 3>{
               0U, trajectory.interval_count / 2U, trajectory.interval_count}) {
        const auto stamp = trajectory.execute_at_monotonic_ns +
            trajectory.duration_ns / trajectory.interval_count * index;
        const auto sample = quintic_sample_clock(trajectory, stamp);
        for (std::size_t joint = 0; joint < 6; ++joint) {
          if (command.moving_joint_mask[joint])
            apply_quintic_reference(static_cast<int>(joint), command, sample, q, dq);
        }
        samples.push_back({{"index", sample.sample_index}, {"q_rad", q}, {"dq_rad_s", dq}});
      }
      if (q != trajectory.target_rad)
        throw std::runtime_error("OFFLINE_ENDPOINT_DIFFERS_FROM_ROUTED_TARGET");
      result["trajectory_sha256"] = trajectory.trajectory_sha256;
      result["samples"] = samples;
    }
    std::cout << result.dump() << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "OFFLINE_PACKET_REJECTED: " << error.what() << '\n';
    return 1;
  }
}
