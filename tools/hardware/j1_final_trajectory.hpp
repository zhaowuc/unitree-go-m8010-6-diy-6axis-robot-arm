#pragma once

namespace j1_final {

struct Sample {
  double q_deg = 0.0;
  double dq_deg_s = 0.0;
};

struct MotorReference {
  double q_rotor_rad = 0.0;
  double dq_rotor_rad_s = 0.0;
};

// A symmetric, rest-to-rest trapezoid sampled on an integer time grid.
//
// The intervals are:
//   [0, acceleration_intervals) acceleration
//   [acceleration_intervals, acceleration_intervals+cruise_intervals) cruise
//   [acceleration_intervals+cruise_intervals, total_intervals) deceleration
// There are total_intervals + 1 samples, including both endpoints.
struct Profile {
  double q0_deg = 0.0;
  double qf_deg = 0.0;
  double distance_deg = 0.0;
  double direction = 0.0;

  double vmax_deg_s = 0.0;
  double amax_deg_s2 = 0.0;
  double dt_s = 0.0;

  double acceleration_deg_s2 = 0.0;
  double peak_velocity_deg_s = 0.0;

  int acceleration_intervals = 0;
  int cruise_intervals = 0;
  int accel_plus_cruise_intervals = 0;
  int total_intervals = 0;

  // k == 0 is exactly (q0, 0). k >= total_intervals is exactly
  // (qf, 0), which also makes post-profile holds safe and deterministic.
  Sample sample(int k) const;
};

Profile makeProfile(double q0_deg, double qf_deg,
                    double vmax_deg_s = 12.0,
                    double amax_deg_s2 = 40.0,
                    double dt_s = 0.01);

// Maps both references from one joint-space Sample. q_raw_zero_rad is the
// session zero in rotor radians and gear_ratio must be positive.
MotorReference mapToMotor(const Sample& joint_sample,
                          double q_raw_zero_rad, double gear_ratio);

// Pure mathematical tests. Throws std::runtime_error on the first failure.
// It never constructs a driver and never performs I/O.
void runTrajectorySelfTests();

}  // namespace j1_final
