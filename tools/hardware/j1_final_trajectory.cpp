#include "j1_final_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace j1_final {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kGoldenTolerance = 1.0e-12;
constexpr double kPositionTolerance = 1.0e-12;
constexpr double kVelocityTolerance = 1.0e-12;
constexpr double kAccelerationTolerance = 1.0e-9;
constexpr double kKinematicTolerance = 1.0e-10;

[[noreturn]] void fail(const std::string& message) {
  throw std::runtime_error("J1_TRAJECTORY_SELF_TEST_FAILED: " + message);
}

void require(bool condition, const std::string& message) {
  if (!condition) {
    fail(message);
  }
}

bool near(double lhs, double rhs, double absolute_tolerance,
          double relative_tolerance = 0.0) {
  const double scale = std::max(std::abs(lhs), std::abs(rhs));
  return std::abs(lhs - rhs) <=
         absolute_tolerance + relative_tolerance * scale;
}

void requireNear(double lhs, double rhs, double absolute_tolerance,
                 const std::string& message,
                 double relative_tolerance = 0.0) {
  if (!near(lhs, rhs, absolute_tolerance, relative_tolerance)) {
    std::ostringstream stream;
    stream.precision(17);
    stream << message << " (actual=" << lhs << ", expected=" << rhs << ')';
    fail(stream.str());
  }
}

void verifyProfileProperties(const Profile& profile) {
  require(profile.total_intervals >= 0, "negative total interval count");
  require(profile.acceleration_intervals >= 0,
          "negative acceleration count");
  require(profile.cruise_intervals >= 0, "negative cruise count");
  require(profile.accel_plus_cruise_intervals ==
              profile.acceleration_intervals + profile.cruise_intervals,
          "M != Na + Nc");
  require(profile.total_intervals ==
              profile.acceleration_intervals +
                  profile.accel_plus_cruise_intervals,
          "N != Na + M");

  const Sample first = profile.sample(0);
  const Sample last = profile.sample(profile.total_intervals);
  require(first.q_deg == profile.q0_deg, "initial position is not exact");
  require(first.dq_deg_s == 0.0, "initial velocity is not exactly zero");
  require(last.q_deg == profile.qf_deg, "final position is not exact");
  require(last.dq_deg_s == 0.0, "final velocity is not exactly zero");
  const Sample after = profile.sample(profile.total_intervals + 17);
  require(after.q_deg == profile.qf_deg,
          "post-profile hold position is not exact");
  require(after.dq_deg_s == 0.0,
          "post-profile hold velocity is not exactly zero");

  if (profile.total_intervals == 0) {
    require(profile.distance_deg == 0.0,
            "zero-interval profile has nonzero distance");
    return;
  }

  require(profile.acceleration_intervals > 0,
          "moving profile has no acceleration intervals");
  require(profile.acceleration_deg_s2 > 0.0,
          "moving profile has nonpositive acceleration");
  require(profile.peak_velocity_deg_s > 0.0,
          "moving profile has nonpositive peak velocity");
  require(profile.acceleration_deg_s2 <=
              profile.amax_deg_s2 + kAccelerationTolerance,
          "planned acceleration exceeds limit");
  require(profile.peak_velocity_deg_s <=
              profile.vmax_deg_s + kVelocityTolerance,
          "planned velocity exceeds limit");

  const double position_scale =
      std::max({1.0, std::abs(profile.q0_deg), std::abs(profile.qf_deg),
                profile.distance_deg});
  const double position_tolerance =
      kPositionTolerance * position_scale;

  Sample previous = first;
  for (int k = 1; k <= profile.total_intervals; ++k) {
    const Sample current = profile.sample(k);
    require(std::isfinite(current.q_deg) &&
                std::isfinite(current.dq_deg_s),
            "non-finite trajectory sample");

    const double directed_step =
        profile.direction * (current.q_deg - previous.q_deg);
    require(directed_step >= -position_tolerance,
            "profile is not monotonic");

    const double directed_position =
        profile.direction * (current.q_deg - profile.q0_deg);
    require(directed_position >= -position_tolerance &&
                directed_position <=
                    profile.distance_deg + position_tolerance,
            "profile overshoots its endpoints");

    require(std::abs(current.dq_deg_s) <=
                profile.vmax_deg_s + kVelocityTolerance,
            "sample velocity exceeds limit");

    const double discrete_acceleration =
        (current.dq_deg_s - previous.dq_deg_s) / profile.dt_s;
    require(std::abs(discrete_acceleration) <=
                profile.amax_deg_s2 + kAccelerationTolerance,
            "sample acceleration exceeds limit");

    const double expected_step =
        0.5 * (previous.dq_deg_s + current.dq_deg_s) * profile.dt_s;
    requireNear(current.q_deg - previous.q_deg, expected_step,
                kKinematicTolerance * position_scale,
                "q and dq are not from the same discrete state");
    previous = current;
  }
}

void expectInvalidProfile(double q0_deg, double qf_deg, double vmax_deg_s,
                          double amax_deg_s2, double dt_s) {
  bool threw = false;
  try {
    (void)makeProfile(q0_deg, qf_deg, vmax_deg_s, amax_deg_s2, dt_s);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  require(threw, "invalid profile input was accepted");
}

void goldenTenDegreeTest() {
  const Profile profile = makeProfile(0.0, 10.0);
  require(profile.acceleration_intervals == 30, "10deg Na != 30");
  require(profile.cruise_intervals == 54, "10deg Nc != 54");
  require(profile.accel_plus_cruise_intervals == 84,
          "10deg M != 84");
  require(profile.total_intervals == 114, "10deg N != 114");
  requireNear(profile.acceleration_deg_s2, 39.682539682539684,
              kGoldenTolerance, "10deg acceleration mismatch");
  requireNear(profile.peak_velocity_deg_s, 11.904761904761905,
              kGoldenTolerance, "10deg peak velocity mismatch");

  const Sample accel_end = profile.sample(30);
  requireNear(accel_end.q_deg, 1.7857142857142858, kGoldenTolerance,
              "10deg acceleration boundary position mismatch");
  requireNear(accel_end.dq_deg_s, profile.peak_velocity_deg_s,
              kGoldenTolerance,
              "10deg acceleration boundary velocity mismatch");

  const Sample decel_start = profile.sample(84);
  requireNear(decel_start.q_deg, 8.214285714285715, kGoldenTolerance,
              "10deg deceleration boundary position mismatch");
  requireNear(decel_start.dq_deg_s, profile.peak_velocity_deg_s,
              kGoldenTolerance,
              "10deg deceleration boundary velocity mismatch");

  requireNear(profile.total_intervals * profile.dt_s, 1.14,
              kGoldenTolerance, "10deg duration mismatch");
  verifyProfileProperties(profile);
}

void boundaryTests() {
  const Profile zero = makeProfile(7.25, 7.25);
  require(zero.total_intervals == 0, "zero move has intervals");
  require(zero.sample(0).q_deg == 7.25, "zero move changed position");
  require(zero.sample(100).dq_deg_s == 0.0,
          "zero move has post-profile velocity");
  verifyProfileProperties(zero);

  const Profile tiny = makeProfile(0.0, 1.0e-9);
  require(tiny.acceleration_intervals == 1 &&
              tiny.cruise_intervals == 0 &&
              tiny.total_intervals == 2,
          "tiny move is not a two-interval triangle");
  verifyProfileProperties(tiny);

  const Profile one_degree = makeProfile(-4.125, -3.125);
  require(one_degree.acceleration_intervals == 16 &&
              one_degree.cruise_intervals == 0 &&
              one_degree.total_intervals == 32,
          "1deg triangular profile counts mismatch");
  verifyProfileProperties(one_degree);

  const Profile transition = makeProfile(0.0, 3.6);
  require(transition.acceleration_intervals == 30 &&
              transition.cruise_intervals == 0 &&
              transition.total_intervals == 60,
          "3.6deg transition profile counts mismatch");
  requireNear(transition.acceleration_deg_s2, 40.0, kGoldenTolerance,
              "3.6deg transition acceleration mismatch");
  requireNear(transition.peak_velocity_deg_s, 12.0, kGoldenTolerance,
              "3.6deg transition velocity mismatch");
  verifyProfileProperties(transition);
  verifyProfileProperties(makeProfile(0.0, 3.6 - 1.0e-9));
  verifyProfileProperties(makeProfile(0.0, 3.6 + 1.0e-9));
  verifyProfileProperties(makeProfile(7.3, -2.7));
  verifyProfileProperties(makeProfile(-10.0, 10.0));

  expectInvalidProfile(std::numeric_limits<double>::quiet_NaN(), 0.0,
                       12.0, 40.0, 0.01);
  expectInvalidProfile(0.0, std::numeric_limits<double>::infinity(),
                       12.0, 40.0, 0.01);
  expectInvalidProfile(0.0, 1.0, 0.0, 40.0, 0.01);
  expectInvalidProfile(0.0, 1.0, 12.0, -40.0, 0.01);
  expectInvalidProfile(0.0, 1.0, 12.0, 40.0, 0.0);
}

void mirrorTests() {
  const std::vector<double> distances = {1.0e-9, 0.001, 1.0, 3.6,
                                         3.600000001, 10.0, 20.0};
  for (double distance : distances) {
    const Profile positive = makeProfile(0.0, distance);
    const Profile negative = makeProfile(0.0, -distance);
    require(positive.acceleration_intervals ==
                    negative.acceleration_intervals &&
                positive.cruise_intervals == negative.cruise_intervals &&
                positive.total_intervals == negative.total_intervals,
            "mirrored profiles have different interval counts");
    requireNear(positive.acceleration_deg_s2,
                negative.acceleration_deg_s2, kGoldenTolerance,
                "mirrored accelerations differ");
    requireNear(positive.peak_velocity_deg_s,
                negative.peak_velocity_deg_s, kGoldenTolerance,
                "mirrored peak velocities differ");
    for (int k = 0; k <= positive.total_intervals; ++k) {
      const Sample plus = positive.sample(k);
      const Sample minus = negative.sample(k);
      requireNear(plus.q_deg, -minus.q_deg, kKinematicTolerance,
                  "mirrored positions differ");
      requireNear(plus.dq_deg_s, -minus.dq_deg_s,
                  kKinematicTolerance, "mirrored velocities differ");
    }
  }
}

void randomizedPropertyTests() {
  std::mt19937_64 generator(0x4a315f5452414aULL);
  std::uniform_real_distribution<double> offset_distribution(-1000.0,
                                                              1000.0);
  std::uniform_real_distribution<double> exponent_distribution(
      -8.0, std::log10(200.0));
  std::bernoulli_distribution direction_distribution(0.5);

  for (int iteration = 0; iteration < 10000; ++iteration) {
    const double q0 = offset_distribution(generator);
    const double distance =
        std::pow(10.0, exponent_distribution(generator));
    const double signed_distance =
        direction_distribution(generator) ? distance : -distance;
    const Profile profile = makeProfile(q0, q0 + signed_distance);
    verifyProfileProperties(profile);
  }
}

void concatenationTest() {
  std::vector<Sample> sequence;

  const auto append_leg = [&sequence](double q0_deg, double qf_deg) {
    const Profile profile = makeProfile(q0_deg, qf_deg);
    const int first_k = sequence.empty() ? 0 : 1;
    for (int k = first_k; k <= profile.total_intervals; ++k) {
      sequence.push_back(profile.sample(k));
    }
  };
  const auto append_hold = [&sequence](int intervals) {
    require(!sequence.empty(), "cannot hold an empty sequence");
    const Sample endpoint = sequence.back();
    require(endpoint.dq_deg_s == 0.0,
            "hold did not start at zero reference velocity");
    for (int interval = 0; interval < intervals; ++interval) {
      sequence.push_back(endpoint);
    }
  };

  append_leg(0.0, 10.0);
  append_hold(30);
  append_leg(10.0, 0.0);
  append_hold(20);
  append_leg(0.0, -10.0);
  append_hold(30);
  append_leg(-10.0, 0.0);
  append_hold(20);

  require(sequence.size() == 557,
          "full sequence is not 557 samples / 556 intervals");
  requireNear((sequence.size() - 1) * 0.01, 5.56,
              kGoldenTolerance, "full sequence duration mismatch");

  const struct ExpectedBoundary {
    std::size_t index;
    double q_deg;
  } expected_boundaries[] = {{0, 0.0},   {114, 10.0}, {144, 10.0},
                             {258, 0.0}, {278, 0.0},  {392, -10.0},
                             {422, -10.0}, {536, 0.0}, {556, 0.0}};
  for (const ExpectedBoundary& expected : expected_boundaries) {
    require(sequence[expected.index].q_deg == expected.q_deg,
            "concatenated boundary position mismatch");
    require(sequence[expected.index].dq_deg_s == 0.0,
            "concatenated boundary velocity mismatch");
  }

  for (std::size_t index = 1; index < sequence.size(); ++index) {
    const Sample& previous = sequence[index - 1];
    const Sample& current = sequence[index];
    const double expected_step =
        0.5 * (previous.dq_deg_s + current.dq_deg_s) * 0.01;
    requireNear(current.q_deg - previous.q_deg, expected_step,
                kKinematicTolerance,
                "concatenated q/dq state mismatch");
  }
}

void motorMappingTest() {
  constexpr double kGearRatio = 6.3299999237060547;
  constexpr double kRawZero = -0.73125;
  const Profile profile = makeProfile(7.3, -2.7);
  const Sample joint = profile.sample(17);
  require(joint.dq_deg_s != 0.0,
          "motor mapping test accidentally selected zero velocity");
  const MotorReference motor = mapToMotor(joint, kRawZero, kGearRatio);
  const double expected_q =
      kRawZero + joint.q_deg * (kPi / 180.0) * kGearRatio;
  const double expected_dq =
      joint.dq_deg_s * (kPi / 180.0) * kGearRatio;
  requireNear(motor.q_rotor_rad, expected_q, kGoldenTolerance,
              "motor q is not mapped from the trajectory sample");
  requireNear(motor.dq_rotor_rad_s, expected_dq, kGoldenTolerance,
              "motor dq is not mapped from the trajectory sample");

  const double recovered_q_deg =
      (motor.q_rotor_rad - kRawZero) / kGearRatio * (180.0 / kPi);
  const double recovered_dq_deg_s =
      motor.dq_rotor_rad_s / kGearRatio * (180.0 / kPi);
  requireNear(recovered_q_deg, joint.q_deg, kKinematicTolerance,
              "motor position round trip mismatch");
  requireNear(recovered_dq_deg_s, joint.dq_deg_s,
              kKinematicTolerance, "motor velocity round trip mismatch");

  bool threw = false;
  try {
    (void)mapToMotor(joint, kRawZero, 0.0);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  require(threw, "zero gear ratio was accepted");
}

}  // namespace

Sample Profile::sample(int k) const {
  if (k < 0) {
    throw std::out_of_range("trajectory sample index must be nonnegative");
  }
  if (k == 0) {
    return {q0_deg, 0.0};
  }
  if (total_intervals == 0 || k >= total_intervals) {
    return {qf_deg, 0.0};
  }

  double x_deg = 0.0;
  double speed_deg_s = 0.0;
  if (k <= acceleration_intervals) {
    const double t_s = static_cast<double>(k) * dt_s;
    x_deg = 0.5 * acceleration_deg_s2 * t_s * t_s;
    speed_deg_s = acceleration_deg_s2 * t_s;
  } else if (k <= accel_plus_cruise_intervals) {
    const double t_s = static_cast<double>(k) * dt_s;
    const double accel_time_s =
        static_cast<double>(acceleration_intervals) * dt_s;
    x_deg = 0.5 * acceleration_deg_s2 * accel_time_s * accel_time_s +
            peak_velocity_deg_s * (t_s - accel_time_s);
    speed_deg_s = peak_velocity_deg_s;
  } else {
    const double remaining_time_s =
        static_cast<double>(total_intervals - k) * dt_s;
    x_deg = distance_deg -
            0.5 * acceleration_deg_s2 * remaining_time_s *
                remaining_time_s;
    speed_deg_s = acceleration_deg_s2 * remaining_time_s;
  }

  return {q0_deg + direction * x_deg, direction * speed_deg_s};
}

Profile makeProfile(double q0_deg, double qf_deg, double vmax_deg_s,
                    double amax_deg_s2, double dt_s) {
  if (!std::isfinite(q0_deg) || !std::isfinite(qf_deg) ||
      !std::isfinite(vmax_deg_s) || !std::isfinite(amax_deg_s2) ||
      !std::isfinite(dt_s) || vmax_deg_s <= 0.0 ||
      amax_deg_s2 <= 0.0 || dt_s <= 0.0) {
    throw std::invalid_argument(
        "trajectory inputs must be finite and limits/dt must be positive");
  }

  Profile profile;
  profile.q0_deg = q0_deg;
  profile.qf_deg = qf_deg;
  profile.vmax_deg_s = vmax_deg_s;
  profile.amax_deg_s2 = amax_deg_s2;
  profile.dt_s = dt_s;
  profile.distance_deg = std::abs(qf_deg - q0_deg);
  if (!std::isfinite(profile.distance_deg)) {
    throw std::invalid_argument("trajectory distance overflowed");
  }
  if (profile.distance_deg == 0.0) {
    return profile;
  }
  profile.direction = qf_deg > q0_deg ? 1.0 : -1.0;

  const double velocity_intervals =
      profile.distance_deg / (vmax_deg_s * dt_s);
  const double acceleration_area =
      profile.distance_deg / (amax_deg_s2 * dt_s * dt_s);
  if (!std::isfinite(velocity_intervals) ||
      !std::isfinite(acceleration_area)) {
    throw std::overflow_error("trajectory interval calculation overflowed");
  }

  const double upper_double = std::ceil(std::sqrt(acceleration_area)) + 1.0;
  if (!std::isfinite(upper_double) ||
      upper_double > static_cast<double>(std::numeric_limits<int>::max())) {
    throw std::overflow_error("trajectory requires too many intervals");
  }
  const int upper_accel_intervals =
      std::max(1, static_cast<int>(upper_double));

  int best_accel_intervals = 0;
  int best_accel_plus_cruise_intervals = 0;
  int best_total_intervals = std::numeric_limits<int>::max();

  for (int accel_intervals = 1;
       accel_intervals <= upper_accel_intervals; ++accel_intervals) {
    const double initial_m_double =
        std::max({static_cast<double>(accel_intervals), 1.0,
                  std::floor(velocity_intervals),
                  std::floor(acceleration_area /
                             static_cast<double>(accel_intervals))});
    if (!std::isfinite(initial_m_double) ||
        initial_m_double >
            static_cast<double>(std::numeric_limits<int>::max())) {
      continue;
    }

    int accel_plus_cruise_intervals =
        static_cast<int>(initial_m_double);
    while (true) {
      const double peak_velocity =
          profile.distance_deg /
          (dt_s * static_cast<double>(accel_plus_cruise_intervals));
      const double acceleration =
          profile.distance_deg /
          (dt_s * dt_s * static_cast<double>(accel_intervals) *
           static_cast<double>(accel_plus_cruise_intervals));
      if (peak_velocity <= vmax_deg_s && acceleration <= amax_deg_s2) {
        break;
      }
      if (accel_plus_cruise_intervals ==
          std::numeric_limits<int>::max()) {
        break;
      }
      ++accel_plus_cruise_intervals;
    }

    if (accel_plus_cruise_intervals < accel_intervals ||
        accel_plus_cruise_intervals >
            std::numeric_limits<int>::max() - accel_intervals) {
      continue;
    }
    const int total_intervals =
        accel_intervals + accel_plus_cruise_intervals;
    if (total_intervals < best_total_intervals ||
        (total_intervals == best_total_intervals &&
         accel_intervals > best_accel_intervals)) {
      best_accel_intervals = accel_intervals;
      best_accel_plus_cruise_intervals =
          accel_plus_cruise_intervals;
      best_total_intervals = total_intervals;
    }
  }

  if (best_accel_intervals == 0) {
    throw std::overflow_error("no finite integer trajectory was found");
  }

  profile.acceleration_intervals = best_accel_intervals;
  profile.accel_plus_cruise_intervals =
      best_accel_plus_cruise_intervals;
  profile.cruise_intervals = best_accel_plus_cruise_intervals -
                             best_accel_intervals;
  profile.total_intervals = best_total_intervals;
  profile.peak_velocity_deg_s =
      profile.distance_deg /
      (dt_s * static_cast<double>(best_accel_plus_cruise_intervals));
  profile.acceleration_deg_s2 =
      profile.distance_deg /
      (dt_s * dt_s * static_cast<double>(best_accel_intervals) *
       static_cast<double>(best_accel_plus_cruise_intervals));

  if (profile.peak_velocity_deg_s > vmax_deg_s ||
      profile.acceleration_deg_s2 > amax_deg_s2) {
    throw std::logic_error("integer trajectory violates its limits");
  }
  return profile;
}

MotorReference mapToMotor(const Sample& joint_sample,
                          double q_raw_zero_rad, double gear_ratio) {
  if (!std::isfinite(joint_sample.q_deg) ||
      !std::isfinite(joint_sample.dq_deg_s) ||
      !std::isfinite(q_raw_zero_rad) || !std::isfinite(gear_ratio) ||
      gear_ratio <= 0.0) {
    throw std::invalid_argument(
        "motor mapping inputs must be finite and gear ratio positive");
  }
  const double joint_q_rad = joint_sample.q_deg * (kPi / 180.0);
  const double joint_dq_rad_s = joint_sample.dq_deg_s * (kPi / 180.0);
  return {q_raw_zero_rad + joint_q_rad * gear_ratio,
          joint_dq_rad_s * gear_ratio};
}

void runTrajectorySelfTests() {
  goldenTenDegreeTest();
  boundaryTests();
  mirrorTests();
  randomizedPropertyTests();
  concatenationTest();
  motorMappingTest();
}

}  // namespace j1_final

#ifdef J1_FINAL_TRAJECTORY_SELF_TEST_MAIN
#include <iostream>

int main() {
  try {
    j1_final::runTrajectorySelfTests();
    std::cout << "J1_FINAL_TRAJECTORY_SELF_TEST=PASS\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
#endif
