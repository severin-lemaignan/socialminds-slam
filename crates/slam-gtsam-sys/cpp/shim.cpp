#include "slam-gtsam-sys/cpp/shim.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "slam-gtsam-sys/src/lib.rs.h"

namespace slam_gtsam {

namespace {

gtsam::Pose3 to_pose3(const FfiPose& p) {
  // FfiPose carries (x, y, z, w); gtsam::Rot3's quaternion constructor wants w first.
  return gtsam::Pose3(gtsam::Rot3::Quaternion(p.q[3], p.q[0], p.q[1], p.q[2]),
                      gtsam::Point3(p.t[0], p.t[1], p.t[2]));
}

FfiPose from_pose3(const gtsam::Pose3& p) {
  const auto q = p.rotation().toQuaternion();
  FfiPose out;
  out.t = {p.x(), p.y(), p.z()};
  out.q = {q.x(), q.y(), q.z(), q.w()};
  return out;
}

gtsam::Vector3 to_vec3(const std::array<double, 3>& v) {
  return gtsam::Vector3(v[0], v[1], v[2]);
}

gtsam::SharedNoiseModel diagonal_sigmas(const std::array<double, 6>& sigmas) {
  gtsam::Vector6 s;
  s << sigmas[0], sigmas[1], sigmas[2], sigmas[3], sigmas[4], sigmas[5];
  return gtsam::noiseModel::Diagonal::Sigmas(s);
}

}  // namespace

// ---- Preintegrator ---------------------------------------------------------------------

Preintegrator::Preintegrator(double accel_sigma, double gyro_sigma, double integration_sigma,
                             double gravity, const std::array<double, 3>& accel_bias,
                             const std::array<double, 3>& gyro_bias)
    : pim_([&] {
        // MakeSharedU: Z-up world with gravity (0, 0, -g) — the engine convention.
        auto params = gtsam::PreintegrationParams::MakeSharedU(gravity);
        params->accelerometerCovariance =
            gtsam::I_3x3 * accel_sigma * accel_sigma;
        params->gyroscopeCovariance = gtsam::I_3x3 * gyro_sigma * gyro_sigma;
        params->integrationCovariance =
            gtsam::I_3x3 * integration_sigma * integration_sigma;
        return gtsam::PreintegratedImuMeasurements(
            params, gtsam::imuBias::ConstantBias(to_vec3(accel_bias), to_vec3(gyro_bias)));
      }()) {}

void Preintegrator::integrate(const std::array<double, 3>& accel,
                              const std::array<double, 3>& gyro, double dt) {
  pim_.integrateMeasurement(to_vec3(accel), to_vec3(gyro), dt);
}

void Preintegrator::reset() { pim_.resetIntegration(); }

double Preintegrator::delta_t() const { return pim_.deltaTij(); }

FfiNavState Preintegrator::predict(const FfiNavState& state) const {
  const gtsam::NavState predicted =
      pim_.predict(gtsam::NavState(to_pose3(state.pose), to_vec3(state.velocity)),
                   pim_.biasHat());
  FfiNavState out;
  out.pose = from_pose3(predicted.pose());
  const auto v = predicted.velocity();
  out.velocity = {v.x(), v.y(), v.z()};
  return out;
}

// ---- GraphBuilder ----------------------------------------------------------------------

void GraphBuilder::insert_pose(std::uint64_t key, const FfiPose& pose) {
  values_.insert(key, to_pose3(pose));
}

void GraphBuilder::insert_velocity(std::uint64_t key, const std::array<double, 3>& velocity) {
  values_.insert(key, to_vec3(velocity));
}

void GraphBuilder::insert_bias(std::uint64_t key, const std::array<double, 3>& accel_bias,
                               const std::array<double, 3>& gyro_bias) {
  values_.insert(key, gtsam::imuBias::ConstantBias(to_vec3(accel_bias), to_vec3(gyro_bias)));
}

void GraphBuilder::add_prior_pose(std::uint64_t key, const FfiPose& pose,
                                  const std::array<double, 6>& sigmas) {
  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(key, to_pose3(pose), diagonal_sigmas(sigmas)));
}

void GraphBuilder::add_between_pose(std::uint64_t key_from, std::uint64_t key_to,
                                    const FfiPose& relative,
                                    const std::array<double, 6>& sigmas) {
  graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(key_from, key_to, to_pose3(relative),
                                                diagonal_sigmas(sigmas)));
}

void GraphBuilder::add_prior_velocity(std::uint64_t key, const std::array<double, 3>& velocity,
                                      const std::array<double, 3>& sigmas) {
  graph_.add(gtsam::PriorFactor<gtsam::Vector3>(
      key, to_vec3(velocity), gtsam::noiseModel::Diagonal::Sigmas(to_vec3(sigmas))));
}

void GraphBuilder::add_prior_bias(std::uint64_t key, const std::array<double, 3>& accel_bias,
                                  const std::array<double, 3>& gyro_bias,
                                  const std::array<double, 6>& sigmas) {
  graph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
      key, gtsam::imuBias::ConstantBias(to_vec3(accel_bias), to_vec3(gyro_bias)),
      diagonal_sigmas(sigmas)));
}

void GraphBuilder::add_imu_factor(std::uint64_t pose_i, std::uint64_t velocity_i,
                                  std::uint64_t pose_j, std::uint64_t velocity_j,
                                  std::uint64_t bias, const Preintegrator& preintegrated) {
  graph_.add(
      gtsam::ImuFactor(pose_i, velocity_i, pose_j, velocity_j, bias, preintegrated.pim()));
}

FfiOptimizeStats GraphBuilder::optimize(std::uint32_t max_iterations) {
  gtsam::LevenbergMarquardtParams params;
  params.setMaxIterations(static_cast<int>(max_iterations));
  FfiOptimizeStats stats;
  stats.initial_error = graph_.error(values_);
  gtsam::LevenbergMarquardtOptimizer optimizer(graph_, values_, params);
  values_ = optimizer.optimize();
  stats.final_error = graph_.error(values_);
  stats.iterations = static_cast<std::uint64_t>(optimizer.iterations());
  return stats;
}

FfiPose GraphBuilder::pose_at(std::uint64_t key) const {
  return from_pose3(values_.at<gtsam::Pose3>(key));
}

FfiVec3 GraphBuilder::velocity_at(std::uint64_t key) const {
  const auto v = values_.at<gtsam::Vector3>(key);
  FfiVec3 out;
  out.v = {v.x(), v.y(), v.z()};
  return out;
}

// ---- Factories -------------------------------------------------------------------------

std::unique_ptr<GraphBuilder> new_graph_builder() { return std::make_unique<GraphBuilder>(); }

std::unique_ptr<Preintegrator> new_preintegrator(double accel_sigma, double gyro_sigma,
                                                 double integration_sigma, double gravity,
                                                 const std::array<double, 3>& accel_bias,
                                                 const std::array<double, 3>& gyro_bias) {
  return std::make_unique<Preintegrator>(accel_sigma, gyro_sigma, integration_sigma, gravity,
                                         accel_bias, gyro_bias);
}

}  // namespace slam_gtsam
