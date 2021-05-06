#include "integration.h"

Eigen::Quaterniond approximateSmallRotation(const Eigen::Vector3d& phi) {
  const Eigen::Vector3d d = 0.5 * phi;
  return Eigen::Quaterniond(1.0, d(0), d(1), d(2));
}

Eigen::Quaterniond clacRotationChange(const Eigen::Vector3d& w0,
                                      const Eigen::Vector3d& w1,
                                      const double dt) {
  const Eigen::Vector3d midpoint = 0.5 * (w0 + w1) * dt;
  return approximateSmallRotation(midpoint);
}

Eigen::Quaterniond updateRotation(const Eigen::Quaterniond& q0,
                                  const Eigen::Quaterniond& dq) {
  return (q0 * dq).normalized();
}

Eigen::Vector3d calcVelocityChange(const Eigen::Quaterniond& q0,
                                   const Eigen::Quaterniond& q1,
                                   const Eigen::Vector3d& a0,
                                   const Eigen::Vector3d& a1,
                                   const double dt) {
  return 0.5 * (q0 * a0 + q1 * a1) * dt;
}

Eigen::Vector3d updateVelocity(const Eigen::Vector3d& v0,
                               const Eigen::Vector3d& dv) {
  return v0 + dv;
}

Eigen::Vector3d calcPositionChange(const Eigen::Vector3d& v0,
                                   const Eigen::Vector3d& v1,
                                   const double dt) {
  return 0.5 * (v0 + v1) * dt;
}

Eigen::Vector3d updatePosition(const Eigen::Vector3d& p0,
                               const Eigen::Vector3d& dp) {
  return p0 + dp;
}

void AngularVelocityIntegration::update(
    const Eigen::Vector3d& angular_velocity1, const double dt) {
  const Eigen::Vector3d w0 = angular_velocity0_;
  const Eigen::Vector3d w1 = angular_velocity1;

  const Eigen::Quaterniond dq = clacRotationChange(w0, w1, dt);
  quaternion_ = updateRotation(quaternion_, dq);

  angular_velocity0_ = w1;
}

void EuclideanIntegration::update(
    const Eigen::Vector3d& acceleration1, const double dt) {
  const Eigen::Vector3d a0 = acceleration0_;
  const Eigen::Vector3d a1 = acceleration1;
  const Eigen::Vector3d dv = 0.5 * (a0 + a1) * dt;
  velocity_ = updateVelocity(velocity_, dv);
  acceleration0_ = a1;
}
