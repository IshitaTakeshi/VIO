#ifndef INTEGRATION_H
#define INTEGRATION_H

#include <Eigen/Geometry>
#include <iostream>

#include <sophus/so3.hpp>

class AngularVelocityIntegration {
 public:
  AngularVelocityIntegration(const Eigen::Vector3d& w0) :
      quaternion_(Eigen::Quaterniond::Identity()),
      angular_velocity0_(w0) {}

  void update(const Eigen::Vector3d& omega, const double dt);

  Eigen::Quaterniond get() {
    return quaternion_;
  }

 private:
  Eigen::Quaterniond quaternion_;
  Eigen::Vector3d angular_velocity0_;
};

#endif
