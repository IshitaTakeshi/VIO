#ifndef INTEGRATION_H
#define INTEGRATION_H

#include <Eigen/Geometry>
#include <iostream>

#include <sophus/so3.hpp>

class AngularVelocityIntegration {
 public:
  AngularVelocityIntegration() {};

  AngularVelocityIntegration(const Eigen::Vector3d& w0) :
      quaternion_(Eigen::Quaterniond::Identity()),
      angular_velocity0_(w0) {}

  void update(const Eigen::Vector3d& angular_velocity1, const double dt);

  Eigen::Quaterniond get() {
    return quaternion_;
  }

 private:
  Eigen::Quaterniond quaternion_;
  Eigen::Vector3d angular_velocity0_;
};

class EuclideanIntegration {
 public:
  EuclideanIntegration() {};

  EuclideanIntegration(const Eigen::Vector3d& a0) :
      velocity_(Eigen::Vector3d::Zero()),
      acceleration0_(a0) {}

  void update(const Eigen::Vector3d& acceleration1, const double dt);

  Eigen::Vector3d get() {
    return velocity_;
  }

 private:
  Eigen::Vector3d velocity_;
  Eigen::Vector3d acceleration0_;
};

class PoseIntegration {
 public:
  PoseIntegration(const Eigen::Vector3d& angular_velocity,
                  const Eigen::Vector3d& acceleration) :
      rotation_(AngularVelocityIntegration(angular_velocity)),
      velocity_(EuclideanIntegration(acceleration)) {}

  void update(const Eigen::Vector3d& angular_velocity,
              const Eigen::Vector3d& local_acceleration,
              const double dt);

  Eigen::Quaterniond rotation() {
    return rotation_.get();
  }

  Eigen::Vector3d velocity() {
    return velocity_.get();;
  }

  Eigen::Vector3d position() {
    return position_.get();;
  }

 private:
  AngularVelocityIntegration rotation_;
  EuclideanIntegration velocity_;
  EuclideanIntegration position_;
};

#endif
