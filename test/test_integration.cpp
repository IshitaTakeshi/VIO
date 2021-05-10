#include "integration.h"

#include <cmath>

#include <sophus/so3.hpp>

#include <gtest/gtest.h>


Eigen::Vector3d acceleration(const double t) {
  return Eigen::Vector3d(
      t,
      1 / (1. + t),
      1.0
  );
}

Eigen::Vector3d velocity(const double t) {
  return Eigen::Vector3d(
      (1. / 2.) * t * t,
      std::log(1 + t),
      t
  );
}

Eigen::Vector3d position(const double t) {
  return Eigen::Vector3d(
      (1. / 6.) * t * t * t,
      (1 + t) * std::log(1 + t) - t,
      (1. / 2.) * t * t
  );
}

Eigen::Vector3d omega(const double t) {
  // return Eigen::Vector3d(0.1, 0.1, -0.1);
  return Eigen::Vector3d(
      0.5 * M_PI * t * t,
      0.25 * M_PI * t,
      std::sin(0.25 * M_PI * t)
  );
}

TEST(RotationIntegration, Update) {
  srand(3939);

  const int n = 200;

  const double dt = 0.01;

  // This is still not the exact rotation.
  // To obtain the exact rotation, we need to analyticially integrate omega(t)
  // .i.e matrix exponential of skew(omega(t))
  const int m = 20;
  Sophus::SO3d r;
  for (int i = 0; i < m * n; i++) {
    const double ddt = dt / m;
    const double t = double(i) * ddt;
    const Sophus::SO3d dr = Sophus::SO3d::exp(omega(t) * ddt);
    r = r * dr;
  }

  // NOTE omega(n * dt) has to be incorporated
  AngularVelocityIntegration integration(omega(0.0));
  for (int i = 1; i < n + 1; i++) {
    const double t = double(i) * dt;
    integration.update(omega(t), dt);
  }

  const auto pred = integration.get();

  ASSERT_LE((pred.vec() - r.unit_quaternion().vec()).norm(), 5e-4);
}

TEST(EuclideanIntegration, Update) {
  const int n = 10;

  const double dt = 0.1;

  std::vector<Eigen::Vector3d> accelerations(n + 1);
  for (int i = 0; i < n + 1; i++) {
    accelerations[i] = acceleration(double(i) * dt);
  }

  EuclideanIntegration integration(accelerations[0]);

  for (int i = 1; i < n + 1; i++) {
    integration.update(accelerations[i], dt);
  }

  ASSERT_LE((integration.get() - velocity(n * dt)).norm(), 1e-3);
}

TEST(PoseIntegration, UpdateWithoutRotation) {
  const int n = 3;

  const double dt = 0.1;

  // NOTE n is inclusive
  PoseIntegration pose(Eigen::Vector3d::Zero(), acceleration(0.0 * dt));
  for (int i = 1; i < n + 1; i++) {
    pose.update(Eigen::Vector3d::Zero(), acceleration(double(i) * dt), dt);
  }

  ASSERT_LE((pose.velocity() - velocity(n * dt)).norm(), 1e-3);
  ASSERT_LE((pose.position() - position(n * dt)).norm(), 1e-3);
}

TEST(PoseIntegration, UpdateWithRotation) {
  const int n = 100;

  std::vector<Eigen::Vector3d> angular_velocities(n);
  std::vector<Eigen::Vector3d> accelerations(n);
  std::vector<Eigen::Quaterniond> expected_rotations(n);
  std::vector<Eigen::Vector3d> expected_velocities(n);
  std::vector<Eigen::Vector3d> expected_positions(n);

  const double dt = 0.01;

  {
    const int m = 10;
    const double ddt = dt / m;

    Sophus::SO3d r;
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    for (int i = 0; i < n; i++) {
      const double t = double(i * m) * ddt;  // == i * dt

      angular_velocities[i] = omega(t);
      accelerations[i] = acceleration(t);

      expected_rotations[i] = r.unit_quaternion();
      expected_velocities[i] = v;
      expected_positions[i] = p;

      for (int j = 0; j < m; j++) {
        const double t = double(i * m + j) * ddt;
        const Eigen::Vector3d a = acceleration(t);

        v = v + (r * a) * ddt;
        p = p + v * ddt + 0.5 * (r * a) * ddt * ddt;

        const Sophus::SO3d dr = Sophus::SO3d::exp(omega(t) * ddt);
        r = r * dr;
      }
    }
  }

  PoseIntegration pose(angular_velocities[0], accelerations[0]);
  for (int i = 1; i < n; i++) {
    pose.update(angular_velocities[i], accelerations[i], dt);

    EXPECT_LE((expected_rotations[i].vec() - pose.rotation().vec()).norm(), 1e-2);
    EXPECT_LE((expected_velocities[i] - pose.velocity()).norm(), 1e-2);;
    EXPECT_LE((expected_positions[i] - pose.position()).norm(), 1e-2);;
  }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
