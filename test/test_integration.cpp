#include "integration.h"

#include <cmath>

#include <sophus/so3.hpp>

#include <gtest/gtest.h>

TEST(RotationIntegration, Update) {
  srand(3939);

  const int n = 10;

  auto generateOmega = [&]() -> Eigen::Vector3d {
    const Eigen::Vector3d noise = 0.1 * Eigen::Vector3d::Random();
    const Eigen::Vector3d direction = Eigen::Vector3d(0.5, 0.2, -0.3);
    return (direction + noise).eval();
  };

  const double dt = 0.1;

  std::vector<Eigen::Vector3d> omegas(n);
  for (int i = 0; i < n; i++) {
    omegas[i] = generateOmega();
  }

  Sophus::SO3d r;
  for (int i = 1; i < n; i++) {
    const Sophus::SO3d dr = Sophus::SO3d::exp(omegas[i] * dt);
    r = r * dr;
  }

  AngularVelocityIntegration integration(omegas[0]);
  for (int i = 1; i < n; i++) {
    integration.update(omegas[i], dt);
  }

  const auto pred = integration.get();

  ASSERT_LE((pred.vec() - r.unit_quaternion().vec()).norm(), 4e-3);
}

TEST(EuclideanIntegration, Update) {
  const int n = 10;

  auto acceleration = [](const double t) -> Eigen::Vector3d {
    return Eigen::Vector3d(
        t,
        1 / (1. + t),
        1.0
    );
  };

  auto velocity = [](const double t) -> Eigen::Vector3d {
    return Eigen::Vector3d(
        (1. / 2.) * t * t,
        std::log(1 + t),
        t
    );
  };

  const double dt = 0.1;

  EuclideanIntegration integration(acceleration(0.0 * dt));

  for (int i = 1; i <= n; i++) {
    integration.update(acceleration(double(i) * dt), dt);
  }

  ASSERT_LE((integration.get() - velocity(n * dt)).norm(), 1e-3);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
