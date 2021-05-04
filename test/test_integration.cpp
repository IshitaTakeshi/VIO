#include "integration.h"

#include <sophus/so3.hpp>

#include <gtest/gtest.h>

TEST(Integration, Update) {
  srand(3939);

  std::cout.precision(10);
  const int n = 20;

  auto generateOmega = [&]() -> Eigen::Vector3d {
    const Eigen::Vector3d noise = 0.6 * Eigen::Vector3d::Random();
    const Eigen::Vector3d direction = Eigen::Vector3d(0.5, 0.2, -0.1);
    return (direction + noise).eval();
  };

  const double dt = 0.1;

  const Eigen::Vector3d omega = generateOmega();
  AngularVelocityIntegration integration(omega);
  Sophus::SO3d r;
  for (int i = 1; i < n; i++) {
    const Sophus::SO3d dr = Sophus::SO3d::exp(omega * dt);

    integration.update(omega, dt);
    r = r * dr;
    const Eigen::Vector3d omega = generateOmega();
  }

  const auto pred = integration.get();

  ASSERT_LE((pred.vec() - r.unit_quaternion().vec()).norm(), 1e-3);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
