#include <Eigen/Core>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

inline Eigen::Vector3d getAcceleration(
    const sensor_msgs::ImuConstPtr& message) {
  return Eigen::Vector3d(
      message->linear_acceleration.x,
      message->linear_acceleration.y,
      message->linear_acceleration.z);
}

inline Eigen::Vector3d getAngularVelocity(
    const sensor_msgs::ImuConstPtr& message) {
  return Eigen::Vector3d(
      message->angular_velocity.x,
      message->angular_velocity.y,
      message->angular_velocity.z);
}

inline double getTimestamp(const sensor_msgs::ImuConstPtr& message) {
  return message->header.stamp.toSec();
}
