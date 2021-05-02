#include "buffer.h"
#include "imu_message.h"

#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <boost/bind.hpp>
#include <Eigen/Core>

const std::string IMU_TOPIC = "/camera/imu";
const std::string IMAGE0_TOPIC = "/camera/infra1/image_rect_raw";
const std::string IMAGE1_TOPIC = "/camera/infra2/image_rect_raw";
static std::atomic<bool> is_running = true;

std::mutex stdout_mutex;

void image0Callback(const sensor_msgs::ImageConstPtr& message,
                    std::deque<sensor_msgs::ImageConstPtr>& image0_buffer) {
  image0_buffer.push_back(message);
}

void image1Callback(const sensor_msgs::ImageConstPtr& message,
                    std::deque<sensor_msgs::ImageConstPtr>& image1_buffer) {
  image1_buffer.push_back(message);
}

void imuCallback(const sensor_msgs::ImuConstPtr& message,
                 Buffer<sensor_msgs::ImuConstPtr>& buffer) {
  std::cout << "pushed to buffer" << std::endl;
  buffer.push(message);
}

void assignHandler(void (*handler)(int)) {
  signal(SIGINT, handler);
}

void handler(int id) {
  std::cout << "Interrupt signal (" << id << ") received.\n";
  is_running = false;
  ros::shutdown();
}

void imuIntegration(Buffer<sensor_msgs::ImuConstPtr>& buffer) {
  assignHandler(handler);

  std::cout << "imuIntegration" << std::endl;
  while (is_running) {
    const auto message = buffer.pop();
    std::cout << "buffer size = " << buffer.size() << std::endl;
    if (message == nullptr) {
      std::cout << "popped null " << std::endl;
    } else {
      std::cout << "popped " << getTimestamp(message) << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  std::cout << "imuIntegration thread terminates" << std::endl;
}

int main(int argc, char *argv[]) {
  // ros::NodeHandle nh_image0;
  // ros::NodeHandle nh_image1;

  Buffer<sensor_msgs::ImuConstPtr> buffer;

  // ros::Subscriber sub_image0 = nh_image0.subscribe<sensor_msgs::Image>(
  //     IMAGE0_TOPIC, 1000, boost::bind(image0Callback, _1, image0_buffer));
  // ros::Subscriber sub_image1 = nh_image1.subscribe<sensor_msgs::Image>(
  //     IMAGE1_TOPIC, 1000, boost::bind(image1Callback, _1, image1_buffer));

  ros::init(argc, argv, "listener");
  std::cout << "read imu started" << std::endl;
  ros::NodeHandle nh_imu;
  ros::Subscriber sub_imu = nh_imu.subscribe<sensor_msgs::Imu>(
      IMU_TOPIC, 1000, boost::bind(imuCallback, _1, std::ref(buffer)));

  std::thread integration_thread(imuIntegration, std::ref(buffer));
  integration_thread.detach();

  ros::spin();
  return 0;
}
