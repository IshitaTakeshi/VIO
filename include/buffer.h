#include <deque>
#include <iostream>
#include <mutex>

template <class T>
class Buffer {
 public:
  Buffer() {};

  void push(const T& message) {
    std::lock_guard<std::mutex> imu_lock(imu_mutex_);
    imu_buffer_.push_back(message);
  }

  T pop() {
    std::lock_guard<std::mutex> imu_lock(imu_mutex_);
    if (imu_buffer_.size() == 0) {
      return nullptr;
    }
    const auto message = imu_buffer_.front();
    imu_buffer_.pop_front();
    return message;
  }

  int size() {
    return imu_buffer_.size();
  }

 private:
  std::mutex imu_mutex_;
  std::deque<T> imu_buffer_;
};
