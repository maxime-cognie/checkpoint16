#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/detail/float32__struct.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <Eigen/Dense>
#include <cstddef>
#include <vector>

using namespace std::chrono_literals;

class WheelVelocitiesPublisher : public rclcpp::Node {
public:
  WheelVelocitiesPublisher() : Node("wheel_velocities_publisher_node") {
    timer_ = this->create_wall_timer(
        100ms, std::bind(&WheelVelocitiesPublisher::timer_callback, this));

    wheel_vel_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "wheel_speed", 10);

    Jacobian = Eigen::Matrix<float, 4, 3>{
      {-L - W, 1, -1}, {L + W, 1, 1}, {L + W, 1, -1}, {-L - W, 1, 1}
    };
    Jacobian /= R;
  }

private:
  void timer_callback() {
    // Perform the movement sequence only once
    timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "Starts movement sequence!!!");
    RCLCPP_INFO(this->get_logger(), "Move forward!");
    twist_2_wheels(0.0, 0.5, 0.0);
    rclcpp::sleep_for(3s);

    RCLCPP_INFO(this->get_logger(), "Move backward!");
    twist_2_wheels(0.0, -0.5, 0.0);
    rclcpp::sleep_for(3s);

    RCLCPP_INFO(this->get_logger(), "Move sideways to the left!");
    twist_2_wheels(0.0, 0.0, 0.5);
    rclcpp::sleep_for(3s);

    RCLCPP_INFO(this->get_logger(), "Move sideways to the right!");
    twist_2_wheels(0.0, 0.0, -0.5);
    rclcpp::sleep_for(3s);

    RCLCPP_INFO(this->get_logger(), "turn clockwise!");
    twist_2_wheels(1.5708, 0.0, 0.0);
    rclcpp::sleep_for(3s);

    RCLCPP_INFO(this->get_logger(), "turn counter-clockwise!");
    twist_2_wheels(-1.5708, 0.0, 0.0);
    rclcpp::sleep_for(3s);
  }

  void twist_2_wheels(const float wz, const float vx, const float vy) {

    // desired twist command
    Eigen::Vector3f V(wz, vx, vy);
    // wheel command vector
    Eigen::Vector4f u = Jacobian * V;

    std_msgs::msg::Float32MultiArray wheel_speed;
    for (auto ui : u) {
      wheel_speed.data.push_back(ui);
    }

    wheel_vel_pub_->publish(wheel_speed);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_vel_pub_;

  // robot properties
  static constexpr double L = 0.170 / 2;
  static constexpr double W = 0.26969 / 2;
  static constexpr double R = 0.100 / 2;

  /* initialize the Jacobian matrix 4x3(4 output: wheel speed command /
   3 input: z angular velocity, x linear velocity, y linear velocity)*/
  Eigen::Matrix<float, 4, 3> Jacobian;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto wheel_vel_pub = std::make_shared<WheelVelocitiesPublisher>();

  while (rclcpp::ok()) {
    rclcpp::spin_some(wheel_vel_pub);
  }

  rclcpp::shutdown();
  return 0;
}