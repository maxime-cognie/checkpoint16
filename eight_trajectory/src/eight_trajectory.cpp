#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2/impl/utils.h"

#include <Eigen/Dense>
#include <cmath>
#include <cstddef>
#include <functional>
#include <vector>


using namespace std::chrono_literals;

class EightTrajectory : public rclcpp::Node {
public:
  EightTrajectory() : Node("eight_trajectory_node") {
    using std::placeholders::_1;

    timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&EightTrajectory::timer_callback, this),
        timer_callback_group_);

    wheel_vel_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "wheel_speed", 10);

    rclcpp::SubscriptionOptions odom_sub_option;
    odom_sub_option.callback_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry/filtered", 10,
      std::bind(&EightTrajectory::odom_callback, this, _1),
      odom_sub_option);
    Jacobian /= R;
  }

private:
  void timer_callback() {
    // Perform the movement sequence only once
    timer_->cancel();

    RCLCPP_INFO(this->get_logger(), "Starts movement sequence!!!");
    for(auto wp : waypoints) {
      RCLCPP_INFO(this->get_logger(), "Moving to the next waypoint.");
      target_pose_ = current_pose_ + wp;
      error_pose_ = target_pose_ - current_pose_;

      /* compute wheel velocities until the norm2(distance) of the error between 
        the current pose of the robot and the the targeted pose is lesser than ε */
      while(error_pose_.norm() >= 0.02 && rclcpp::ok()) {
        velocity_2_twist();
        rclcpp::sleep_for(100ms);
      }
    }
  }

  // convert twist command into wheel velocity command
  void twist_2_wheels(const Eigen::Vector3f nu) {
    // wheel command vector
    Eigen::Vector4f u = Jacobian * nu;

    // move wheel velocity command vector to ros message
    std_msgs::msg::Float32MultiArray wheel_speed;
    for (auto ui : u) {
      wheel_speed.data.push_back(ui);
    }

    wheel_vel_pub_->publish(wheel_speed);
  }

  // convert dphi,dx,dy into twist
  void velocity_2_twist() {
    error_pose_ = target_pose_ - current_pose_;

    float phi = current_pose_(0);
    // rotation matrix 
    Eigen::Matrix3f Rx{
      {1.0,           0.0,            0.0},
      {0.0,           std::cos(phi),  std::sin(phi)},
      {0.0,           -std::sin(phi), std::cos(phi)}
    };
    // transpose command into chassis-fixed frame
    Eigen::Vector3f nu = Rx * error_pose_;
  
    twist_2_wheels(nu);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom) {
    tf2::Quaternion q(
      odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y,
      odom->pose.pose.orientation.z,
      odom->pose.pose.orientation.w
    );

    float p = tf2::impl::getYaw(q);
    // 2 * std::atan2(odom->pose.pose.orientation.z,
    // odom->pose.pose.orientation.w);
    current_pose_(0) = p;
    current_pose_(1) = odom->pose.pose.position.x;
    current_pose_(2) = odom->pose.pose.position.y;
  }

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  Eigen::Vector3f current_pose_;
  Eigen::Vector3f target_pose_;
  Eigen::Vector3f error_pose_;

  // robot properties
  static constexpr double L = 0.170 / 2;
  static constexpr double W = 0.26969 / 2;
  static constexpr double R = 0.100 / 2;

  /* initialize the Jacobian matrix 4x3(4 output: wheel velocity command /
   3 input: z angular velocity, x linear velocity, y linear velocity)*/
  Eigen::Matrix<float, 4, 3> Jacobian{
    {-L - W, 1, -1}, {L + W, 1, 1}, {L + W, 1, -1}, {-L - W, 1, 1}
  };

  // list of waypoints
  std::vector<Eigen::Vector3f> waypoints{
      {0.0,       1.0,      -1.0},
      {0.0,       1.0,       1.0},
      {0.0,       1.0,       1.0},
      {1.5708,    1.0,      -1.0},
      {-1.5708,  -0.5,      -0.5},
      {-1.5708,  -0.5,      -0.5},
      {0.0,      -1.0,       1.0},
      {0.0,      -1.0,       1.0},
      {0.0,      -1.0,      -1.0}
  };
//   {
//       {0.0,       1.0,      -1.0},
//       {0.0,       1.0,       1.0},
//       {0.0,       1.0,       1.0},
//       {1.5708,    1.0,      -1.0},
//       {-3.1415,  -1.0,      -1.0},
//       {0.0,      -1.0,       1.0},
//       {0.0,      -1.0,       1.0},
//       {0.0,      -1.0,      -1.0}
//   };
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto wheel_vel_pub = std::make_shared<EightTrajectory>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(wheel_vel_pub);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}