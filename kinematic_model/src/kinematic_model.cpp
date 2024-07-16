#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <Eigen/Dense>

using namespace std::chrono_literals;

class KinematicModel : public rclcpp::Node {
public:
  KinematicModel() : Node("kinematic_model_node") {
    using std::placeholders::_1;

    wheel_vel_sub_ =
      this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "wheel_speed", 10,
        std::bind(&KinematicModel::wheel_vel_callback, this, _1));   

    cmd_vel_pub_ = 
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    Jacobian = Eigen::Matrix<float, 4, 3>{
      {-L - W, 1, -1}, {L + W, 1, 1}, {L + W, 1, -1}, {-L - W, 1, 1}
    };
    Jacobian /= R;
  }

private:
  void wheel_vel_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    Eigen::Vector4f U;

    for(size_t i = 0; i < msg->data.size(); ++i) {
      U(i) = msg->data[i];
    }

    Eigen::JacobiSVD<Eigen::Matrix<float, 4, 3>> svd(Jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<float, 3, 4> Jacobian_inv = svd.solve(Eigen::MatrixXf::Identity(Jacobian.rows(), Jacobian.rows()));

    Eigen::Vector3f Twist = Jacobian_inv * U;

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.angular.z = Twist[0];
    cmd_vel.linear.x = Twist[1];
    cmd_vel.linear.y = Twist[2];

    cmd_vel_pub_->publish(cmd_vel);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

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
  auto kinematic_model_node = std::make_shared<KinematicModel>();

  while(rclcpp::ok()) {
    rclcpp::spin_some(kinematic_model_node);
  }
  
  rclcpp::shutdown();
  return 0;
}