#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <chrono>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

class MDriverNode : public rclcpp::Node
{
public:
  MDriverNode()
  : Node("mdriver_node_test"),
    delta_T_(10ms),
    i_amp_(2.0),
    omega_(2.0 * M_PI),
    currents_(6, 0.0),
    msg_()
  {
    RCLCPP_INFO(this->get_logger(), "Starting up mdriver test node");

    msg_.data.resize(6);

    currents_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/mdriver/des_currents_reg", 256);

    timer_ = this->create_wall_timer(
      delta_T_, std::bind(&MDriverNode::mdriver_timer_cb, this));

    RCLCPP_INFO(this->get_logger(), "Start spinning");
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr currents_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  //duration is defined in multiples of 1/1000 seconds (i.e. milliseconds)
  const std::chrono::duration<long long, std::ratio<1, 1000>> delta_T_;
  const double i_amp_;
  const double omega_;
  std::vector<double> currents_;
  std_msgs::msg::Float32MultiArray msg_;

  void mdriver_timer_cb()
  {
    rclcpp::Time current_time = this->get_clock()->now();
    double current = i_amp_ * sin(omega_ * current_time.seconds());

    for (size_t i = 0; i < msg_.data.size(); ++i) {
      msg_.data[i] = 0.0f;
    }

    msg_.data[0] = static_cast<float>(current);

    currents_pub_->publish(msg_);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MDriverNode>());
  rclcpp::shutdown();
  return 0;
}