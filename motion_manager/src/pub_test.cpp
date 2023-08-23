#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <random>
 
#include "rclcpp/rclcpp.hpp"
#include "pub_test.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

std::random_device rd;
std::mt19937 gen(rd());

int random(int low, int high)
{
    std::uniform_int_distribution<> dist(low, high);
    return dist(gen);
} 

namespace motion_manager {


PubTest::PubTest(const rclcpp::NodeOptions & options)
  : Node("pub_test", options), count_(0) {
  
  publisher_ = create_publisher<nao_robocup_interfaces::msg::MotionID>("motion/req_motion", 10);

  // Use a timer to schedule periodic message publishing.
  timer_ = this->create_wall_timer(
             2s, std::bind(&PubTest::timer_callback, this));
  //motions = {motion_manager::MotionType::WALK, motion_manager::MotionType::KICK, motion_manager::MotionType::GOALIE_SIT};
  motions[0] = motion_manager::MotionType::WALK;
  motions[1] = motion_manager::MotionType::KICK;
  motions[2] = motion_manager::MotionType::GOALIE_SIT;
  motions[3] = motion_manager::MotionType::GETUP_FRONT;
  motions[4] = motion_manager::MotionType::GETUP_BACK;
}


void PubTest::timer_callback() {
  auto message = nao_robocup_interfaces::msg::MotionID();
  message.id = motions[count_];
  RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << std::to_string(message.id) << "'");
  publisher_->publish(message);
  //count_ = (count_ + 1) % (sizeof(motions) / sizeof(motions[0]));
  count_ = random(0, (sizeof(motions) / sizeof(motions[0]))-1 );
}

}  // namespace motion_manager

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(motion_manager::PubTest)