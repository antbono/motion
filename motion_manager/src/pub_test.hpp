#ifndef PUB_TEST_HPP_
#define PUB_TEST_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nao_robocup_interfaces/msg/motion_id.hpp"
#include "nao_robocup_interfaces/msg/exe_transition.hpp"
#include "motion_manager_node.hpp"

namespace motion_manager
{

class PubTest : public rclcpp::Node
{
public:
  //COMPOSITION_PUBLIC
  explicit PubTest(const rclcpp::NodeOptions & options);

protected:
  void timer_callback();
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nao_robocup_interfaces::msg::MotionID>::SharedPtr publisher_;            
  size_t count_;
  motion_manager::MotionType motions [5];
};

}  // namespace composition

#endif  // PUB_TEST_HPP_