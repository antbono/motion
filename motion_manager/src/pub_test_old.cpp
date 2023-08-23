#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nao_robocup_interfaces/msg/motion_id.hpp"
#include "nao_robocup_interfaces/msg/exe_transition.hpp" 
#include "motion_manager_node.hpp"                                      

using namespace std::chrono_literals;

class MotionManagerPublisher : public rclcpp::Node
{
public:
  MotionManagerPublisher()
  : Node("motion_manager_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<nao_robocup_interfaces::msg::MotionID>("motion/req_motion", 10);  
    timer_ = this->create_wall_timer(
      3s, std::bind(&MotionManagerPublisher::timer_callback, this));
    //motions = {motion_manager::MotionType::WALK, motion_manager::MotionType::KICK, motion_manager::MotionType::GOALIE_SIT};
    motions[0] = motion_manager::MotionType::WALK;
    motions[1] = motion_manager::MotionType::KICK;
    motions[2] = motion_manager::MotionType::GOALIE_SIT;
  }

private:
  void timer_callback()
  {
    auto message = nao_robocup_interfaces::msg::MotionID();  
    message.id = motions[count_];                                                    
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.id << "'");    
    publisher_->publish(message);
    count_ = (count_+1)%(sizeof(motions)/sizeof(motions[0]));
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nao_robocup_interfaces::msg::MotionID>::SharedPtr publisher_;            
  size_t count_;
  motion_manager::MotionType motions [3];
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionManagerPublisher>());
  rclcpp::shutdown();
  return 0;
}