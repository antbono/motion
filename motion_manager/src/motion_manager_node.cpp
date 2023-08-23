#include <memory>
#include <utility>
#include <string>

#include "motion_manager_node.hpp"

namespace motion_manager {

  MotionManagerNode::MotionManagerNode(const rclcpp::NodeOptions & options)
    : Node("MotionManagerNode", options) {

    // DEFAULT_MOTION
    DEFAULT_MOTION_ = nao_robocup_interfaces::msg::MotionID();

    DEFAULT_MOTION_.id = nao_robocup_interfaces::msg::MotionID::NONE;

    // publisher
    pub_walk_cmd_ = this->create_publisher<nao_robocup_interfaces::msg::ExeTransition>("motion/walk_cmd", 1);
    pub_kick_cmd_ = this->create_publisher<nao_robocup_interfaces::msg::ExeTransition>("motion/kick_cmd", 1);
    pub_getup_front_cmd_ = this->create_publisher<nao_robocup_interfaces::msg::ExeTransition>("motion/getup_front_cmd", 1);
    pub_getup_back_cmd_ = this->create_publisher<nao_robocup_interfaces::msg::ExeTransition>("motion/getup_back_cmd", 1);
    pub_goalie_sit_cmd_ = this->create_publisher<nao_robocup_interfaces::msg::ExeTransition>("motion/goalie_sit_cmd", 1);

    //subscribers
    sub_req_motion_ = this->create_subscription<nao_robocup_interfaces::msg::MotionID>(
                        "motion/req_motion", 10, std::bind(
                          &MotionManagerNode::reqCallback, this, std::placeholders::_1));
    sub_getup_front_feedback_ = this->create_subscription<nao_robocup_interfaces::msg::ExeTransition>(
                                  "motion/getup_front_feedback", 10, std::bind(
                                    &MotionManagerNode::staticActionCallback, this, std::placeholders::_1));
    sub_getup_back_feedback_ = this->create_subscription<nao_robocup_interfaces::msg::ExeTransition>(
                                 "motion/getup_back_feedback", 10, std::bind(
                                   &MotionManagerNode::staticActionCallback, this, std::placeholders::_1));
    sub_goalie_sit_feedback_ = this->create_subscription<nao_robocup_interfaces::msg::ExeTransition>(
                                 "motion/goalie_sit_feedback", 10, std::bind(
                                   &MotionManagerNode::staticActionCallback, this, std::placeholders::_1));

  RCLCPP_INFO( get_logger(), "motion_manager_node()" );  

  }//MotionManagerNode

  MotionManagerNode::~MotionManagerNode() {}

  void MotionManagerNode::reqCallback(const nao_robocup_interfaces::msg::MotionID & requested_motion) {


    RCLCPP_DEBUG( get_logger(),("reqCallback() with motion "+std::to_string(requested_motion.id)).c_str());

    if ( requested_motion.id != curr_motion_.id &&
         motion_manager::priorities[requested_motion.id] >= motion_manager::priorities[curr_motion_.id]) {

      MotionManagerNode::stopCurrMotion();
      MotionManagerNode::startMotion(requested_motion);
      curr_motion_ = requested_motion;
      //curr_state.transition = MotionManagerNode::msg::ExeTransition.START; //some node is active at the moment
    }

  }//reqCallBack

  void MotionManagerNode::staticActionCallback(const nao_robocup_interfaces::msg::ExeTransition & sub_state) {
    auto a = sub_state.transition;
    a=a+1;
    MotionManagerNode::startMotion(DEFAULT_MOTION_);
    curr_motion_ = DEFAULT_MOTION_;
  }


  void MotionManagerNode::stopCurrMotion() {

    RCLCPP_DEBUG(get_logger(), ("stopCurrMotion() motion: " + std::to_string(curr_motion_.id)).c_str() );

    auto msg = nao_robocup_interfaces::msg::ExeTransition();
    msg.transition = nao_robocup_interfaces::msg::ExeTransition::STOP;

    switch (curr_motion_.id) {

    case nao_robocup_interfaces::msg::MotionID::NONE:
      ;
      break;

    case nao_robocup_interfaces::msg::MotionID::WALK:
      pub_walk_cmd_->publish(msg);
      break;

    case nao_robocup_interfaces::msg::MotionID::KICK:
      pub_kick_cmd_->publish(msg);
      break;

    case nao_robocup_interfaces::msg::MotionID::GETUP_FRONT:
      pub_getup_front_cmd_->publish(msg);
      break;

    case nao_robocup_interfaces::msg::MotionID::GETUP_BACK:
      pub_getup_back_cmd_->publish(msg);
      break;

    case nao_robocup_interfaces::msg::MotionID::GOALIE_SIT:
      pub_goalie_sit_cmd_->publish(msg);
      break;

    default:
      ;
    }
  }// stopCurrMotion

  void MotionManagerNode::startMotion(const nao_robocup_interfaces::msg::MotionID & requested_motion) {

    RCLCPP_DEBUG(get_logger(), ("startMotion() motion: " + std::to_string(requested_motion.id)).c_str() );

    auto msg = nao_robocup_interfaces::msg::ExeTransition();
    msg.transition = nao_robocup_interfaces::msg::ExeTransition::START;

    // start req motion
    switch (requested_motion.id) {

    case nao_robocup_interfaces::msg::MotionID::NONE:
      ;
      break;

    case nao_robocup_interfaces::msg::MotionID::WALK:
      pub_walk_cmd_->publish(msg);
      break;

    case nao_robocup_interfaces::msg::MotionID::KICK:
      pub_kick_cmd_->publish(msg);
      break;

    case nao_robocup_interfaces::msg::MotionID::GETUP_FRONT:
      pub_getup_front_cmd_->publish(msg);
      break;

    case nao_robocup_interfaces::msg::MotionID::GETUP_BACK:
      pub_getup_back_cmd_->publish(msg);
      break;

    case nao_robocup_interfaces::msg::MotionID::GOALIE_SIT:
      pub_goalie_sit_cmd_->publish(msg);
      break;

    default:
      ;
    }

  }// start motion

}  // namespace motion_manager

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(motion_manager::MotionManagerNode)
