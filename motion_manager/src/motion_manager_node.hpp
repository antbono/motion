#ifndef MOTION_MANAGER_NODE_HPP_
#define MOTION_MANAGER_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nao_robocup_interfaces/msg/motion_id.hpp"
#include "nao_robocup_interfaces/msg/exe_transition.hpp"

namespace motion_manager {

enum MotionType : uint8_t {
  NONE = 0,
  STAND,
  WALK,
  TURN_DRIBBLE,
  GETUP_FRONT,
  GETUP_BACK,
  TIP_OVER,
  KICK,
  INITIAL,
  LIMP,
  REF_PICKUP,
  GOALIE_SIT,
  GOALIE_DIVE_RIGHT,
  GOALIE_DIVE_LEFT,
  GOALIE_CENTRE,
  GOALIE_UNCENTRE,
  GOALIE_INITIAL,
  GOALIE_AFTERSIT_INITIAL,
  DEFENDER_CENTRE,
  GOALIE_FAST_SIT,
  MOTION_CALIBRATE,
  STAND_STRAIGHT,
  LINE_UP,
  TEST_ARMS,
  RAISE_ARM,
  UKEMI_FRONT,
  UKEMI_BACK,
  GOALIE_STAND,
  SIT,
  SIGNAL_KICK_IN_RIGHT,
  SIGNAL_KICK_IN_LEFT,
  SIGNAL_GOAL_KICK_RIGHT,
  SIGNAL_GOAL_KICK_LEFT,
  SIGNAL_CORNER_KICK_RIGHT,
  SIGNAL_CORNER_KICK_LEFT,
  SIGNAL_GOAL_RIGHT,
  SIGNAL_GOAL_LEFT,
  SIGNAL_PUSHING_FREE_KICK_RIGHT,
  SIGNAL_PUSHING_FREE_KICK_LEFT,
  SIGNAL_FULL_TIME,
  NUM_ACTION_TYPES, //40
};


const uint8_t priorities[40] = {
  0, // NONE
  0, // STAND
  0, // WALK
  0, // TURN_DRIBBLE
  3, // GETUP_FRONT
  3, // GETUP_BACK
  3, // TIP_OVER
  0, // KICK
  2, // INITIAL
  1, // LIMP
  1, // REF_PICKUP
  2, // GOALIE_SIT
  3, // GOALIE_DIVE_LEFT
  3, // GOALIE_DIVE_RIGHT
  3, // GOALIE_CENTRE
  2, // GOALIE_UNCENTRE
  0, // GOALIE_INITIAL
  0, // GOALIE_AFTERSIT_INITIAL
  2, // DEFENDER_CENTRE
  2, // GOALIE FAST SIT
  0, // MOTION_CALIBRATE
  0, // STAND_STRAIGHT
  0, // LINE_UP
  0, // TEST_ARMS
  1, // RAISE_ARM
  3, // UKEMI_FRONT
  3, // UKEMI_BACK
  1, // GOALIE_STAND
  1, // SIT
  1, // SIGNAL_KICK_IN_RIGHT
  1, // SIGNAL_KICK_IN_LEFT
  1, // SIGNAL_GOAL_KICK_RIGHT
  1, // SIGNAL_GOAL_KICK_LEFT
  1, // SIGNAL_CORNER_KICK_RIGHT
  1, // SIGNAL_CORNER_KICK_LEFT
  1, // SIGNAL_GOAL_RIGHT
  1, // SIGNAL_GOAL_LEFT
  1, // SIGNAL_PUSHING_FREE_KICK_RIGHT
  1, // SIGNAL_PUSHING_FREE_KICK_LEFT
  1, // SIGNAL_FULL_TIME
};


class MotionManagerNode : public rclcpp::Node {
 public:

  explicit MotionManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  virtual ~MotionManagerNode();

 private:
  
  // Subscriptions
  rclcpp::Subscription<nao_robocup_interfaces::msg::MotionID>::SharedPtr sub_req_motion_;

  rclcpp::Subscription<nao_robocup_interfaces::msg::ExeTransition>::SharedPtr sub_getup_front_feedback_;
  rclcpp::Subscription<nao_robocup_interfaces::msg::ExeTransition>::SharedPtr sub_getup_back_feedback_;
  rclcpp::Subscription<nao_robocup_interfaces::msg::ExeTransition>::SharedPtr sub_goalie_sit_feedback_;


  // Publishers
  rclcpp::Publisher<nao_robocup_interfaces::msg::ExeTransition>::SharedPtr pub_walk_cmd_;
  rclcpp::Publisher<nao_robocup_interfaces::msg::ExeTransition>::SharedPtr pub_kick_cmd_;
  rclcpp::Publisher<nao_robocup_interfaces::msg::ExeTransition>::SharedPtr pub_getup_front_cmd_;
  rclcpp::Publisher<nao_robocup_interfaces::msg::ExeTransition>::SharedPtr pub_getup_back_cmd_;
  rclcpp::Publisher<nao_robocup_interfaces::msg::ExeTransition>::SharedPtr pub_goalie_sit_cmd_;

  void reqCallback(const nao_robocup_interfaces::msg::MotionID & requested_motion);
  void staticActionCallback(const nao_robocup_interfaces::msg::ExeTransition & sub_state);
  //void staticActionCallback();
  void stopCurrMotion();
  void startMotion(const nao_robocup_interfaces::msg::MotionID & requested_motion);

  // State variables
  nao_robocup_interfaces::msg::MotionID curr_motion_;
  nao_robocup_interfaces::msg::MotionID DEFAULT_MOTION_;
  
};





}  // namespace motion_manager

#endif  // MOTION_MANAGER_NODE_HPP_
