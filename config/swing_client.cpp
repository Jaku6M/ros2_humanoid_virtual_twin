#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "swing_action/action/leg_swing.hpp"// here include your action contents file located in: YOUR_WORKSPACE_NAME/build/ACTION_FOLDER_NAME/rosidl_generator_cpp/ACTION_FOLDER_NAME/action
//example directory: Humanoid_workspace/build/swing_action/rosidl_generator_cpp/swing_action/action
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace ros2_humanoid_virtual_twin//here write your package name
{
class SwingActionClient : public rclcpp::Node//Here SwingActionClient Class is created, its inheriting after class Node
{
public:
  //Here I create shortcuts for long names so when i type Swing i mean swing_action::action::LegSwing
  using Swing = swing_action::action::LegSwing;
                //ACTION_DIRECTORY::action::ACTION_FILE_NAME
  using GoalHandleSwing= rclcpp_action::ClientGoalHandle<Swing>;
  //Here Node SwingAcitionClient is initialized with some options
  explicit SwingActionClient(const rclcpp::NodeOptions & options)
  : Node("swing_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Swing>(this,"swing");//Initialization of actions client

    this->timer_ = this->create_wall_timer(
      //timer callin send_goal function every 500ms
      std::chrono::milliseconds(500),
      std::bind(&SwingActionClient::send_goal, this));//We call send_goal method from SwingActionClient class without creating object of this class because we use std::bind
  }

  void send_goal()// send goal to the server
  {
    using namespace std::placeholders;

    this->timer_->cancel(); // stop the timer to avoid sending the goal multiple times

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Swing::Goal();//Creating goal message
    goal_msg.goal = 3.0;//How much seconds the leg should swing
            //nuimerical value is passed to our action file
    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Swing>::SendGoalOptions();//Creating options for sending goal
    send_goal_options.goal_response_callback = std::bind(&SwingActionClient::goal_response_callback, this, _1);//Function handling the response of the goal
    send_goal_options.feedback_callback = std::bind(&SwingActionClient::feedback_callback, this, _1, _2);//Function handling the feedback of the goal
    send_goal_options.result_callback = std::bind(&SwingActionClient::result_callback, this, _1);// Function handling the result of the goal
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);//  Sending goal to the server
  }

private:
  rclcpp_action::Client<Swing>::SharedPtr client_ptr_;//intellident Pointer to the action client
  rclcpp::TimerBase::SharedPtr timer_;//Intelligent Pointer to the timer

  void goal_response_callback(const GoalHandleSwing::SharedPtr & goal_handle)//Function handling the response of the goal
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(// Function handling the feedback of the goal
    GoalHandleSwing::SharedPtr,
    const std::shared_ptr<const Swing::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Time left: "<< feedback->timeleft;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());//Prints the feedback message=time left to the end of the leg movement
  }

  void result_callback(const GoalHandleSwing::WrappedResult & result)// Function handling the result of the goal
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Procedure ended with status: " << result.result->success;//Prints the result of the goal if it was successful
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class SwingActionClient

}  // namespace ros2_humanoid_virtual_twin
//Line below allows us to run this code as a node. "It acts as a main function"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_humanoid_virtual_twin::SwingActionClient)