#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "swing_action/action/leg_swing.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace ros2_humanoid_virtual_twin
{
class SwingActionClient : public rclcpp::Node
{
public:
  using Swing = swing_action::action::LegSwing;
  using GoalHandleSwing= rclcpp_action::ClientGoalHandle<Swing>;
  //powyższe linijki powodują że możesz korzystać ze skróconych zapisów długich nazw
  explicit SwingActionClient(const rclcpp::NodeOptions & options)
  : Node("swing_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Swing>(this,"swing");

    this->timer_ = this->create_wall_timer(
      //timer wywolujacy metode send_goal co 500ms
      std::chrono::milliseconds(500),
      std::bind(&SwingActionClient::send_goal, this));
  }

  void send_goal()// Wysyłanie celu do serwera akcji
  {
    using namespace std::placeholders;

    this->timer_->cancel(); // zatrzymanie timera aby uniknac wielokrotnego wyslania celu

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Swing::Goal();
    goal_msg.goal = 3.0;//ile sekund Swing chcesz?
            //A to wrzucamy do akcji ktora napisalismy
    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Swing>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&SwingActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&SwingActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&SwingActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Swing>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleSwing::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleSwing::SharedPtr,
    const std::shared_ptr<const Swing::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Time left: "<< feedback->timeleft;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleSwing::WrappedResult & result)
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
    ss << "Procedure ended with status: " << result.result->success;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class SwingActionClient

}  // namespace action_tutorials_cpp
//To poniżej sprawia że możemy uruchomić ten kod jako węzeł. Pelni to role tak jakby maina
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_humanoid_virtual_twin::SwingActionClient)