#include <functional>
#include <memory>
#include <thread>

#include "swing_action/action/leg_swing.hpp"// here include your action contents file located in: YOUR_WORKSPACE_NAME/build/ACTION_FOLDER_NAME/rosidl_generator_cpp/ACTION_FOLDER_NAME/action
//example directory: Humanoid_workspace/build/swing_action/rosidl_generator_cpp/swing_action/action
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"// here is additional inlcude for publishing joint states therefore moving the leg
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace ros2_humanoid_virtual_twin//here write your package name
{
class SwingActionServer : public rclcpp::Node//Here SwingActionseerver Class is created, its inheriting after class Node
{
public:
  //Here I create shortcuts for long names so when i type Swing i mean swing_action::action::LegSwing
  using Swing = swing_action::action::LegSwing; 
              //ACTION_DIRECTORY::action::ACTION_FILE_NAME
  using GoalHandleSwing = rclcpp_action::ServerGoalHandle<Swing>;

  //Here Node SwingAcitionServer is initialized with some options
  explicit SwingActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("swing_action_server", options)
  {
    time_ = 0.0;//here time value is initialized
    publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    //Publisher is created for joint states (moving the leg)
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Swing>(//Initialization of actions server 
      this,// Pointer to instacje of object SwingActionServer (which is type rclcpp::Node).
      "swing", // Server action name.
      std::bind(&SwingActionServer::handle_goal, this, _1, _2),//Function handling the process of accepting the goal.
      std::bind(&SwingActionServer::handle_cancel, this, _1),//Function handling the cancellation of the goal.
      std::bind(&SwingActionServer::handle_accepted, this, _1));//Function handling the acceptance of the goal.
  }

private:
  rclcpp_action::Server<Swing>::SharedPtr action_server_;//Pointer to the action server
    void publish_joint_states()//Function responsible for publishing joint states therefore moving the leg
    {
        auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
        joint_state_msg->header.stamp = now();
        joint_state_msg->name = {"LTz", "LTx", "LTy", "LSy", "LFy", "LFx"};

        // Generate sinusoidal trajectory
        joint_state_msg->position = {
            0,
            0,
            0.5 * std::sin(5.0* time_),
            0,
            0,
            0
        };

        publisher_->publish(std::move(joint_state_msg));
        RCLCPP_INFO(get_logger(), "Publishing joint states");
    }

  rclcpp_action::GoalResponse handle_goal(//Function handling the process of accepting the goal.
    const rclcpp_action::GoalUUID & uuid,//unical identifier of the goal
    std::shared_ptr<const Swing::Goal> goal)//pointer to the goal
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with duration time %f", goal->goal);//Log message about received goal (in this case- duration time)
    (void)uuid;//We do not use the uuid parameter so to avoid the warning we use (void)uuid
    //to see UID write this instead:(this->get_logger(), "Received goal request with duration time %f and UUID %s",goal->goal, uuid.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;//Return the response to the goal request
  }

  rclcpp_action::CancelResponse handle_cancel(//Function handling the cancellation of the goal.
  // handle_cancel returns object of a type: rclcpp_action::CancelResponse
    const std::shared_ptr<GoalHandleSwing> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;//we do not use the goal_handle parameter so to avoid the warning we use (void)goal_handle
    return rclcpp_action::CancelResponse::ACCEPT;//Return the response to the goal cancellation request
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSwing> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&SwingActionServer::execute, this, _1), goal_handle}.detach();
    /*new thread is created (through DETACH command it is separated from the main thread and doesn't wait for its end)
    bind creates functional object with:  (pointer to the method eexecute in the class SwingActionServer) and (pointer to the current object "this") and (_1 special place for passing arguments)
    thread runs the execute function with the argument goal_handle*/
  }

    void execute(const std::shared_ptr<GoalHandleSwing> goal_handle)//Function REALLY responsible for the movement of leg
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(10);// frequency of processing the goals = 10Hz- coherent with the time_ incrementation
        const auto goal = goal_handle->get_goal();//download the goal from the goal_handle
        auto feedback = std::make_shared<Swing::Feedback>();//Creration of the feedback object
        auto & remainedtime = feedback->timeleft;//creating a reference to the feedback variable
        remainedtime = goal->goal;//initializing the reference
        auto result = std::make_shared<Swing::Result>();//Creation of the result object

        while (time_ < goal->goal && rclcpp::ok()) {
            // Check if goal is canceled:
            if (goal_handle->is_canceling()) {
              result->success = "canceled";
              goal_handle->canceled(result);
              RCLCPP_INFO(this->get_logger(), "Goal canceled");
              return;
            }
            publish_joint_states(); // Call the publish_joint_states function

            // Update time for the next iteration
            time_ += 0.1;
            remainedtime -= 0.1;//decrement the time left
            std::cout << "Counted time: " << time_ << std::endl;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish feedback");
            loop_rate.sleep(); // Sleep to maintain the desired loop rate
        }

        // Check if goal is done
        if (rclcpp::ok()) {
            result->success = "success";
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            time_=0.0;
        }
    }  

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;//publisher for joint states (moving the leg)
    double time_;    
};  // class SwingActionServer

}  // namespace swing_action_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_humanoid_virtual_twin::SwingActionServer)//RUN THE NODE