#include <functional>
#include <memory>
#include <thread>

#include "swing_action/action/leg_swing.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace ros2_humanoid_virtual_twin
{
class SwingActionServer : public rclcpp::Node//Tworzymy klase SwingActionseerver dziedziczaca po klasie Node
{
public:
  using Swing = swing_action::action::LegSwing;
  using GoalHandleSwing = rclcpp_action::ServerGoalHandle<Swing>;

  //ACTION_TUTORIALS_CPP_PUBLIC
  explicit SwingActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("swing_action_server", options)//inicjalizacja wezla Node z nazwa swing_action_server i opcjami zdefiniowanymi linijke wyzej
  {
    time_ = 0.0;
    publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Swing>(//inicjlaizacja serwera akcji (przekazujemy wskaznik do tego serwera dla klasy SwingActionServer (opisanej tu jako "this"))
      this,// Wskaźnik do instancji obiektu SwingActionServer (który jest typu rclcpp::Node).
      "swing", // Nazwa serwera akcji.
      std::bind(&SwingActionServer::handle_goal, this, _1, _2),// Funkcja obsługująca przyjmowanie celu.
      std::bind(&SwingActionServer::handle_cancel, this, _1),// Funkcja obsługująca żądanie anulowania akcji.
      std::bind(&SwingActionServer::handle_accepted, this, _1));// Funkcja obsługująca akceptację celu (po udanej inicjacji).
  }

private:
  rclcpp_action::Server<Swing>::SharedPtr action_server_;//Wskaźnik do serwera akcji.
    void publish_joint_states()
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

  rclcpp_action::GoalResponse handle_goal(//Funkcja obsługująca przyjmowanie celu.
    const rclcpp_action::GoalUUID & uuid,//Unikalny identyfikator celu.
    std::shared_ptr<const Swing::Goal> goal)//Wskaźnik do celu.
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with duration time %f", goal->goal);//Logujemy informacje o otrzymanym celu.
    (void)uuid;//Zapobiegamy ostrzeżeniu o nieużytej zmiennej.
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;//Zwracamy odpowiedź na otrzymany cel.
  }

  rclcpp_action::CancelResponse handle_cancel(//Funkcja obsługująca żądanie anulowania akcji.
  // handle_cancel zwraca obiekt typu rclcpp_action::CancelResponse
    const std::shared_ptr<GoalHandleSwing> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSwing> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&SwingActionServer::execute, this, _1), goal_handle}.detach();
    /*Tworzymy nowy watek (ktory poprzez uzycie detach odlacza sie od glownego watku i nie czeka na jego zakonczenie)
    bind tworzy obiekt funkcyjny z:  (wskaznikiem do metody eexecute w klasie SwingActionServer) i (wskaznikiem do biezacego obiektu "this") i (_1 specjalne miejsce przekazywania argumentow)
     watek uruchamia funkcje execute z argumentem goal_handle*/ 
  }

    void execute(const std::shared_ptr<GoalHandleSwing> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(10);// frequency of processing the goals = 10Hz
        const auto goal = goal_handle->get_goal();//download the goal from the goal_handle
        auto feedback = std::make_shared<Swing::Feedback>();
        auto & remainedtime = feedback->timeleft;//creating a reference to the feedback variable
        remainedtime = goal->goal;//initializing the reference
        auto result = std::make_shared<Swing::Result>();//Tworzenie obiektu result

        while (time_ < goal->goal && rclcpp::ok()) {
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
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    double time_;    
};  // class SwingActionServer

}  // namespace swing_action_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_humanoid_virtual_twin::SwingActionServer)