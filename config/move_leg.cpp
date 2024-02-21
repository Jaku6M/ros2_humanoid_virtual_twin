#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class MoveLegNode : public rclcpp::Node
{
public:
    MoveLegNode() : Node("move_leg_node")
    {
        publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&MoveLegNode::publish_joint_states, this));
        
        // Initialize member variables
        x_ = 0.0;
        y_ = 0.0;
        z_ = 0.0;
        a_ = 0.0;
        b_ = 0.0;
        c_ = 0.0;
    }

private:
    void publish_joint_states()
    {
        auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
        joint_state_msg->header.stamp = now();
        joint_state_msg->name = {"LTz", "LTx", "LTy", "LSy", "LFy", "LFx"};
        joint_state_msg->position = {x_, y_, z_, a_, b_, c_};
        joint_state_msg->velocity = {};  // You can set velocities if needed
        joint_state_msg->effort = {};    // You can set efforts if needed

        // Update member variables for the next iteration
        x_ += 0.1;
        y_ -= 0.1;
        z_ += 0.1;
        a_ += 0.1;
        b_ += 0.1;
        c_ -= 0.1;

        publisher_->publish(std::move(joint_state_msg));
        RCLCPP_INFO(get_logger(), "Publishing joint states");
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Member variables for continuous movement
    float x_, y_, z_, a_, b_, c_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveLegNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
