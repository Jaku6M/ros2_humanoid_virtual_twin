#include "rclcpp/rclcpp.hpp"
#include "ros2_humanoid_virtual_twin/srv/legmove.hpp"                                        

#include <memory>

void add(const std::shared_ptr<ros2_humanoid_virtual_twin::srv::Legmove::Request> request,     
          std::shared_ptr<ros2_humanoid_virtual_twin::srv::Legmove::Response>       response)  
{
  response->frequencygot = request->frequency;                                      
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %f" , request->frequency);                                         
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%f]", (float)response->frequencygot);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("legmove_server");   

  rclcpp::Service<ros2_humanoid_virtual_twin::srv::Legmove>::SharedPtr service =               
    node->create_service<ros2_humanoid_virtual_twin::srv::Legmove>("legmove",  &add);   

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to set frequency.");                     

  rclcpp::spin(node);
  rclcpp::shutdown();
}