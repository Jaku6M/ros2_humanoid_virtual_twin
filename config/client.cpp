#include "rclcpp/rclcpp.hpp"
#include "ros2_humanoid_virtual_twin/srv/legmove.hpp"                                       // CHANGE

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4) { // CHANGE
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set frequency of robot movement");      // CHANGE
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("client");  // CHANGE
  rclcpp::Client<ros2_humanoid_virtual_twin::srv::Legmove>::SharedPtr client =                // CHANGE
    node->create_client<ros2_humanoid_virtual_twin::srv::Legmove>("Legmove");          // CHANGE

  auto request = std::make_shared<ros2_humanoid_virtual_twin::srv::Legmove::Request>();       // CHANGE
  request->frequency = atoll(argv[1]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Set frequency: %2.f", result.get()->frequencygot);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service Legmove");    // CHANGE
  }

  rclcpp::shutdown();
  return 0;
}