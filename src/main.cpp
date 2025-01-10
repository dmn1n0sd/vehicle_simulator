#include <memory>
#include "vehicle_simulator/vehicle_simulator.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleSimulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
