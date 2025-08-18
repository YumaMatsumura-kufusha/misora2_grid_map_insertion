#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "misora2_grid_map_insertion/grid_map_insertion_component.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  auto node = std::make_shared<misora2_grid_map_insertion::GridMapInsertion>();
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
