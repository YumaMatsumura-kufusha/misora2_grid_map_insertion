#ifndef MISORA2_GRID_MAP_INSERTION__GRID_MAP_INSERTION_COMPONENT_HPP_
#define MISORA2_GRID_MAP_INSERTION__GRID_MAP_INSERTION_COMPONENT_HPP_

#include <memory>
#include <string>

#include <Eigen/Dense>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "misora2_common/type_adapter.hpp"

namespace misora2_grid_map_insertion
{

class GridMapInsertion : public rclcpp::Node
{
public:
  using GridMapAdaptedType = rclcpp::TypeAdapter<grid_map::GridMap, grid_map_msgs::msg::GridMap>;

  explicit GridMapInsertion(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GridMapInsertion();

private:
  void globalGridMapCallback(const grid_map::GridMap & msg);
  void localGridMapCallback(const grid_map::GridMap & msg);
  void timerCallback();
  void insertLocalGridMapAtPosition(
    grid_map::GridMap & global_grid_map, const grid_map::GridMap & local_grid_map,
    const std::string & layer, const grid_map::Position & position);

  // Publisher, Subscriber
  rclcpp::Publisher<GridMapAdaptedType>::SharedPtr pub_grid_map_;
  rclcpp::Subscription<GridMapAdaptedType>::SharedPtr sub_global_grid_map_;
  rclcpp::Subscription<GridMapAdaptedType>::SharedPtr sub_local_grid_map_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Callback Group
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  // Others
  grid_map::GridMap global_grid_map_;
  grid_map::GridMap local_grid_map_;

  // Mutex
  mutable std::mutex data_mutex_;
};

}  // namespace misora2_grid_map_insertion

#endif  // MISORA2_GRID_MAP_INSERTION__GRID_MAP_INSERTION_COMPONENT_HPP_
