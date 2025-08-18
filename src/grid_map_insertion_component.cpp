#include "misora2_grid_map_insertion/grid_map_insertion_component.hpp"

namespace misora2_grid_map_insertion
{

GridMapInsertion::GridMapInsertion(const rclcpp::NodeOptions & options)
: Node("grid_map_insertion_node", options)
{
  // Parameters
  double publish_rate;

  this->declare_parameter<double>("publish_rate", 10.0);

  this->get_parameter("publish_rate", publish_rate);

  double publish_dt;
  publish_dt = 1.0 / std::max(publish_rate, 0.1);

  // Publisher
  pub_grid_map_ = this->create_publisher<GridMapAdaptedType>("insertion_grid_map", 1);

  // Callback Group
  rclcpp::CallbackGroup::SharedPtr map_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto map_sub_opt = rclcpp::SubscriptionOptions();
  map_sub_opt.callback_group = map_callback_group;

  // Subscriber
  sub_global_grid_map_ = this->create_subscription<GridMapAdaptedType>(
    "global_grid_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&GridMapInsertion::globalGridMapCallback, this, std::placeholders::_1), map_sub_opt);
  sub_local_grid_map_ = this->create_subscription<GridMapAdaptedType>(
    "local_grid_map", 1,
    std::bind(&GridMapInsertion::localGridMapCallback, this, std::placeholders::_1), map_sub_opt);

  // Timer
  timer_ = rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Duration::from_seconds(publish_dt),
    std::bind(&GridMapInsertion::timerCallback, this), timer_callback_group_);
}

GridMapInsertion::~GridMapInsertion()
{
}

void GridMapInsertion::globalGridMapCallback(const grid_map::GridMap & msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  global_grid_map_ = msg;
}

void GridMapInsertion::localGridMapCallback(const grid_map::GridMap & msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  local_grid_map_ = msg;
}

void GridMapInsertion::timerCallback()
{
  grid_map::GridMap global_grid_map;
  grid_map::GridMap local_grid_map;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    global_grid_map = global_grid_map_;
    local_grid_map = local_grid_map_;
  }

  // 挿入位置はダミー（本当は現在の位置）
  grid_map::Position insert_pos(1.0, 1.0);

  insertLocalGridMapAtPosition(global_grid_map, local_grid_map, "elevation", insert_pos);

  pub_grid_map_->publish(global_grid_map);
}

void GridMapInsertion::insertLocalGridMapAtPosition(
  grid_map::GridMap & global_grid_map, const grid_map::GridMap & local_grid_map,
  const std::string & layer, const grid_map::Position & position)
{
  grid_map::Index index_center;
  if (!global_grid_map_.getIndex(position, index_center)) {
    RCLCPP_ERROR(
      this->get_logger(), "Position (%.2f, %.2f) is outside the global grid map.",
      position.x(), position.y());
    return;
  }

  const auto & local_size = local_grid_map.getSize();
  const int rows = static_cast<int>(local_size(0));
  const int cols = static_cast<int>(local_size(1));
  const int half_rows = rows / 2;
  const int half_cols = cols / 2;

  auto & global_mat = global_grid_map[layer].matrix();
  const auto & local_mat = local_grid_map[layer].matrix();

  const int start_row = static_cast<int>(index_center.x()) - half_rows;
  const int start_col = static_cast<int>(index_center.y()) - half_cols;

  const int global_rows = global_mat.rows();
  const int global_cols = global_mat.cols();

  const int clamp_row_begin = std::max(0, start_row);
  const int clamp_col_begin = std::max(0, start_col);
  const int clamp_row_end = std::min(global_rows, start_row + rows);
  const int clamp_col_end = std::min(global_cols, start_col + cols);

  const int block_rows = clamp_row_end - clamp_row_begin;
  const int block_cols = clamp_col_end - clamp_col_begin;

  if (block_rows <= 0 || block_cols <= 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "No overlap between local grid map and global grid map; nothing to insert.");
    return;
  }

  const int local_row_begin = clamp_row_begin - start_row;
  const int local_col_begin = clamp_col_begin - start_col;

  global_mat.block(clamp_row_begin, clamp_col_begin, block_rows, block_cols) =
    local_mat.block(local_row_begin, local_col_begin, block_rows, block_cols);
}

}  // namespace misora2_grid_map_insertion

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(misora2_grid_map_insertion::GridMapInsertion)
