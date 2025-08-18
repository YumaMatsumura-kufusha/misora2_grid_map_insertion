cmake_minimum_required(VERSION 3.8)
project(misora2_grid_map_insertion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(grid_map_insertion_executable grid_map_insertion)
set(grid_map_insertion_library ${grid_map_insertion_executable}_core)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

# create the executor target
ament_auto_add_executable(${grid_map_insertion_executable}
  src/grid_map_insertion_node.cpp
  src/grid_map_insertion_component.cpp
)
target_include_directories(${grid_map_insertion_executable} PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
)

# create the component target
ament_auto_add_library(${grid_map_insertion_library} SHARED
  src/grid_map_insertion_component.cpp
)
target_include_directories(${grid_map_insertion_executable} PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
)
rclcpp_components_register_nodes(${grid_map_insertion_library}
  "misora2_grid_map_insertion::GridMapInsertion"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright ament_cmake_clang_format)
  ament_lint_auto_find_test_dependencies()
  ament_clang_format(
    CONFIG_FILE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../.clang-format>
  )
endif()

ament_auto_package()
