cmake_minimum_required(VERSION 3.8)
project(41068_ignition_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(ros_ign_interfaces REQUIRED)

install(
  DIRECTORY config launch models urdf_husky worlds urdf_parrot maps behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

# Shared Python helpers used by the demo scripts and the test suite. The ROS
# package name starts with a digit and so cannot be a Python module name; the
# module is therefore called rs1_nav.
ament_python_install_package(rs1_nav)

install(
  PROGRAMS
    scripts/dynamic_world_demo.py
    scripts/basic_autonomy_demo.py
    scripts/odometry_tf_broadcaster.py
    scripts/obstacle_injector.py
    scripts/robot_status_gui.py
    scripts/thermal_camera_gui.py
    scripts/wasd_teleop.py
    scripts/fire_hazard_nav_bridge.py
    scripts/prior_map_publisher.py
    scripts/path_bank_manager.py
    scripts/rebuild_bush_trail_nav_geometry.py
    scripts/generate_hilly_heightmap.py
    scripts/generate_custom_terrain_heightmap.py
    scripts/generate_terrain_sources_3.py
    scripts/install_terrain_sources_3_gazebo.py
    scripts/enhance_bush_trail_walls.py
    scripts/flatten_bush_trail_floors.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
