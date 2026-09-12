# 【讲解】接口包：只定义 PickPlace 动作，由 rosidl 生成 C++/Python 代码。与逻辑包分开是 ROS 2 的惯例（接口包用 CMake，逻辑包可以是纯 Python）。
cmake_minimum_required(VERSION 3.8)
project(mecharm_sort_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickPlace.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
