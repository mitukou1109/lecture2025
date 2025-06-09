cmake_minimum_required(VERSION 3.8)
project(choreonoid_tutorial)

find_package(ament_cmake_auto REQUIRED)
find_package(Torch REQUIRED)

ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotObservation.msg"
  DEPENDENCIES
    std_msgs
    sensor_msgs
    trajectory_msgs
)

rosidl_get_typesupport_target(${PROJECT_NAME}_cpp_typesupport
  ${PROJECT_NAME} rosidl_typesupport_cpp)

choreonoid_add_simple_controller(StandController src/StandController.cpp)
set_target_properties(StandController PROPERTIES
  CXX_STANDARD ${CHOREONOID_CXX_STANDARD}
  CMAKE_CXX_EXTENSIONS OFF
)

choreonoid_add_simple_controller(ROSChoreonoidBridgeController src/ROSChoreonoidBridgeController.cpp)
target_link_libraries(ROSChoreonoidBridgeController "${${PROJECT_NAME}_cpp_typesupport}")
ament_target_dependencies(ROSChoreonoidBridgeController
  rclcpp
  sensor_msgs
  std_msgs
  trajectory_msgs
)
set_target_properties(ROSChoreonoidBridgeController PROPERTIES
  CXX_STANDARD ${CHOREONOID_CXX_STANDARD}
  CMAKE_CXX_EXTENSIONS OFF
)

# inference
choreonoid_add_simple_controller(InferenceController src/InferenceController.cpp)
target_link_libraries(InferenceController ${CHOREONOID_BASE_LIBRARIES} ${TORCH_LIBRARIES})
set_target_properties(InferenceController PROPERTIES
  CXX_STANDARD 17
  CMAKE_CXX_EXTENSIONS OFF
)

ament_auto_package()
