cmake_minimum_required(VERSION 3.8)
project(inference_tutorial)

find_package(ament_cmake_auto REQUIRED)
find_package(Torch REQUIRED)

ament_auto_find_build_dependencies()

add_executable(inference src/inference.cpp)
target_link_libraries(inference ${TORCH_LIBRARIES})
set_target_properties(inference PROPERTIES
  CXX_STANDARD 17
  CMAKE_CXX_STANDARD_REQUIRED ON
  CMAKE_CXX_EXTENSIONS OFF
)

ament_auto_package()
