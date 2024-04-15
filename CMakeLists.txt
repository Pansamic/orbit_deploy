cmake_minimum_required(VERSION 3.10)
project(orbit_deploy C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Boost REQUIRED COMPONENTS system)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ONNX_PREFIX "x64")
  set(UNITREE_PREFIX "amd64")
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ONNX_PREFIX "aarch64")
  set(UNITREE_PREFIX "arm64")
endif()

link_directories(
    third_party/onnxruntime-linux-${ONNX_PREFIX}-1.17.1/lib
    third_party/unitree_legged_sdk/lib/cpp/${UNITREE_PREFIX}
)

add_executable(go1_locomotion 
    src/main.cpp
    src/model/orbit_base_model.cpp
    src/model/go1_rough_model.cpp
    src/model/go1_flat_model.cpp
    src/agent/go1_agent.cpp
)

target_link_libraries(go1_locomotion
    ${Boost_LIBRARIES}
    libonnxruntime.so
    libunitree_legged_sdk.a
    pthread
)

target_include_directories(go1_locomotion PRIVATE 
    include
    ${Boost_INCLUDE_DIR}
    ./third_party/unitree_legged_sdk/include
    ./third_party/onnxruntime-linux-${ONNX_PREFIX}-1.17.1/include
)

ament_target_dependencies(go1_locomotion rclcpp geometry_msgs nav_msgs)

install(TARGETS go1_locomotion
    DESTINATION lib/${PROJECT_NAME}
)

install(FILES
    third_party/onnxruntime-linux-${ONNX_PREFIX}-1.17.1/lib/libonnxruntime.so
    third_party/onnxruntime-linux-${ONNX_PREFIX}-1.17.1/lib/libonnxruntime.so.1.17.1
    DESTINATION /opt/ros/$ENV{ROS_DISTRO}/lib
)

ament_package()