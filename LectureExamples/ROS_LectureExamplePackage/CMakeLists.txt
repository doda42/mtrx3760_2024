cmake_minimum_required(VERSION 3.8)
project(mtrx3760)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(mtrx3760 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomMessage.msg"
  "action/MoveRobot.action"
  DEPENDENCIES
)


add_executable(simplePublisher src/simple_publisher.cpp)
ament_target_dependencies(simplePublisher rclcpp geometry_msgs)

add_executable(simpleSubscriber src/simple_subscriber.cpp)
ament_target_dependencies(simpleSubscriber rclcpp geometry_msgs)

add_executable(simplePublisherCustom src/simple_publisher_custom.cpp)
ament_target_dependencies(simplePublisherCustom rclcpp mtrx3760)

add_executable(simpleSubscriberCustom src/simple_subscriber_custom.cpp)
ament_target_dependencies(simpleSubscriberCustom rclcpp mtrx3760)

add_executable(staticBroadcast src/static_broadcast.cpp)
ament_target_dependencies(staticBroadcast rclcpp tf2 tf2_ros)

add_executable(dynamicBroadcast src/dynamic_broadcast.cpp)
ament_target_dependencies(dynamicBroadcast rclcpp tf2 tf2_ros)

add_executable(frameListener src/frame_listener.cpp)
ament_target_dependencies(frameListener rclcpp tf2 tf2_ros)

add_executable(timeTraveller src/time_traveller.cpp)
ament_target_dependencies(timeTraveller rclcpp tf2 tf2_ros)

add_executable(nodeWithParam src/node_with_param.cpp)
ament_target_dependencies(nodeWithParam rclcpp tf2 tf2_ros)

add_executable(dynamicReconfiguration src/dynamic_reconfiguration.cpp)
ament_target_dependencies(dynamicReconfiguration rclcpp tf2 tf2_ros)

add_executable(serviceServer src/service_server.cpp)
ament_target_dependencies(serviceServer rclcpp example_interfaces)

add_executable(serviceClient src/service_client.cpp)
ament_target_dependencies(serviceClient rclcpp example_interfaces)

add_executable(actionServer src/action_server.cpp)
ament_target_dependencies(actionServer rclcpp mtrx3760 action_msgs rclcpp_action)

add_executable(actionClient src/action_client.cpp)
ament_target_dependencies(actionClient rclcpp mtrx3760 action_msgs rclcpp_action)

add_executable(lifecycleNode src/lifecycle_node.cpp)
ament_target_dependencies(lifecycleNode rclcpp_lifecycle std_msgs lifecycle_msgs)

add_executable(simpleCameraDisplay src/simple_camera_display.cpp)
ament_target_dependencies(simpleCameraDisplay rclcpp std_msgs sensor_msgs cv_bridge)
target_link_libraries(simpleCameraDisplay ${OpenCV_LIBRARIES})

add_executable(aprilTagDetection src/april_tag_detection.cpp)
ament_target_dependencies(aprilTagDetection rclcpp std_msgs sensor_msgs cv_bridge)
target_link_libraries(aprilTagDetection ${OpenCV_LIBRARIES})

add_executable(colourDetection src/colour_detection.cpp)
ament_target_dependencies(colourDetection rclcpp std_msgs sensor_msgs cv_bridge)
target_link_libraries(colourDetection ${OpenCV_LIBRARIES})

install(TARGETS
  simplePublisher
  simpleSubscriber
  simplePublisherCustom
  simpleSubscriberCustom
  staticBroadcast
  dynamicBroadcast
  frameListener
  timeTraveller
  nodeWithParam
  dynamicReconfiguration
  serviceServer
  serviceClient
  actionServer
  actionClient
  lifecycleNode
  simpleCameraDisplay
  aprilTagDetection
  colourDetection
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
