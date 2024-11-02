cmake_minimum_required(VERSION 3.5)
project(webrtc_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(async_web_server_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(webrtc_ros_msgs REQUIRED)
find_package(webrtc REQUIRED)
find_package(X11 REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgcodecs imgproc videoio)


###########
## Build ##
###########

include_directories(
  include
  include/${PROJECT_NAME}
  ${OpenCV_INCLUDE_DIRS}
  ${webrtc_ros_msgs_INCLUDE_DIRS}
  ${webrtc_INCLUDE_DIRS}
  
)
add_definitions(${webrtc_DEFINITIONS})


add_executable(${PROJECT_NAME}_server_node
  src/webrtc_ros_server_node.cpp
  src/webrtc_web_server.cpp
  src/configure_message.cpp
  src/ice_candidate_message.cpp
  src/image_transport_factory.cpp
  src/ros_video_capturer.cpp
  src/ros_video_renderer.cpp
  src/sdp_message.cpp
  src/webrtc_client.cpp
  src/webrtc_ros_message.cpp
  src/webrtc_ros_server.cpp
  src/webrtc_ros_json_parser.cpp
)

target_include_directories(${PROJECT_NAME}_server_node
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PROJECT_NAME}_server_node
    async_web_server_cpp
    cv_bridge
    image_transport
    rclcpp
    std_msgs
    webrtc_ros_msgs
)

target_link_libraries(${PROJECT_NAME}_server_node 
  ${webrtc_LIBRARIES}
  webrtc
  jsoncpp_lib
  ${X11_LIBRARIES}
)

set_target_properties(${PROJECT_NAME}_server_node PROPERTIES COMPILE_OPTIONS "-std=c++17")

add_library(${PROJECT_NAME} SHARED
  src/burger.cpp
  src/webrtc_ros_client_node.cpp
  src/cv_mat_sensor_msgs_image_type_adapter.cpp
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "IMAGE_TOOLS_BUILDING_DLL")
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp_components::component
  ${OpenCV_LIBS})

rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "webrtc_ros::Cam2Image" EXECUTABLE cam2image)

#############
## Install ##
#############
install(TARGETS
  ${PROJECT_NAME}_server_node
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# Install web files
install(DIRECTORY web
  DESTINATION share/${PROJECT_NAME}
  PATTERN "Gruntfile.js" EXCLUDE
  PATTERN "node_modules" EXCLUDE
  PATTERN "package.json" EXCLUDE
  PATTERN ".gitignore" EXCLUDE
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install license files
install(FILES
  LICENSE
  DESTINATION share/${PROJECT_NAME}/
)
install(FILES
  LICENSE_webrtc_adapter
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}/scripts
)


ament_package()
