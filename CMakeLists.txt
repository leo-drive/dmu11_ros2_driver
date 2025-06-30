cmake_minimum_required(VERSION 3.8)
project(dmu11_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies

# Enable ament_cmake_auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


ament_auto_add_library(dmu11_driver SHARED
        src/serial_port.cpp
        src/dmu11.cpp
        src/dmu11_parser.cpp
)

# List of message files
set(msg_files
        "msg/DmuRaw.msg"
)

# Generate ROS 2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
          ${msg_files}
          DEPENDENCIES
            std_msgs
            geometry_msgs
        )

     
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(dmu11_driver "${cpp_typesupport_target}") 


rclcpp_components_register_node(dmu11_driver
        PLUGIN "DMU11::Dmu11Receiver"
        EXECUTABLE dmu11_driver_node
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
        INSTALL_TO_SHARE
        launch
        config
)
