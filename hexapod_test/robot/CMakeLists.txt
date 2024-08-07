cmake_minimum_required(VERSION 3.16)
project(robot)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Euler.msg"
    "msg/FootState.msg"
    "msg/BodyState.msg"
    "msg/InitCmd.msg"
    "msg/JointCmd.msg"
    "msg/FootCmd.msg"
    "msg/PlanCmd.msg"
    "msg/PlanFdb.msg"
    "msg/HlcState.msg"
    DEPENDENCIES std_msgs geometry_msgs
)

if($ENV{ROS_DISTRO} STREQUAL "foxy")
    install(
        DIRECTORY include/
        DESTINATION include/
    )
endif()

if($ENV{ROS_DISTRO} STREQUAL "humble")
    install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
    )
    install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
    )
    install(
        DIRECTORY urdf
        DESTINATION share/${PROJECT_NAME}/
    )
    install(
        DIRECTORY world
        DESTINATION share/${PROJECT_NAME}/
    )
endif()

if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
endif()

ament_package()
