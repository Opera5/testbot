set(_AMENT_PACKAGE_NAME "testbot")
set(testbot_VERSION "0.0.0")
set(testbot_MAINTAINER "jamih <ajamiu46@email.com>")
set(testbot_BUILD_DEPENDS "control_msgs")
set(testbot_BUILDTOOL_DEPENDS "ament_cmake" "ament_cmake_python")
set(testbot_BUILD_EXPORT_DEPENDS )
set(testbot_BUILDTOOL_EXPORT_DEPENDS )
set(testbot_EXEC_DEPENDS "joint_state_publisher" "robot_state_publisher" "ros2_control" "ros2_controllers" "rviz2" "xacro" "robot_localization" "ros_ign_gazebo" "ign_ros2_control" "topic_tools")
set(testbot_TEST_DEPENDS "ament_lint_auto" "ament_lint_common")
set(testbot_GROUP_DEPENDS )
set(testbot_MEMBER_OF_GROUPS )
set(testbot_DEPRECATED "")
set(testbot_EXPORT_TAGS)
list(APPEND testbot_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
