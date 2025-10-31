cmake_minimum_required(VERSION 3.8)
project(qi_unipa_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
#ament_python_install_package(src)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bumper.msg"
  "msg/Sonar.msg"
  "msg/Track.msg"
  "srv/SetState.srv"
  "srv/SetHand.srv"
  "srv/SetPosture.srv"
  "srv/SetJointAngles.srv"
  "srv/GetPosition.srv"
  "action/Walking.action"
  "action/Talking.action"
  "action/Recording.action"
  "action/RecordingNoMic.action"
  "action/Ending.action"
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()