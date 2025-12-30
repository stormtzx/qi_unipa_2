cmake_minimum_required(VERSION 3.8)
project(qi_unipa_2_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dipendenze per interfacce
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  


# Genera interfacce (msg/srv/action)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bumper.msg"
  "msg/Sonar.msg"
  "msg/HeadTouch.msg"
  "msg/HandTouch.msg"
  "msg/Battery.msg"
  "msg/Infrared.msg"
  "msg/IMU.msg"
  "msg/UserData.msg"
  "msg/TestData.msg"
  "msg/TrackingState.msg"
  "srv/SetState.srv"
  "srv/SetHand.srv"
  "srv/SetPosture.srv"
  "srv/SetJointAngles.srv"
  "srv/StartTracking.srv"
  "srv/StopTracking.srv"
  "srv/GetPosition.srv"
  "srv/GetImage.srv"
  "srv/GetEmotion.srv"
  "srv/GetCameraCoordinates.srv"
  "srv/GetTrackedSubjectCoordinates.srv"
  "srv/PlayAnimation.srv"
  "action/Moving.action"
  "action/Walking.action"
  "action/Navigating.action"
  "action/Talking.action"
  "action/Listening.action"
  "action/Browsing.action"
  DEPENDENCIES std_msgs action_msgs sensor_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
