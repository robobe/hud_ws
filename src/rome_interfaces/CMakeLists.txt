cmake_minimum_required(VERSION 3.8)
project(rome_interfaces VERSION 0.2.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(joystick_interface_DIR build_arm/joystick_interface/ament_cmake_core/)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(joystick_interface REQUIRED)

set(msg_files
  "msg/TrackerResult.msg"
  "msg/TrackerRequest.msg"
  "msg/BoardState.msg"
  "msg/AttitudeControl.msg"
  "msg/StateMachine.msg"
  "msg/PIDdiagnostic.msg"
  "msg/RomeJoystick.msg"
  "msg/GCSTelemetry.msg"
  "msg/GimbalStatus.msg"
  # etc
  )

set(srv_files
  "srv/GimbalMode.srv"
  "srv/CommandLong.srv"
  "srv/SetPoint.srv"
  "srv/SetString.srv"
  "srv/GetList.srv"
   # etc
  )

rosidl_generate_interfaces(${PROJECT_NAME}
${msg_files}
${srv_files}
DEPENDENCIES
  geometry_msgs
  std_msgs
  diagnostic_msgs
  sensor_msgs
  joystick_interface
 )
ament_export_dependencies(rosidl_default_runtime)

if(DEFINED OUTPUT_FOLDER)
  include(Packing)
endif()
ament_package()
