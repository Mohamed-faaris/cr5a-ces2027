cmake_minimum_required(VERSION 3.8)
project(pgea_gripper_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  srv/GetRegister.srv srv/SetRegister.srv srv/GetStatus.srv
  srv/SetIoGroup.srv srv/Jog.srv srv/ClearAlarm.srv
  action/GripUntilForce.action DEPENDENCIES action_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
