cmake_minimum_required(VERSION 3.5)
project(dobot_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

install(PROGRAMS scripts/joint_state_mimic_relay.py DESTINATION lib/${PROJECT_NAME})
