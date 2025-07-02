cmake_minimum_required(VERSION 0.0.1)
project(kyon_actions)

find_package(catkin REQUIRED)


catkin_package()

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)


