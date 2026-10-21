cmake_minimum_required(VERSION 3.8)
project(kyon_actions)

find_package(ament_cmake REQUIRED)

install(PROGRAMS replay_sim_ros2.py replayer_ros2.py plot_experiment.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config mat_files launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
