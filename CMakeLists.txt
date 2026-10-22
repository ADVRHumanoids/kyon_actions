cmake_minimum_required(VERSION 3.8)
project(kyon_actions)

find_package(ament_cmake REQUIRED)

option(INSTALL_SIMULATION_TOOLS "Install optional MuJoCo and plotting commands" ON)

install(PROGRAMS replayer_ros2.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES trajectory_io.py DESTINATION lib/${PROJECT_NAME})
if(INSTALL_SIMULATION_TOOLS)
  install(PROGRAMS replay_sim_ros2.py plot_experiment.py DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
endif()
install(DIRECTORY config mat_files
  DESTINATION share/${PROJECT_NAME})

ament_package()
