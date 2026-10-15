cmake_minimum_required(VERSION 3.8)
project(kyon_actions)

find_package(ament_cmake REQUIRED)

install(PROGRAMS replay_sim_ros2.py plot_experiment.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config mat_files
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(replay_validation test_replay_sim_ros2.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
