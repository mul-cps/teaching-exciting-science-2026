cmake_minimum_required(VERSION 3.8)
project(so101_station_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ../../config/ DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS scripts/plan_only_smoke.py DESTINATION lib/${PROJECT_NAME} RENAME plan_only_smoke)

ament_package()
