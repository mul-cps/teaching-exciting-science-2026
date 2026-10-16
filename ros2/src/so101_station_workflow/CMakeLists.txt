cmake_minimum_required(VERSION 3.8)
project(so101_station_workflow)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CommissionGripperStep.srv" "srv/SelectPlacementRegion.srv" "srv/EV3Command.srv" "srv/SorterPosition.srv" "srv/RunCycles.srv" "srv/ConfirmReference.srv" "srv/SelectDestination.srv")
install(PROGRAMS scripts/station_api DESTINATION lib/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
