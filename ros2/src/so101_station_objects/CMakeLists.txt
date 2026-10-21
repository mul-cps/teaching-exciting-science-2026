cmake_minimum_required(VERSION 3.8)
project(so101_station_objects)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(FILES so101_station_objects/color_masks.slang
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/colored_objects DESTINATION lib/${PROJECT_NAME})

ament_package()
