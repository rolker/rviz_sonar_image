cmake_minimum_required(VERSION 3.14.4)
project(rviz_sonar_image)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)

set(CMAKE_AUTOMOC ON)

qt5_wrap_cpp(MOC_FILES
  include/rviz_sonar_image/color_map.h
  include/rviz_sonar_image/projected_sonar_image_curtain.h
  include/rviz_sonar_image/projected_sonar_image_display.h
  include/rviz_sonar_image/projected_sonar_image_fan.h
  include/rviz_sonar_image/sonar_image_curtain.h
  include/rviz_sonar_image/sonar_image_display.h
  include/rviz_sonar_image/sonar_image_fan.h
)

add_library(${PROJECT_NAME}
  src/color_map.cpp
  src/projected_sonar_image_curtain.cpp
  src/projected_sonar_image_display.cpp
  src/projected_sonar_image_fan.cpp
  src/sonar_image_curtain.cpp
  src/sonar_image_display.cpp
  src/sonar_image_fan.cpp
  ${MOC_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  "marine_acoustic_msgs"
  "pluginlib"
  "rviz_common"
  "rviz_default_plugins"
  "rclcpp"
)

install (TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
pluginlib_export_plugin_description_file(rviz_common plugin.xml)

ament_package()
