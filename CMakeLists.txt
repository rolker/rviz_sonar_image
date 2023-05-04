cmake_minimum_required(VERSION 3.0.2)
project(rviz_sonar_image)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  marine_acoustic_msgs
  pluginlib
  roscpp
  rviz
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_AUTOMOC ON)

find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

set(HDR_FILES
  include/rviz_sonar_image/color_map.h
  include/rviz_sonar_image/projected_sonar_image_curtain.h
  include/rviz_sonar_image/projected_sonar_image_display.h
  include/rviz_sonar_image/projected_sonar_image_fan.h
  include/rviz_sonar_image/sonar_image_curtain.h
  include/rviz_sonar_image/sonar_image_display.h
  include/rviz_sonar_image/sonar_image_fan.h
)

set(SRC_FILES
  src/color_map.cpp
  src/projected_sonar_image_curtain.cpp
  src/projected_sonar_image_display.cpp
  src/projected_sonar_image_fan.cpp
  src/sonar_image_curtain.cpp
  src/sonar_image_display.cpp
  src/sonar_image_fan.cpp
)

add_library(${PROJECT_NAME} ${SRC_FILES} ${HDR_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
