cmake_minimum_required(VERSION 2.8.3)
project(thubot)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  #catkin_make
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  nav_msgs
  message_filters
  riki_msgs
)
find_package(Qt5  REQUIRED COMPONENTS  Widgets  )

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES
  src/mainwindow.cpp
  src/qnode.cpp
  src/main.cpp
)

set(FORMS
  src/mainwindow.ui
)

catkin_package(
)

include_directories(
    src
    ${catkin_INCLUDE_DIRS}
)

 add_executable(thubot ${SOURCES} ${FORMS} )
 target_link_libraries(thubot  Qt5::Widgets ${catkin_LIBRARIES})




