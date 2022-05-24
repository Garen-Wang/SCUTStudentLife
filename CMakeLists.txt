cmake_minimum_required(VERSION 3.22)
project(SCUTStudentLife)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)


find_package(Qt5 COMPONENTS
  Core
  Gui
  Widgets
  REQUIRED)


add_executable(SCUTStudentLife
  src/main.cpp
  include/config.h include/json.hpp
  src/mainwindow.cpp include/mainwindow.h src/mainwindow.ui
  src/agent.cpp include/agent.h
  src/map.cpp include/map.h
  src/reader.cpp include/reader.h
  src/schedule.cpp include/schedule.h
  src/social-force.cpp include/social-force.h
  src/wall.cpp include/wall.h
  src/loader.cpp include/loader.h
)

target_include_directories(SCUTStudentLife
  PUBLIC
  include
)

target_link_libraries(SCUTStudentLife
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  )

