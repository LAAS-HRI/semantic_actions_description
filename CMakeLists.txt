cmake_minimum_required(VERSION 3.0.2)
project(semantic_actions_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
)

find_package(Qt5 COMPONENTS Core Gui Widgets PrintSupport)
find_package(cmake_modules REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs roslib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(action_description_lib
  src/reader/ActionReader.cpp
)
add_dependencies(action_description_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(action_description_lib yaml-cpp ${catkin_LIBRARIES})

add_executable(action_description_test src/reader/test.cpp)
add_dependencies(action_description_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(action_description_test action_description_lib)

add_executable(analyse src/analyse.cpp)
add_dependencies(analyse ${catkin_EXPORTED_TARGETS})
target_link_libraries(analyse action_description_lib)

##############################################################################
# Qt Environment
##############################################################################

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}  resources/*.qrc)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

add_definitions( -std=c++11 -fPIC)

##############################################################################
# Sources
##############################################################################

set( QT_SOURCES
    src/gui/main.cpp
    src/gui/gui.cpp
    src/gui/DarkStyle.cpp
    src/gui/switch.cpp

    include/gui/gui.h
    include/gui/DarkStyle.h
    include/gui/SwitchStyle.h
    include/gui/switch.h
)
##############################################################################
# Binaries
##############################################################################

add_executable(actions_description_gui ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_include_directories(actions_description_gui
  PRIVATE
    ${catkin_INCLUDE_DIRS}
)
add_dependencies(actions_description_gui ontologenius_gencpp)
target_link_libraries(actions_description_gui
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport
)
install(TARGETS actions_description_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
