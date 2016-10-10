cmake_minimum_required(VERSION 2.8.3)

project(refbox)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  refbox_msgs
  roscpp
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs refbox_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(H_FILES
  include/EnumConverter.h
  include/NodeController.h
  include/refbox/RefereeBoxNode.h
  include/refbox/phases/GamePhase.h
  include/refbox/phases/GamePhaseConverter.h
  include/refbox/states/GameState.h
  include/refbox/states/GameStateConverter.h
  include/refbox/states/PuckState.h
  include/refbox/states/PuckStateConverter.h
)

set(CPP_FILES
  src/NodeController.cpp
  src/refbox/RefereeBoxNode.cpp
  src/refbox/phases/GamePhaseConverter.cpp
  src/refbox/states/GameStateConverter.cpp
  src/refbox/states/PuckStateConverter.cpp
)

add_executable(refbox_node src/refbox_node.cpp ${CPP_FILES})
add_dependencies(refbox_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(refbox_node ${catkin_LIBRARIES})
