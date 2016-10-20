cmake_minimum_required(VERSION 2.8.3)

project(refbox)

set(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(Boost REQUIRED COMPONENTS
	 filesystem
	 system
)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  refbox_msgs
  roscpp
)

set(llsf_LIB_NAMES
	 protobuf
	 llsfrbconfig
	 llsfrbcore
	 llsfrbutils
	 llsf_msgs
	 llsf_protobuf_comm
	 )

set(llsf_DIRECTORIES
	 /usr/local/llsf-refbox/lib
	 /usr/local/llsf-refbox/lib/protobuf
)

foreach(lib_NAME ${llsf_LIB_NAMES})
	 set(lib_PATH "lib_PATH-NOTFOUND")
	 find_library(lib_PATH REQUIRED
		  NAMES	 "lib${lib_NAME}.so"
		  PATHS	 ${llsf_DIRECTORIES}
	 )
    if (lib_PATH MATCHES "lib_PATH-NOTFOUND")
		  message(SEND_ERROR "${lib_NAME} not found!!!")
	 else()
		  set(llsf_LIBRARIES ${llsf_LIBRARIES} ${lib_PATH})
	 endif()
endforeach(lib_NAME)

set(llsf_INCLUDE_DIRS
	 /usr/local/llsf-refbox/src
	 /usr/local/llsf-refbox/src/libs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs refbox_msgs
  INCLUDE_DIRS include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${llsf_INCLUDE_DIRS}
  EXPORTED_TARGETS ${Boost_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
  LIBRARIES ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${llsf_LIBRARIES}
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${llsf_INCLUDE_DIRS}
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
add_dependencies(refbox_node ${Boost_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(refbox_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${llsf_LIBRARIES})

add_executable(refbox_talker src/refbox_talker.cpp)
add_dependencies(refbox_talker ${Boost_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(refbox_talker ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${llsf_LIBRARIES})
