cmake_minimum_required(VERSION 2.8.3)

project(refbox_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
	FILES
	Exploration.msg
	Game.msg
	Machines.msg
	Order.msg
	)

add_service_files(
	FILES
	ReportMachine.srv
	)

generate_messages(
	DEPENDENCIES
	std_msgs
	)

catkin_package(
	 CATKIN_DEPENDS message_runtime std_msgs
	 )
