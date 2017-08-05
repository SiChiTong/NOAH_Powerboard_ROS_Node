cmake_minimum_required(VERSION 2.8.3)
project(noah_powerboard)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  #  mrobot_driver_msgs 
  #  roscan
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/noah_powerboard
)

add_executable( noah_powerboard_node
    src/main.cpp
	src/uart.cpp
	src/powerboard.cpp
	#	src/system.cpp
#	src/report.cpp
)
target_link_libraries(noah_powerboard_node
  ${catkin_LIBRARIES} 
)

#############
## Install ##
#############


#############
## Testing ##
#############

