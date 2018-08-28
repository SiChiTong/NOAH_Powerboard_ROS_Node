cmake_minimum_required(VERSION 2.8.3)
project(noah_powerboard)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mrobot_driver_msgs 
  roscan
  message_generation
)


###########
## Build ##
###########
set (CMAKE_CXX_FLAGS "-std=c++11")

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/noah_powerboard
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    remote_power_ctrl_srv.srv 
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

add_executable( noah_powerboard_node
    src/main.cpp
    src/powerboard.cpp
)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs message_runtime
)

add_dependencies(noah_powerboard_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(noah_powerboard_node
  ${catkin_LIBRARIES} 
)

#############
## Install ##
#############


#############
## Testing ##
#############


install(TARGETS noah_powerboard_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

