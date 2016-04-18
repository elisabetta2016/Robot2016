cmake_minimum_required(VERSION 2.8.3)
project(donkey_rover)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  tf
  nav_msgs
  message_generation
)

add_message_files(
   DIRECTORY msg
   FILES
  Scanner_Command.msg  
  Rover_Power_Data.msg 
  Rover_Scanner.msg
  Rover_Track_Bogie_Angle.msg
  Rover_Track_Speed.msg
 )

generate_messages(
   DEPENDENCIES
   donkey_rover
   std_msgs
)
 
catkin_package(
#  INCLUDE_DIRS include LIBRARIES Rover donkey_rover 
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
## generate_messages(
##   DEPENDENCIES
##   std_msgs
## )




include_directories(
  include
  LibRover
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
)



##set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
##set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -I../LibRover")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L../LibRover")

add_library(Rover LibRover/BogieScanner.cc LibRover/CANopenHelper.cc LibRover/CANopenMaster.cc LibRover/Drive.cc LibRover/Track.cc LibRover/libRover.cc)
## Declare a C++ executable
##add_executable(rovstate src/rovstate.cpp)
add_executable(rover_odom src/rover_odom.cpp)
#add_executable(rover_drive src/rover_drive.cpp)
#add_executable(rover_joy src/rover_joy.cpp)
#add_executable(rover_scannerangle src/rover_scannerangle.cpp) 



## add_executable(rovstate src/rovstate.cpp src/libRover.cc src/BogieScanner.cc src/CANopenHelper.cc src/CANopenMaster.cc src/Drive.cc src/Track.cc)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(donkey_rover_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(rover_odom ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
##target_link_libraries(rovstate Rover
##  ${catkin_LIBRARIES}
##  ${Boost_INCLUDE_DIRS}
##  ${GSTREAMER_INCLUDE_DIRS}
## )
target_link_libraries(rover_odom Rover
  ${catkin_LIBRARIES}
  ${Boost_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
 )
#target_link_libraries(rover_drive Rover
#  ${catkin_LIBRARIES}
#  ${Boost_INCLUDE_DIRS}
#  ${GSTREAMER_INCLUDE_DIRS}
# )
#target_link_libraries(rover_joy Rover
#  ${catkin_LIBRARIES}
#  ${Boost_INCLUDE_DIRS} 
#  ${GSTREAMER_INCLUDE_DIRS}
# )


#target_link_libraries(rover_scannerangle Rover
#  ${catkin_LIBRARIES}
#  ${Boost_INCLUDE_DIRS}
#  ${GSTREAMER_INCLUDE_DIRS}
# )

install(TARGETS Rover
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )


##add_subdirectory(include)



