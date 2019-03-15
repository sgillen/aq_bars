cmake_minimum_required(VERSION 2.8.3)
project(aqbar)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


##############################
# Package Setup ##############
##############################


find_package(catkin REQUIRED COMPONENTS
 roscpp
 message_generation
 std_msgs
 message_generation
 roscpp
 rospy
 genmsg 
)

find_package(Boost REQUIRED)
include_directories(${BOOST_INCLUDE_DIRECTORIES})

##############################
# Messages #########
##############################


add_message_files(
  FILES
  falconPos.msg
  falconForces.msg
  falconSetPoint.msg
  )
  
##############################
#Generation##########
##############################

generate_messages(
  DEPENDENCIES
  std_msgs
  )

  
##############################
# Catkin Setup ###############
##############################

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

  
##############################
# Build Executables ##########
##############################


add_executable(falcon_node src/falcon_node.cpp)
add_dependencies(falcon_node aqbar_generate_messages_cpp)
target_link_libraries(falcon_node ${catkin_LIBRARIES} nifalcon usb-1.0)
