cmake_minimum_required(VERSION 3.0.2)
project(doser_bot_package)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
  dynamic_reconfigure
  sensor_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV 4 REQUIRED)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIR})
    message(STATUS "OpenCV ${OpenCV_VERSION} FOUND")
    message(STATUS ${OpenCV_LIBS})
    if(${OpenCV_VERSION_MAJOR} EQUAL 4)
        add_definitions(-DOPENCV_VERSION_4)
    elseif(${OpenCV_VERSION_MAJOR} LESS 4)
        add_definitions(-DOPENCV_VERSION_3)
    endif()
else()
    message(FATAL_ERROR "OpenCV Library Not Found")
endif()

include_directories(
    include
    ${PROJECT_SOURCE_DIR}/msg
    ${PROJECT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

catkin_python_setup()

# Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/Interface.cfg
)

## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg 
   FILES 
   Pulses.msg
   Rotations.msg
   RotationsAtRpm.msg
   PulsesAtRpm.msg
   DoserBotJobComplete.msg 
   DoserBotJobCancelled.msg
   DoserBotAcknowledge.msg 
   MotorStatus.msg 
   LoadCellStatus.msg
   
   MultiMotorRotationsJob.msg
   SequentialMultiMotorRotationsJob.msg
   
   MultiMotorPulsesJob.msg
   SequentialMultiMotorPulsesJob.msg
   
   MultiMotorRotationsAtRpmJob.msg
   SequentialMultiMotorRotationsAtRpmJob.msg
   
   MultiMotorPulsesAtRpmJob.msg
   SequentialMultiMotorPulsesAtRpmJob.msg
   MessageProgression.msg
)

## Generate services in the 'srv' folder
add_service_files(
    DIRECTORY srv
    FILES 
    EnableMotorService.srv 
    DisableMotorService.srv 
    ResetMotorService.srv 
    SleepMotorService.srv 
    WakeMotorService.srv 
    PauseJobService.srv 
    ResumeJobService.srv 
    CancelJobService.srv  
    MotorPulsesService.srv 
    MotorRotationsService.srv 
    MotorPulsesSpecificRpmService.srv 
    MotorRotationsSpecificRpmService.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
)

# check arch and os
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ARCH arm64)
endif()

catkin_package(
   CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs message_runtime geometry_msgs image_transport cv_bridge
)

add_executable(dynamic_node src/dynamic_node.cpp)
add_dependencies(dynamic_node ${PROJECT_NAME}_gencfg)
target_link_libraries(dynamic_node ${catkin_LIBRARIES})

add_executable(selections_node src/selections_node.cpp)
add_dependencies(selections_node ${PROJECT_NAME}_gencfg)
target_link_libraries(selections_node ${catkin_LIBRARIES})
