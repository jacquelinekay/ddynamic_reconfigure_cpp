cmake_minimum_required(VERSION 2.8)
project(ddynamic_reconfigure_cpp)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure roscpp)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

#add_message_files(
#  DIRECTORY msg
#  FILES 
#  BoolParameter.msg      Config.msg           Group.msg       IntParameter.msg      SensorLevels.msg
#  ConfigDescription.msg  DoubleParameter.msg  GroupState.msg  ParamDescription.msg  StrParameter.msg)
#
#add_service_files(
#  DIRECTORY srv
#  FILES Reconfigure.srv)
#
#generate_messages(DEPENDENCIES std_msgs)
#

if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  set(dynamic_reconfigure_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
#  include(${dynamic_reconfigure_SOURCE_DIR}/cmake/dynamic_reconfigure-macros.cmake)
#  generate_dynamic_reconfigure_options(cfg/Test.cfg)
#  add_subdirectory(test)
# compile a simple example
  add_executable(api_test src/api_sketch.cpp)
  target_link_libraries(api_test ${catkin_LIBRARIES})
  target_compile_options(api_test PUBLIC "-std=c++11")
endif()

catkin_package(INCLUDE_DIRS include
               CATKIN_DEPENDS dynamic_reconfigure
)


install(DIRECTORY include/ddynamic_reconfigure_cpp/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp")
