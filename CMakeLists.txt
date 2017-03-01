cmake_minimum_required(VERSION 2.4.6)
project(asr_direct_search_manager)
set(CMAKE_BUILD_TYPE Debug)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DOTHER_STUFF=1")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -DOTHER_STUFF=1")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  actionlib_msgs
  actionlib
  message_generation
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  eigen_conversions
  tf
  asr_rapidxml
  asr_robot_model_services
  next_best_view
  asr_lib_ism
  dynamic_reconfigure
)
find_package(cmake_modules)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)

generate_dynamic_reconfigure_options(rsc/DynamicParameters.cfg)

add_action_files(
   DIRECTORY action
   FILES directSearch.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  asr_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS actionlib_msgs roscpp rospy std_msgs geometry_msgs message_generation message_runtime
   DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)
FILE(GLOB_RECURSE HEADERS "include/*.hpp"  "include/*.h")
add_executable(${PROJECT_NAME}
    ${HEADERS}
    src/main.cpp
    src/grid_manager.cpp
    src/model/robot_state.cpp
    src/direct_search_handler.cpp
    src/recording_manager.cpp
    src/model/ptu_tuple.cpp
    src/helper/pose_helper.cpp
    src/grid_initialisation.cpp
    src/direct_search_manager.cpp
    src/helper/sort_helper.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${direct_search_manager_EXPORTED_TARGETS}
  asr_robot_model_services_generate_messages_cpp
)


#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
catkin_add_gtest(direct_search_manager_test
    ${HEADERS}
    src/grid_manager.cpp
    src/model/robot_state.cpp
    src/direct_search_handler.cpp
    src/recording_manager.cpp
    src/model/ptu_tuple.cpp
    src/helper/pose_helper.cpp
    src/grid_initialisation.cpp
    src/direct_search_manager.cpp
    src/helper/sort_helper.cpp

    test/src/utest.cpp
    test/src/pose_helper_test.cpp
    test/src/direct_search_handler_test.cpp
    test/src/direct_search_manager_test.cpp
    test/src/filter_test.cpp
)
add_dependencies(direct_search_manager_test
    ${direct_search_manager_EXPORTED_TARGETS}
    asr_robot_model_services_generate_messages_cpp
)
target_link_libraries(direct_search_manager_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
