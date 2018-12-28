cmake_minimum_required(VERSION 2.8.3)
project(hotwheels_hardware)

find_package(catkin REQUIRED COMPONENTS
  controller_manager diagnostic_updater 
  hardware_interface roscpp rosserial_server hotwheels_msgs)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package()

set(CXX_FLAGS ${CXX_FLAGS} -std=c++11)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(hotwheels_node src/hotwheels_base src/hotwheels_hardware)
target_link_libraries(hotwheels_node ${catkin_LIBRARIES})
add_dependencies(hotwheels_node 
  rosserial_msgs_gencpp
  hotwheels_msgs_gencpp
  hotwheels_msgs_generate_messages_cpp
  ${PROJECT_NAME}_gencfg)


