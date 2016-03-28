cmake_minimum_required(VERSION 2.8.3)
project(hotwheels_description)

find_package(catkin REQUIRED COMPONENTS
  xacro
)

catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

