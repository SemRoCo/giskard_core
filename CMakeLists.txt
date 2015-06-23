cmake_minimum_required(VERSION 2.8.3)
project(giskard)

find_package(catkin REQUIRED COMPONENTS expressiongraph)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Finding system dependencies which come without cmake
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIRS yaml-cpp/yaml.h PATH_SUFFIXES include)
find_library(YAML_CPP_LIBRARIES NAMES yaml-cpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS expressiongraph
  DEPENDS yaml_cpp
)

##############
## Building ##
##############

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

FILE(GLOB SRCS src/${PROJECT_NAME}/[^.]*.cpp)

add_library(${PROJECT_NAME} ${SRCS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

#############
## Testing ##
#############

FILE(GLOB TEST_SRCS test/[^.]*.cpp test/giskard/[^.]*.cpp)
message(${TEST_SRCS})
catkin_add_gtest(${PROJECT_NAME}-test ${TEST_SRCS})
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${PROJECT_NAME})
endif()
