cmake_minimum_required(VERSION 2.8.3)
project(giskard)

find_package(catkin REQUIRED COMPONENTS expressiongraph qpoases kdl_parser)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Finding system dependencies which come without cmake
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIRS yaml-cpp/yaml.h PATH_SUFFIXES include)
find_library(YAML_CPP_LIBRARIES NAMES yaml-cpp)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS expressiongraph qpoases
  DEPENDS yaml_cpp
)

##############
## Building ##
##############

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS})

#############
## Testing ##
#############

set(TEST_SRCS
  test/main.cpp
  test/${PROJECT_NAME}/double_expression_generation.cpp
  test/${PROJECT_NAME}/expression_arrays.cpp
  test/${PROJECT_NAME}/expression_scope.cpp
  test/${PROJECT_NAME}/qp_controller.cpp
  test/${PROJECT_NAME}/qp_problem_builder.cpp
  test/${PROJECT_NAME}/vector_expression_generation.cpp
  test/${PROJECT_NAME}/yaml_parser.cpp)

catkin_add_gtest(${PROJECT_NAME}-test ${TEST_SRCS}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test_data)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
endif()
