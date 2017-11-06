cmake_minimum_required(VERSION 2.8.3)
project(giskard_core)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS expressiongraph qpoases kdl_parser)

# setting additional cmake flags, used for integration with Coveralls.io
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MY_CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self -Wunused-function -Wunused-label -Wunused-variable -Wunused-but-set-parameter")#-Wunused-but-set-variable 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wreorder -Warray-bounds -Wtype-limits -Werror=return-type -Wsequence-point -Wparentheses -Wmissing-braces -Wchar-subscripts -Wswitch -Wwrite-strings -Wenum-compare -Wempty-body")# -Wlogical-op") 

## Finding system dependencies which come without cmake
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)
find_path(yaml_cpp_INCLUDE_DIRS yaml-cpp/yaml.h PATH_SUFFIXES include)
find_library(yaml_cpp_LIBRARIES NAMES yaml-cpp)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS expressiongraph qpoases kdl_parser
  DEPENDS yaml_cpp
)

##############
## Building ##
##############

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${yaml_cpp_INCLUDE_DIRS})

# add_library(${PROJECT_NAME} include/giskard/giskard.hpp)
# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES} yaml-cpp)

add_executable(extract_expression src/${PROJECT_NAME}/extract_expression.cpp)
target_link_libraries(extract_expression
  ${catkin_LIBRARIES} yaml-cpp)

#############
## Testing ##
#############

set(TEST_SRCS
  test/main.cpp
  #test/${PROJECT_NAME}/boxy_fk.cpp
  #test/${PROJECT_NAME}/double_expression_generation.cpp
  #test/${PROJECT_NAME}/expression_arrays.cpp
  #test/${PROJECT_NAME}/equality.cpp
  #test/${PROJECT_NAME}/frame_expression_generation.cpp
  #test/${PROJECT_NAME}/flying_cup.cpp
  #test/${PROJECT_NAME}/pr2_cart_cart.cpp
  #test/${PROJECT_NAME}/pr2_fk.cpp
  #test/${PROJECT_NAME}/pr2_ik.cpp
  #test/${PROJECT_NAME}/qp_controller.cpp
  #test/${PROJECT_NAME}/qp_problem_builder.cpp
  #test/${PROJECT_NAME}/rotation_control.cpp
  #test/${PROJECT_NAME}/rotation_expression_generation.cpp
  #test/${PROJECT_NAME}/robot.cpp
  #test/${PROJECT_NAME}/scope.cpp
  #test/${PROJECT_NAME}/slerp.cpp
  #test/${PROJECT_NAME}/vector_expression_generation.cpp
  test/${PROJECT_NAME}/whole_body_control_params.cpp
  #test/${PROJECT_NAME}/yaml_parser.cpp
  )

catkin_add_gtest(${PROJECT_NAME}-test ${TEST_SRCS}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test_data)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test
      ${catkin_LIBRARIES} ${yaml_cpp_LIBRARIES})
endif()
