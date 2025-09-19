cmake_minimum_required(VERSION 3.0.2)
project(joint_controller)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS)
add_definitions(-DBOOST_MPL_LIMIT_LIST_SIZE=30)
# 1. 找到依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include(cmake/pinocchio_config.cmake)
# 2. 声明catkin包配置
catkin_package(
  INCLUDE_DIRS
    include
    ${pinocchio_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    pinocchio
  CFG_EXTRAS
    pinocchio_config.cmake
)

# 4. 包含头文件目录
include_directories(
  include
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# 6. 编译可执行文件
add_executable(${PROJECT_NAME}
  src/joint_controller.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})


# 9. 安装配置
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
    