cmake_minimum_required(VERSION 3.8)
project(light_lift_arm_6dof)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()


# 查找 libserial 库
find_library(
    LIBSERIAL_LIBRARY 
    NAMES serial 
    PATHS /usr/lib /usr/lib/x86_64-linux-gnu 
    NO_DEFAULT_PATH
    REQUIRED)
find_path(LIBSERIAL_INCLUDE_DIR NAMES libserial/SerialPort.h PATHS /usr/include REQUIRED)
# 打印调试信息，确保路径被正确设置
message(STATUS "ts LIBSERIAL_LIBRARY: ${LIBSERIAL_LIBRARY}")
message(STATUS "ts LIBSERIAL_INCLUDE_DIR: ${LIBSERIAL_INCLUDE_DIR}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

#eigen3
find_package(Eigen3 REQUIRED)
 
####################################################

# orocos_kdl
# CMake 需要 找到 orocos_kdlConfig.cmake 才能成功链接 orocos_kdl。
# 但是 CMake 默认不会自动搜索当前目录（但是你可以先编译一下试试） ~/ros2_ws/install/orocos_kdl/share/orocos_kdl/cmake，
# 因为他会自动搜索系统的目录，所以如果你把kdl安装在系统目录下，那么就不需要设置 orocos_kdl_DIR。
# 所以你需要手动设置 orocos_kdl_DIR 或者 CMAKE_PREFIX_PATH。
#提前在终端运行：
# export orocos_kdl_DIR=orocos_kdlConfig.cmake所在目录（比如/home/qing/ws/6dof/ros2_ws2/install/orocos_kdl/share/orocos_kdl/cmake）
# colcon build

find_package(orocos_kdl REQUIRED)

find_package(kdl_parser REQUIRED)

find_package(urdf REQUIRED)

###################################


find_package(sensor_msgs REQUIRED)

find_package(tf2_ros REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(trajectory_msgs REQUIRED)

find_package(std_msgs REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})


add_executable(light_lift_arm_6dof_node 
src/light_lift_arm_6dof_node.cpp
src/communication/serial_port.cpp
src/drivers/motor_control.cpp
src/mathematical_model/light_lift_arm_6dof.cpp
src/controllers/state_machine.cpp
)

#为目标 light_lift_3dof 指定包含目录，这里指定了两个目录，一个是当前目录下的 include 目录，另一个是安装目录下的 include 目录
#${CMAKE_CURRENT_SOURCE_DIR} 是当前 CMakeLists.txt 文件所在的目录
target_include_directories(light_lift_arm_6dof_node PUBLIC
  #BUILD_INTERFACE 表示这是在构建阶段使用的目录。
  #这个目录就是${CMAKE_CURRENT_SOURCE_DIR}/include
  #在引用头文件时，就需要使用 #include "light_lift_arm/xxx.h" 的方式
  #如果${CMAKE_CURRENT_SOURCE_DIR}/include/light_lift_arm，则只需要 #include "xxx.h" 的方式
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  
  #INSTALL_INTERFACE 表示这是在安装阶段使用的目录
  #这个目录是include
  $<INSTALL_INTERFACE:include>
  ${LIBSERIAL_INCLUDE_DIR}

  )

ament_target_dependencies(
  light_lift_arm_6dof_node rclcpp 
  orocos_kdl 
  kdl_parser 
  urdf 
  sensor_msgs 
  tf2_ros
  std_msgs
  trajectory_msgs
  geometry_msgs
  )

target_link_libraries(light_lift_arm_6dof_node ${LIBSERIAL_LIBRARY})

install(TARGETS
  light_lift_arm_6dof_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
