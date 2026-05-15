cmake_minimum_required(VERSION 3.8)
project(ts_topic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# 查找 libinput 库
pkg_check_modules(LIBINPUT REQUIRED libinput)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(ts_topic_node
 src/ts_topic.cpp
 src/mousekey.cpp
 )
ament_target_dependencies(ts_topic_node rclcpp sensor_msgs)

#为目标 light_lift_3dof 指定包含目录，这里指定了两个目录，一个是当前目录下的 include 目录，另一个是安装目录下的 include 目录
#${CMAKE_CURRENT_SOURCE_DIR} 是当前 CMakeLists.txt 文件所在的目录
target_include_directories(ts_topic_node PUBLIC
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

target_include_directories(ts_topic_node PUBLIC ${LIBINPUT_INCLUDE_DIRS})
target_link_libraries(ts_topic_node ${LIBINPUT_LIBRARIES})


install(TARGETS
ts_topic_node
  DESTINATION lib/${PROJECT_NAME})



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
