cmake_minimum_required(VERSION 3.8)
project(transportation_hub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(EXEC_SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set(EXEC_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

file(GLOB_RECURSE EXEC_SRC ${EXEC_SRC_DIR}/*.cpp)
file(GLOB_RECURSE EXEC_INCLUDE ${EXEC_INCLUDE_DIR}/*.h)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${EXEC_INCLUDE_DIR}/${PROJECT_NAME})

ament_auto_add_executable(${PROJECT_NAME} ${EXEC_SRC})

ament_target_dependencies(${PROJECT_NAME} rclcpp judger_interfaces)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

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

ament_auto_package()