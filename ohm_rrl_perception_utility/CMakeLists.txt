cmake_minimum_required(VERSION 3.5)
project(ohm_rrl_perception_utility)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Qt5 COMPONENTS Gui REQUIRED)

###########
## Build ##
###########

include_directories(
  
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ohm_rrl_perception_utility_node.cpp)
add_executable(hazmat_viz_node src/ohm_convert_findObject2D_toRect.cpp)

ament_target_dependencies(hazmat_viz_node
    "rclcpp"
    "image_transport"
    "std_msgs"
    "cv_bridge"
)

## Specify libraries to link a library or executable target against
target_link_libraries(hazmat_viz_node
  Qt5::Gui
  ${OpenCV_LIBRARIES}
)

#---------------------------------------------------------------------------------------
# Install
#---------------------------------------------------------------------------------------
install(TARGETS
    hazmat_viz_node
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})

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