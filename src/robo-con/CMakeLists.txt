cmake_minimum_required(VERSION 3.5)
project(meitec_ros2_robot)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O1 -Wall")
set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-Wall -Wextra -Wpedantic -Wuninitialized
                    -fdiagnostics-color=always -Werror)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(gflags REQUIRED)
find_package(spdlog CONFIG REQUIRED)

add_subdirectory(ros2)
add_subdirectory(hardware)

# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
