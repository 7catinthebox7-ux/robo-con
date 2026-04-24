add_executable(controller_node controller_node.cc)

ament_target_dependencies(controller_node rclcpp std_srvs)

target_link_libraries(controller_node ${cpp_typesupport_target} gflags spdlog::spdlog meitec-robot)

install(TARGETS controller_node DESTINATION lib/${PROJECT_NAME})
