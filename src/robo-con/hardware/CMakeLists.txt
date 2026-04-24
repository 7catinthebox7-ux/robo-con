add_library(
  meitec-robot SHARED
  feetech_servo/feetech_servo.cc
)

target_include_directories(
    meitec-robot PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}> $<INSTALL_INTERFACE:include>
)

target_link_libraries(
    meitec-robot
    PUBLIC 
    spdlog::spdlog
)

install(TARGETS meitec-robot DESTINATION lib/)
