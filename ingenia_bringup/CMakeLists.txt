cmake_minimum_required(VERSION 3.8)
project(ingenia_bringup)

find_package(ament_cmake REQUIRED)
find_package(lely_core_libraries REQUIRED)

# Generate master DCF from config/j0/bus.yml + EDS at build time.
cogen_dcf(j0)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
