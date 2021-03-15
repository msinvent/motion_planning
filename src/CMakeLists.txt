################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(motion_planning VERSION 1.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_executable(main main.cpp)
add_subdirectory(adder)
add_subdirectory(bfs)
add_subdirectory(A_star)

add_executable(main main.cpp)
target_link_libraries(main adder)
###########################################
# Shortcuts
###########################################
# {PROJECT_NAME}
# target_link_directories(main PRIVATE Adder/)
