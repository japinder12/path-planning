cmake_minimum_required(VERSION 3.16)
project(path_planning_sandbox CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Try SFML 3 first, then fall back to 2.x
find_package(SFML 3 QUIET COMPONENTS Graphics Window System)
if(SFML_FOUND)
  set(USE_SFML3 TRUE)
else()
  find_package(SFML 2.5 REQUIRED COMPONENTS graphics window system)
  set(USE_SFML3 FALSE)
endif()

add_executable(sandbox
  src/main.cpp
)

target_include_directories(sandbox PRIVATE src)
if(USE_SFML3)
  target_link_libraries(sandbox PRIVATE SFML::Graphics SFML::Window SFML::System)
else()
  target_link_libraries(sandbox PRIVATE sfml-graphics sfml-window sfml-system)
endif()

if(MSVC)
  target_compile_options(sandbox PRIVATE /W4)
else()
  target_compile_options(sandbox PRIVATE -Wall -Wextra -Wpedantic)
endif()
