cmake_minimum_required(VERSION "3.19.2")

project(ant VERSION 1.0.0 DESCRIPTION "Automatic nerf turret")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(OpenCV_DIR $ENV{OpenCV_DIR})
find_package(OpenCV REQUIRED)

add_subdirectory(libraries/WiringPi/WiringPi)

add_executable(ant
	src/main.cpp
)

target_include_directories(ant
	PUBLIC libraries/WiringPi/WiringPi
	PUBLIC ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(ant libwiringPi ${OpenCV_LIBS})

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/opencv DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
