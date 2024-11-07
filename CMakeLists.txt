cmake_minimum_required(VERSION "3.19.2")

project(ant VERSION 1.0.0 DESCRIPTION "Automatic nerf turret")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(libraries/WiringPi/WiringPi)

add_executable(ant
	"src/main.cpp"
)

target_include_directories(ant
	PUBLIC headers
	PUBLIC libraries/WiringPi/WiringPi
)
