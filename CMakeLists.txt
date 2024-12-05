cmake_minimum_required(VERSION "3.19.2")

project(ant VERSION 1.0.0 DESCRIPTION "Automatic nerf turret")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(OPENCV_GENERATE_PKGCONFIG OFF CACHE BOOL "" FORCE)
set(OpenCV_STATIC ON)
#find_package(OpenCV REQUIRED)

add_subdirectory(libraries/WiringPi/WiringPi)
add_subdirectory(libraries/opencv)

add_executable(ant
	"src/main.cpp"
)

target_include_directories(ant
	PUBLIC libraries/WiringPi/WiringPi
	PUBLIC libraries/opencv/include
	PUBLIC libraries/opencv/modules/objdetect/include
	PUBLIC libraries/opencv/modules/imgproc/include
	PUBLIC libraries/opencv/modules/videoio/include
	PUBLIC libraries/opencv/modules/highgui/include
)

target_link_libraries(ant libwiringPi opencv)
