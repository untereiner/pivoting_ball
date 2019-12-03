cmake_minimum_required(VERSION 3.7.2 FATAL_ERROR)

project(pivoting_ball
	LANGUAGES CXX C
	)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

set(CGOGN_RELEASE_BUILD_DIR "../cgogn/build" CACHE PATH "CGoGN build or install dir")
set(CGOGN_DEBUG_BUILD_DIR "../cgogn/build" CACHE PATH "CGoGN build or install dir")

#### Default build type
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
	message("cgogn in release mode")
	list(APPEND CMAKE_PREFIX_PATH "${CGOGN_RELEASE_BUILD_DIR}/lib/cmake")
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
	message("cgogn in debug mode")
	list(APPEND CMAKE_PREFIX_PATH "${CGOGN_DEBUG_BUILD_DIR}/lib/cmake")
endif()

find_package(cgogn_core REQUIRED)
find_package(cgogn_geometry REQUIRED)
find_package(cgogn_io REQUIRED)
find_package(cgogn_rendering REQUIRED)
find_package(QOGLViewer REQUIRED)
find_package(Qt5 5.5.0 COMPONENTS Widgets REQUIRED)

set(HEADER_FILES
	pivoting_ball_0.h
	pivoting_ball_1.h
	pivoting_ball_2.h
	)

set(SOURCE_FILES
	pivoting_ball_0.cpp
	pivoting_ball_1.cpp
	pivoting_ball_2.cpp
	viewer.cpp
	)

add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} cgogn::core cgogn::geometry cgogn::io cgogn::rendering)
