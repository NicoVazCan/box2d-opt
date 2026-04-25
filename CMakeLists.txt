cmake_minimum_required(VERSION 3.8)

project(box2d-lite LANGUAGES CXX)

option(BOX2D_BUILD_SAMPLES "Build the box2d-lite sample program" ON)
option(RPI "Build sample for Raspberry Pi" OFF)
option(HEADLESS "Build sample headless" OFF)

if (RPI)
	option(GLFW_BUILD_X11 "X11" 0)
	option(GLFW_BUILD_WAYLAND "WAYLAND" 0)
	option(GLFW_BUILD_KMSDRM "RPI KMSDRM" 1)
endif()

add_subdirectory(src)

add_subdirectory(extern/bvh)

if (BOX2D_BUILD_SAMPLES)
	if (NOT HEADLESS)
		message(STATUS "Not headless")
		add_subdirectory(extern/glad)
		add_subdirectory(extern/glfw-plus-kmsdrm)
		add_subdirectory(extern/imgui)
	endif()
	add_subdirectory(samples)

	# default startup project for Visual Studio
	if (MSVC)
		set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT samples)
	endif()

	target_compile_options(box2d-lite PUBLIC
	    $<$<CONFIG:Profiling>:-O2 -g -fno-omit-frame-pointer>
	)
endif()
