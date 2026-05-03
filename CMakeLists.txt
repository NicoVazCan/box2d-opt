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

add_compile_options(
    $<$<CONFIG:Profiling>:-DNDEBUG>
    $<$<CONFIG:Profiling>:-O2>
    $<$<CONFIG:Profiling>:-g>
    $<$<CONFIG:Profiling>:-fno-omit-frame-pointer>
)

add_subdirectory(src)

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
endif()
