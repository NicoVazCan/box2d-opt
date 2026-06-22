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

if (DEMO_TUNE)
	add_compile_definitions(DEMO_TUNE)
endif()

if (PERF_CONFIG)
	add_compile_options(-g -fno-omit-frame-pointer)
endif()

add_compile_options(
    $<$<CONFIG:Profiling>:-DNDEBUG>
    $<$<CONFIG:Profiling>:-march=native>
    $<$<CONFIG:Profiling>:-O2>
)

message(STATUS "Used C++ compiler: " ${CMAKE_CXX_COMPILER_ID})

if (BOX2D_USE_BROADPHASE_BVH)
	add_compile_definitions(BOX2D_USE_ARBITER_MAP_PER_BODY)
	add_compile_definitions(BOX2D_USE_BROADPHASE_BVH)
	add_subdirectory(extern/bvh)
elseif(BOX2D_USE_BROADPHASE_SAP)
	add_compile_definitions(BOX2D_USE_ARBITER_MAP_PER_BODY)
	add_compile_definitions(BOX2D_USE_BROADPHASE_SAP)
	add_subdirectory(extern/ice)
elseif(BOX2D_USE_ARBITER_MAP_PER_BODY)
	add_compile_definitions(BOX2D_USE_ARBITER_MAP_PER_BODY)
endif()

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
