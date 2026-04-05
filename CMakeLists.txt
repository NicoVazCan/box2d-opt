cmake_minimum_required(VERSION 3.8)

project(box2d-lite LANGUAGES CXX)

add_subdirectory(src)

add_subdirectory(extern/bvh)

option(BOX2D_BUILD_SAMPLES "Build the box2d-lite sample program" ON)

if (RPI)
	option(GLFW_BUILD_X11 "X11" 0)
	option(GLFW_BUILD_WAYLAND "WAYLAND" 0)
	option(GLFW_BUILD_KMSDRM "RPI KMSDRM" 1)
endif()

target_compile_options(box2d-lite PRIVATE
    $<$<CONFIG:Profiling>:-O2 -g -fno-omit-frame-pointer>
)

if (BOX2D_BUILD_SAMPLES)

	add_subdirectory(extern/glad)
	add_subdirectory(extern/glfw-plus-kmsdrm)
	add_subdirectory(extern/imgui)
	add_subdirectory(samples)

	# default startup project for Visual Studio
	if (MSVC)
		set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT samples)
	endif()

endif()
