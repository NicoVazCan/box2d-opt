project(samples LANGUAGES CXX)

set (SAMPLE_SOURCE_FILES main.cpp)

if (NOT HEADLESS)
	set (SAMPLE_SOURCE_FILES
		${SAMPLE_SOURCE_FILES}
		imgui_impl_glfw.cpp
		imgui_impl_opengl2.cpp)
endif()

if (NOT HEADLESS)
	set (SAMPLE_HEADER_FILES
		imgui_impl_glfw.h
		imgui_impl_opengl2.h)
endif()

if (HEADLESS)
	add_compile_definitions(HEADLESS)
endif()

add_executable(samples ${SAMPLE_SOURCE_FILES} ${SAMPLE_HEADER_FILES})
target_include_directories(samples PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(samples PUBLIC box2d-lite)
if (NOT HEADLESS)
	target_link_libraries(samples PUBLIC glfw imgui glad m ${OPENGL_LIBRARIES})
endif()
