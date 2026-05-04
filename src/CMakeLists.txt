set(BOX2D_SOURCE_FILES
	Arbiter.cpp
	Body.cpp
	Collide.cpp
	Joint.cpp
	World.cpp)

set(BOX2D_HEADER_FILES
	../include/box2d-lite/Arbiter.h
	../include/box2d-lite/Body.h
	../include/box2d-lite/Joint.h
	../include/box2d-lite/MathUtils.h
	../include/box2d-lite/World.h)

add_library(box2d-lite STATIC ${BOX2D_SOURCE_FILES} ${BOX2D_HEADER_FILES})

list (GET BOX2D_SOURCE_FILES 2 VEC_INFO_FILE)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
	set_source_files_properties(${VEC_INFO_FILE} PROPERTIES
	    COMPILE_OPTIONS $<$<CONFIG:Profiling>:-fopt-info-vec-all>
	)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
	set_source_files_properties(${VEC_INFO_FILE} PROPERTIES
	    COMPILE_OPTIONS $<$<CONFIG:Profiling>:-qopt-report-phase=vec>
	)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "IntelLLVM")
	set_source_files_properties(${VEC_INFO_FILE} PROPERTIES
	    COMPILE_OPTIONS $<$<CONFIG:Profiling>:-qopt-report-phase=vec>
	)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
	set_source_files_properties(${VEC_INFO_FILE} PROPERTIES
	    COMPILE_OPTIONS $<$<CONFIG:Profiling>:-Rpass-analysis=loop-vectorize>
	)
endif()

target_include_directories(box2d-lite PUBLIC ../include)
