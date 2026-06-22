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
	target_compile_options(box2d-lite PRIVATE
		$<$<CONFIG:Profiling>:-ftree-vectorize>
	)
	set_source_files_properties(${VEC_INFO_FILE} PROPERTIES
		COMPILE_OPTIONS $<$<CONFIG:Profiling>:-fopt-info-vec-all>
	)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
	target_compile_options(box2d-lite PRIVATE
		$<$<CONFIG:Profiling>:-vec>
	)
	set_source_files_properties(${VEC_INFO_FILE} PROPERTIES
		COMPILE_OPTIONS $<$<CONFIG:Profiling>:-qopt-report-phase=vec>
	)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "IntelLLVM")
	target_compile_options(box2d-lite PRIVATE
		$<$<CONFIG:Profiling>:-fvectorize -fslp-vectorize>
	)
	set_source_files_properties(${VEC_INFO_FILE} PROPERTIES
		COMPILE_OPTIONS $<$<CONFIG:Profiling>:-qopt-report-phase=vec>
	)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
	target_compile_options(box2d-lite PRIVATE
		$<$<CONFIG:Profiling>:-fvectorize -fslp-vectorize>
	)
	set_source_files_properties(${VEC_INFO_FILE} PROPERTIES
		COMPILE_OPTIONS $<$<CONFIG:Profiling>:-Rpass-analysis=loop-vectorize>
	)
endif()

target_include_directories(box2d-lite PUBLIC ../include)

if (BOX2D_USE_OMP)
	find_package(OpenMP REQUIRED)
	target_compile_definitions(box2d-lite PRIVATE BOX2D_USE_OMP)
	target_link_libraries(box2d-lite PUBLIC OpenMP::OpenMP_CXX)
endif()

if (BOX2D_USE_BROADPHASE_BVH)
	target_link_libraries(box2d-lite PUBLIC bvh)
elseif(BOX2D_USE_BROADPHASE_SAP)
	target_link_libraries(box2d-lite PUBLIC ice)
endif()
