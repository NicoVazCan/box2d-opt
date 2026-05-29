cmake_minimum_required(VERSION 3.0)
project(ice)

set(ICE_SOURCE_FILES 
    src/IceBoxPruning.cpp
    src/IceContainer.cpp
    src/IceRevisitedRadix.cpp)

set(ICE_HEADER_FILES
    include/IceBoxPruning.h
    include/IceContainer.h
    include/IceFPU.h
    include/IceMemoryMacros.h
    include/IcePoint.h
    include/IceRevisitedRadix.h
    include/IceTypes.h
    include/StdAfx.h)

add_library(ice STATIC ${ICE_SOURCE_FILES} ${ICE_HEADER_FILES})
target_include_directories(ice PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_compile_options(ice PRIVATE
    $<$<CONFIG:Profiling>:-O2 -g -fno-omit-frame-pointer>
)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    target_compile_options(ice PRIVATE
        $<$<CONFIG:Profiling>:-ftree-vectorize>
    )
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
    target_compile_options(ice PRIVATE
        $<$<CONFIG:Profiling>:-vec>
    )
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "IntelLLVM")
    target_compile_options(ice PRIVATE
        $<$<CONFIG:Profiling>:-fvectorize -fslp-vectorize>
    )
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(ice PRIVATE
        $<$<CONFIG:Profiling>:-fvectorize -fslp-vectorize>
    )
endif()

option(ICE_USE_BRANCHLESS_INTERSECTS "ICE_USE_BRANCHLESS_INTERSECTS" 1)
option(ICE_USE_FLOAT_ENCODE "ICE_USE_FLOAT_ENCODE" 1)
option(ICE_USE_LOOP_UNROLL_2 "ICE_USE_LOOP_UNROLL_2" 0)
option(ICE_USE_LOOP_UNROLL_3 "ICE_USE_LOOP_UNROLL_3" 0)
option(ICE_USE_LOOP_UNROLL_4 "ICE_USE_LOOP_UNROLL_4" 1)

if (ICE_USE_BRANCHLESS_INTERSECTS)
    target_compile_definitions(ice PRIVATE ICE_USE_BRANCHLESS_INTERSECTS)
endif()
if (ICE_USE_FLOAT_ENCODE)
    target_compile_definitions(ice PRIVATE ICE_USE_FLOAT_ENCODE)
endif()
if (ICE_USE_LOOP_UNROLL_2)
    target_compile_definitions(ice PRIVATE ICE_USE_LOOP_UNROLL_2)
endif()
if (ICE_USE_LOOP_UNROLL_3)
    target_compile_definitions(ice PRIVATE ICE_USE_LOOP_UNROLL_3)
endif()
if (ICE_USE_LOOP_UNROLL_4)
    target_compile_definitions(ice PRIVATE ICE_USE_LOOP_UNROLL_4)
endif()
