cmake_minimum_required(VERSION 3.0)
project(bvh)

set(BVH_SOURCE_FILES src/bvh.cpp)

set(BVH_HEADER_FILES include/bvh.h)

add_library(bvh STATIC ${BVH_SOURCE_FILES} ${BVH_HEADER_FILES})
target_include_directories(bvh PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_compile_options(bvh PRIVATE
    $<$<CONFIG:Profiling>:-O2 -g -fno-omit-frame-pointer>
)

target_compile_features(bvh PUBLIC cxx_std_17)

if(DEMO)
    find_package(SDL REQUIRED)
    include_directories(${SDL_INCLUDE_DIR})

    add_executable(demo demo/main.cpp)
    target_link_libraries(demo bvh ${SDL_LIBRARY})
endif()
