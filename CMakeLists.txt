cmake_minimum_required(VERSION 3.10)
project(PoseLib VERSION 2.0.5)

# Compilation flags function
function(set_compile_flags build_target)
    if(MSVC)
        target_compile_options(${build_target} PRIVATE /bigobj)
    else()
        target_compile_options(${build_target} PRIVATE
            -O3 -Wall -Werror -fPIC -Wno-sign-compare -Wno-unused-but-set-variable -Wfatal-errors)
        if(MARCH_NATIVE)
            target_compile_options(${build_target} PRIVATE -march=native)
        endif()
        if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            target_compile_options(${build_target} PRIVATE
                -Wno-maybe-uninitialized)
        endif()
    endif()
    target_compile_features(${build_target} PUBLIC cxx_std_17)
endfunction()

# Set variables
set(LIBRARY_NAME   ${PROJECT_NAME})
set(LIBRARY_FOLDER ${PROJECT_NAME})
include(${PROJECT_SOURCE_DIR}/cmake/SetEnv.cmake)

# Eigen
find_package(Eigen3 REQUIRED)

# Library sources
add_subdirectory(${LIBRARY_FOLDER})

# Sturm sequence solver max recursion depth
# You should lower this value only if you need to run PoseLib in environments with low stack limit (<1MB).
# Changing this may affect solver stability.
set(MAX_STURM_RECURSION_DEPTH_LIMIT 300 CACHE STRING
    "Maximum recursion depth for Sturm sequence solver of univariate polynomials.")

target_compile_definitions(PoseLib PUBLIC
    MAX_STURM_RECURSION_DEPTH_LIMIT=${MAX_STURM_RECURSION_DEPTH_LIMIT})

# C API wrapper option
option(WITH_C_API "Build C API wrapper for P/Invoke" ON)
if(WITH_C_API)
    message(STATUS "Building with C API wrapper enabled")
    target_sources(PoseLib PRIVATE
        ${PROJECT_SOURCE_DIR}/PoseLib/poselib_c_api.cc
    )
    if(WIN32)
        target_compile_definitions(PoseLib PRIVATE POSELIB_BUILDING_DLL)
    endif()
    # Ensure shared library is built when C API is enabled
    set_target_properties(PoseLib PROPERTIES
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN ON
    )
endif()

# Benchmark
option(WITH_BENCHMARK "Build benchmark example." OFF)
if(WITH_BENCHMARK)
    add_subdirectory(benchmark)
endif()

# Compilation options
option(MARCH_NATIVE "Enable CPU specific optimizations" OFF)
set_compile_flags(${LIBRARY_NAME})

# python bindings
option(PYTHON_PACKAGE "Build python package." OFF)
if(PYTHON_PACKAGE)
    add_subdirectory(pybind)
endif()
