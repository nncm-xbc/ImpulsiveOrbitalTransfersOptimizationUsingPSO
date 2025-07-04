cmake_minimum_required(VERSION 3.24)
project(PSO)

# ------------------------------------------------------------------------------------------------------
# Build type configuration
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release" FORCE)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Set build type specific flags
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wextra -pedantic -O3 -march=native")
    message(STATUS "Debug mode: Optimizations disabled, debug symbols enabled")
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -funroll-loops -march=native -Wall")
    message(STATUS "Release mode: Full optimizations enabled")
endif()

# ------------------------------------------------------------------------------------------------------

if(APPLE)
    # Use Homebrew's clang instead of Apple's
    execute_process(
        COMMAND brew --prefix llvm
        OUTPUT_VARIABLE LLVM_PREFIX
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    if(LLVM_PREFIX)
        set(CMAKE_C_COMPILER "${LLVM_PREFIX}/bin/clang")
        set(CMAKE_CXX_COMPILER "${LLVM_PREFIX}/bin/clang++")

        # Add include paths from Homebrew's llvm
        include_directories(SYSTEM "${LLVM_PREFIX}/include/c++/v1")
        include_directories(SYSTEM "${LLVM_PREFIX}/include")

        # Add library path
        link_directories("${LLVM_PREFIX}/lib")
    endif()
endif()

set(CMAKE_CXX_STANDARD 20)
option(PROF "Enable profiling" OFF)
set(PROF_DIR "${CMAKE_SOURCE_DIR}/ressources")

# ------------------------------------------------------------------------------------------------------
# Options
option(PROF "Enable profiling" OFF)
option(USE_OPENMP "Enable OpenMP support" ON)
set(PROF_DIR "${CMAKE_SOURCE_DIR}/ressources")

# ------------------------------------------------------------------------------------------------------
# Find OpenMP
if (USE_OPENMP)
    find_package(OpenMP)
    if(OpenMP_CXX_FOUND)
        message("-- OpenMP was found")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        add_definitions(-DUSE_OPENMP)
    else()
        message("-- OpenMP not found. Building without OpenMP support.")
    endif()
endif()

# ------------------------------------------------------------------------------------------------------
# Find OpenGL
find_package(OpenGL REQUIRED)
if(OpenGL_FOUND)
    message("-- OpenGL was found")
    find_path(GLM_INCLUDE_DIR glm/glm.hpp)
    find_path(GLFW_INCLUDE_DIR GLFW/glfw3.h)
    find_library(GLFW_LIBRARY NAMES glfw glfw3)
    # Set path to GLAD
    set(GLAD_SRC ${CMAKE_SOURCE_DIR}/glad/src/glad.c)
else()
    message("-- OpenGL not found. Please install OpenGL")
endif()

# ------------------------------------------------------------------------------------------------------
# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/glad/include)
include_directories(${GLM_INCLUDE_DIR})
include_directories(${GLFW_INCLUDE_DIR})

# ------------------------------------------------------------------------------------------------------
# Source files organized by module

# Core physics and math sources
set(CORE_SOURCES
    src/core/OrbitProblem.cpp
    src/core/LambertSolver.cpp
    src/core/OrbitMechanics.cpp
)

# Optimization sources
set(OPTIMIZATION_SOURCES
    src/optimization/PSO.cpp
    src/optimization/Swarm.cpp
    src/optimization/Logger.cpp
)

# Visualization sources
set(VISUALIZATION_SOURCES
    src/visualization/OrbitModel.cpp
    src/visualization/TransferModel.cpp
    src/visualization/Camera.cpp
    src/visualization/Animation.cpp
    src/visualization/Shader.cpp
    ${GLAD_SRC}  # GLAD is part of visualization
)

# All library sources
set(LIB_SOURCES
    ${CORE_SOURCES}
    ${OPTIMIZATION_SOURCES}
    ${VISUALIZATION_SOURCES}
)

# ------------------------------------------------------------------------------------------------------
# Create libraries

# Core library (physics and optimization - no OpenGL dependencies)
add_library(PSO_core STATIC
    ${CORE_SOURCES}
    ${OPTIMIZATION_SOURCES}
)

# Visualization library (OpenGL dependent)
add_library(PSO_visualization STATIC
    ${VISUALIZATION_SOURCES}
)

# Link OpenGL to visualization library
target_link_libraries(PSO_visualization PUBLIC ${OPENGL_LIBRARIES})
if(APPLE)
    execute_process(
        COMMAND brew --prefix
        OUTPUT_VARIABLE HOMEBREW_PREFIX
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    find_library(GLFW_LIBRARY glfw NAMES glfw3 PATHS "${HOMEBREW_PREFIX}/lib" REQUIRED)
    target_link_libraries(PSO_visualization PUBLIC ${GLFW_LIBRARY})
else()
    target_link_libraries(PSO_visualization PUBLIC glfw)
endif()

# ------------------------------------------------------------------------------------------------------
# Main executables

# PSO computation executable (no visualization)
add_executable(pso_compute src/application/pso_main.cpp)
target_link_libraries(pso_compute PRIVATE PSO_core)
target_link_libraries(pso_compute PRIVATE $<$<BOOL:${OpenMP_CXX_FOUND}>:OpenMP::OpenMP_CXX>)

# Visualization executable (includes everything)
add_executable(pso_visualize src/application/render_main.cpp)
target_link_libraries(pso_visualize PRIVATE PSO_core PSO_visualization)
target_link_libraries(pso_visualize PRIVATE $<$<BOOL:${OpenMP_CXX_FOUND}>:OpenMP::OpenMP_CXX>)
target_compile_definitions(pso_visualize PRIVATE WITH_VISUALIZATION)

# ------------------------------------------------------------------------------------------------------
# Testing
enable_testing()
find_package(Catch2)
if(Catch2_FOUND)
    message("-- Catch2 was found")

    set(TEST_SOURCES
        tests/test_OrbitMechanics.cpp
        tests/test_OrbitProblem.cpp
        tests/test_PSO.cpp
        tests/test_LambertSolver.cpp
        tests/test_swarm.cpp
    )

    add_executable(run_tests ${TEST_SOURCES})
    target_link_libraries(run_tests PRIVATE PSO_core)
    target_link_libraries(run_tests PRIVATE Catch2::Catch2WithMain)

    add_executable(test_orbit_mechanics tests/test_OrbitMechanics.cpp)
    target_link_libraries(test_orbit_mechanics PRIVATE PSO_core Catch2::Catch2WithMain)

    add_executable(test_orbit_problem tests/test_OrbitProblem.cpp)
    target_link_libraries(test_orbit_problem PRIVATE PSO_core Catch2::Catch2WithMain)

    add_executable(test_pso tests/test_PSO.cpp)
    target_link_libraries(test_pso PRIVATE PSO_core Catch2::Catch2WithMain)

    include(Catch)
    catch_discover_tests(run_tests)
    catch_discover_tests(test_orbit_mechanics)
    catch_discover_tests(test_orbit_problem)
    catch_discover_tests(test_pso)

else()
    message(WARNING "Catch2 was not found. Tests will not be built.")
endif()

# ------------------------------------------------------------------------------------------------------
# Profiling
if(PROF)
    message(STATUS "Profiling enabled")

    # Append -pg to CXX flags and linker
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")

    # Custom target for profiling
    add_custom_target(profile
        COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR}
        COMMAND ./pso_compute
        COMMAND gprof pso_compute gmon.out > "${PROF_DIR}/ProfAnalysis.txt"
        DEPENDS pso_compute
    )
endif()

# ------------------------------------------------------------------------------------------------------
# Installation rules (optional)
install(TARGETS pso_compute pso_visualize
    RUNTIME DESTINATION bin
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
    DESTINATION include
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/shaders/
    DESTINATION share/pso/shaders
)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/resources/
    DESTINATION share/pso/resources
)
