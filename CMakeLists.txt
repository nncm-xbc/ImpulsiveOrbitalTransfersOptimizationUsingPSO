cmake_minimum_required(VERSION 3.28)
project(PSO)

set(CMAKE_CXX_STANDARD 20)
option(PROF "Enable profiling" OFF)
set(PROF_DIR "${CMAKE_SOURCE_DIR}/ressources/")

# ------------------------------------------------------------------------------------------------------
# Find OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    message("-- OpenMP was found")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 ${OpenMP_CXX_FLAGS}")
    add_definitions(-DUSE_OPENMP)
else()
    message("-- OpenMP not found. Building without OpenMP support.")
    set(CMAKE_CXX_FLAGS "-O3")
endif()

# ------------------------------------------------------------------------------------------------------
# Find OpenGL
find_package(OpenGL REQUIRED)
if(OpenMP_CXX_FOUND)
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
# Set debug flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -O0 -g")

# Source files
set(SOURCES
  src/Swarm.cpp
  src/PSO.cpp
  src/OrbitProblem.cpp
  src/Logger.cpp
  rendering/src/Animation.cpp
  rendering/src/Camera.cpp
  rendering/src/ImpulseModel.cpp
  rendering/src/OrbitModel.cpp
  rendering/src/Shader.cpp
  rendering/src/TransferModel.cpp
)

# Include directories
include_directories(include)
include_directories(rendering/include)
include_directories(glad/include)
add_library(PSO_lib ${SOURCES})

# Main executable
add_executable(main src/main.cpp)

# Secondary rendering executable
add_executable(rendering 
    rendering/src/main.cpp
    ${GLAD_SRC}  # Add the GLAD implementation
)

target_link_libraries(main PRIVATE PSO_lib)
target_link_libraries(rendering PRIVATE PSO_lib)

# Link OpenMP
target_link_libraries(main PRIVATE $<$<BOOL:${OpenMP_CXX_FOUND}>:OpenMP::OpenMP_CXX>)

# Link OpenGL and GLFW
include_directories(${GLM_INCLUDE_DIR})
include_directories(${GLFW_INCLUDE_DIR})

target_include_directories(rendering PRIVATE 
    ${CMAKE_SOURCE_DIR}/glad/include
)

target_link_libraries(rendering PRIVATE
  ${OPENGL_LIBRARIES} 
  glfw
)

# ------------------------------------------------------------------------------------------------------
# Testing
enable_testing()
find_package(Catch2 REQUIRED)
if(Catch2_FOUND)
  message("-- Catch2 was found")

    # Add test executable
  set(TEST_SOURCES
    # tests/test_particle.cpp
    # tests/test_swarm.cpp
    tests/test_PSO.cpp
    tests/test_OrbitProblem.cpp
    tests/test_OrbitTransfer.cpp
  )

  add_executable(run_tests ${TEST_SOURCES})

  target_link_libraries(run_tests PRIVATE PSO_lib)
  target_link_libraries(run_tests PRIVATE Catch2::Catch2WithMain)

  # Add tests
  include(Catch)
  catch_discover_tests(run_tests)

else()
  message(FATAL_ERROR "Catch2 was not found. Please install it.")
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
        COMMAND ./main
        COMMAND gprof main gmon.out > "${PROF_DIR}/ProfAnalysis.txt"
        DEPENDS main
    )
endif()
