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
# Set debug flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -O0 -g")

# Source files
set(SOURCES
  src/Swarm.cpp
  src/PSO.cpp
  src/OrbitProblem.cpp
  src/Logger.cpp
  src/rendering/Animation.cpp
  src/rendering/Camera.cpp
  src/rendering/ImpulseVisu.cpp
  src/rendering/OrbitModel.cpp
  src/rendering/RenderEngine.cpp
  src/rendering/Shader.cpp
  src/rendering/TransferModel.cpp
)

# Include directories
include_directories(include)
add_library(PSO_lib ${SOURCES})

# Main executable
add_executable(main src/main.cpp)

# Secondary rendering executable
add_executable(rendering src/rendering/main.cpp)

target_link_libraries(main PRIVATE PSO_lib)
target_link_libraries(rendering PRIVATE PSO_lib)

# Link OpenMP if found
target_link_libraries(main PRIVATE $<$<BOOL:${OpenMP_CXX_FOUND}>:OpenMP::OpenMP_CXX>)

# Link OpenGL and GLFW
#set(OPENGL_INCLUDE_DIR "/nix/store/v9hsgy7dqyax5darjiqdn1ikmswwn6f2-orb-pso-tools/include")
#set(OPENGL_opengl_LIBRARY "/nix/store/v9hsgy7dqyax5darjiqdn1ikmswwn6f2-orb-pso-tools/libGL.so")
#set(OPENGL_glx_LIBRARY "/nix/store/v9hsgy7dqyax5darjiqdn1ikmswwn6f2-orb-pso-tools/libGLX.so")

find_library(OPENGL_opengl_LIBRARY 
    NAMES GL
    PATHS "/nix/store/v9hsgy7dqyax5darjiqdn1ikmswwn6f2-orb-pso-tools"
    PATH_SUFFIXES lib
    NO_DEFAULT_PATH
)

find_library(OPENGL_glx_LIBRARY 
    NAMES GLX
    PATHS "/nix/store/v9hsgy7dqyax5darjiqdn1ikmswwn6f2-orb-pso-tools"
    PATH_SUFFIXES lib
    NO_DEFAULT_PATH
)

find_path(OPENGL_INCLUDE_DIR 
    NAMES GL/gl.h
    PATHS "/nix/store/knm44q1srs7pb36ywvjzqjv2yc4axkg9-libglvnd-1.7.0-dev/include"
    PATH_SUFFIXES include
    NO_DEFAULT_PATH
)

find_package(OpenGL REQUIRED)

find_path(GLM_INCLUDE_DIR glm/glm.hpp
    PATHS "/nix/store/jhvm19kxzfkb2jqhqkcflp29dbsff96p-glm-1.0.1/include"
    NO_DEFAULT_PATH
)
include_directories(${GLM_INCLUDE_DIR})

set(GLFW3_INCLUDE_DIR "/nix/store/pihqjg71r401r8z5wf6j4hx3dv69cbqv-glfw-3.4/include")
find_library(GLFW3_LIBRARY 
    NAMES glfw glfw3
    PATHS "/nix/store/pihqjg71r401r8z5wf6j4hx3dv69cbqv-glfw-3.4/lib"
    NO_DEFAULT_PATH
)
include_directories(${GLFW3_INCLUDE_DIR})

target_link_libraries(rendering PRIVATE
  ${OPENGL_LIBRARIES} 
  ${GLFW3_LIBRARY}
  PSO_lib
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
