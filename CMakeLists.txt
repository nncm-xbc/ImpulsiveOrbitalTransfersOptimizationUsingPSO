cmake_minimum_required(VERSION 3.29)
project(PSO)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_VERBOSE_MAKEFILE ON)

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

# Set debug flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -O0 -v -g")

# Include directories
include_directories(include)

# Source files
set(SOURCES 
  src/Swarm.cpp
  src/PSO.cpp
  src/OrbitProblem.cpp
  src/OrbitTransfer.cpp
)
message(STATUS "Source Files: ${SOURCES}")

add_library(PSO_lib ${SOURCES})

# Main executable
add_executable(PSO_EXEC src/main.cpp)

target_link_libraries(PSO_EXEC PRIVATE PSO_lib)
message(STATUS "PSO_EXEC link command: ${CMAKE_CXX_LINK_EXECUTABLE}")

set_target_properties(PSO_lib PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Link OpenMP if found
target_link_libraries(PSO_EXEC PRIVATE
    $<$<BOOL:${OpenMP_CXX_FOUND}>:OpenMP::OpenMP_CXX>)

# Testing
enable_testing()
find_package(Catch2 REQUIRED)
if(Catch2_FOUND)
    message("-- Catch2 was found")

    # Add test executable
    set(TEST_SOURCES 
      tests/test_particle.cpp
      tests/test_swarm.cpp
      tests/test_PSO.cpp
      tests/test_OrbitProblem.cpp
      tests/test_OrbitTransfer.cpp
    )
  message(STATUS "Test source Files: ${TEST_SOURCES}")

  add_executable(run_tests ${TEST_SOURCES})
  
  target_link_libraries(run_tests PRIVATE PSO_lib)
  target_link_libraries(run_tests PRIVATE Catch2::Catch2WithMain)
  message(STATUS "run_tests link command: ${CMAKE_CXX_LINK_EXECUTABLE}")
  
    # Add tests
    include(Catch)
    catch_discover_tests(run_tests)

else()
  message(FATAL_ERROR "Catch2 was not found. Please install it.")
endif()
