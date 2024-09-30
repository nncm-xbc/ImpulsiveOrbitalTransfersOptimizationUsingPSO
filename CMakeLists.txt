cmake_minimum_required(VERSION 3.20)
project(PSO)

set(CMAKE_CXX_STANDARD 20)

set(GTEST_ROOT /usr/src/gtest)
set(GTEST_INCLUDE_DIR ${GTEST_ROOT}/include)
set(GTEST_LIBRARY ${GTEST_ROOT}/lib/libgtest.a)
set(GTEST_MAIN_LIBRARY ${GTEST_ROOT}/lib/libgtest_main.a)

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
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -g")

# Include directories
include_directories(include)

# Source files
file(GLOB SOURCES "src/*.cpp")

# Remove main.cpp from SOURCES
list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

add_library(PSO_lib ${SOURCES})

# Main executable
add_executable(PSO_EXEC src/main.cpp)
target_link_libraries(PSO_EXEC PRIVATE PSO_lib)

# Link OpenMP if found
target_link_libraries(PSO_EXEC PRIVATE
    $<$<BOOL:${OpenMP_CXX_FOUND}>:OpenMP::OpenMP_CXX>
)

# Testing
enable_testing()
find_package(GTest REQUIRED)
if(GTest_FOUND)
    message("-- Google Test was found")
    include_directories(${GTEST_INCLUDE_DIRS})

    # Add test executable
    file(GLOB TEST_SOURCES "tests/*.cpp")
    add_executable(run_tests ${TEST_SOURCES})

    # Link test executable with Google Test and main project
    target_link_libraries(run_tests PRIVATE
      ${GTEST_BOTH_LIBRARIES} pthread PSO_lib)

    # Add tests
    add_test(NAME PSO_tests COMMAND run_tests)
else()
    message(FATAL_ERROR "Google Test not found. Please install it.")
endif()
