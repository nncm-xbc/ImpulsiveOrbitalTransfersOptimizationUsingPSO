cmake_minimum_required(VERSION 3.20)
project(PSO)

set(CMAKE_CXX_STANDARD 20)

set(GTEST_ROOT /usr/src/gtest)
set(GTEST_INCLUDE_DIR ${GTEST_ROOT}/include)
set(GTEST_LIBRARY ${GTEST_ROOT}/lib/libgtest.a)
set(GTEST_MAIN_LIBRARY ${GTEST_ROOT}/lib/libgtest_main.a)

find_package(GTest REQUIRED)
if(GTest_FOUND)
    message("-- Google Test was found")
    include_directories(${GTEST_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "Google Test not found. Please install it.")
endif()

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    message("-- OpenMP was found")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 ${OpenMP_CXX_FLAGS}")
    add_definitions(-DUSE_OPENMP)
else()
    message("-- OpenMP not found. Building without OpenMP support.")
    set(CMAKE_CXX_FLAGS "-O3")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")

include_directories(include)

file(GLOB SOURCES "src/*.cpp")

add_executable(PSO_EXEC ${SOURCES})

target_link_libraries(PSO_EXEC PUBLIC 
    ${GTEST_LIBRARIES}
    $<$<BOOL:${OpenMP_CXX_FOUND}>:OpenMP::OpenMP_CXX>
)
