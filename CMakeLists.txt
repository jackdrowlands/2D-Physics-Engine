cmake_minimum_required(VERSION 3.14) # Required for FetchContent

project(2DPhysicsEngine)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories for header files
include_directories(${CMAKE_SOURCE_DIR}/include)

# Source files
file(GLOB SOURCE_FILES
    src/*.cpp
    src/datatypes/*.cpp
    src/primitives/*.cpp
     src/rigidbody/*.cpp
)

# Main executable
add_executable(2DPhysicsEngine ${SOURCE_FILES})

# Fetch Google Test
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/heads/master.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

# Test files
file(GLOB TEST_FILES
    tests/*.cpp
    tests/datatypes/*.cpp
    tests/primitives/*.cpp
    tests/rigidbody/*.cpp
    src/datatypes/*.cpp
     src/primitives/*.cpp
     src/rigidbody/*.cpp
     src/*.cpp
)

# Test executable
add_executable(2DPhysicsEngineTests ${TEST_FILES})

target_link_libraries(2DPhysicsEngineTests gtest_main)

include(GoogleTest)
gtest_discover_tests(2DPhysicsEngineTests)
