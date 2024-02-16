cmake_minimum_required(VERSION 3.14) # Required for FetchContent

project(2DPhysicsEngine)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories for header files
include_directories(${CMAKE_SOURCE_DIR}/include)

# Automatically include source files
file(GLOB_RECURSE SOURCE_FILES 
     "${CMAKE_SOURCE_DIR}/src/*.cpp"
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

# Define TEST_FILES variable if you have specific test files to include
file(GLOB_RECURSE TEST_FILES 
     "${CMAKE_SOURCE_DIR}/tests/*.cpp"
)

# Test executable
add_executable(2DPhysicsEngineTests ${TEST_FILES})

target_link_libraries(2DPhysicsEngineTests gtest_main)

include(GoogleTest)
gtest_discover_tests(2DPhysicsEngineTests)
