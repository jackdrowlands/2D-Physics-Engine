cmake_minimum_required(VERSION 3.14) # Required for FetchContent

project(2DPhysicsEngine)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories for header files
include_directories(${CMAKE_SOURCE_DIR}/include)

include(FetchContent)

FetchContent_Declare(
  SFML
  GIT_REPOSITORY https://github.com/SFML/SFML.git
  GIT_TAG 2.6.x)
FetchContent_MakeAvailable(SFML)

# Debugging flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wextra -pg")

# Source files
file(GLOB SOURCE_FILES
    src/*.cpp
    src/datatypes/*.cpp
    src/primitives/*.cpp
    src/rigidbody/*.cpp
    src/forces/*.cpp
    src/render/*.cpp

)

# Main executable
add_executable(2DPhysicsEngine ${SOURCE_FILES})

# Link SFML to your project
target_link_libraries(2DPhysicsEngine PRIVATE sfml-graphics)

# Fetch Google Test
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
    src/utils.cpp
    src/physicsSystem.cpp
    src/forces/*.cpp
)

# Test executable
add_executable(2DPhysicsEngineTest ${TEST_FILES})

# Link Google Test
target_link_libraries(2DPhysicsEngineTest gtest_main)

# Add tests
include(GoogleTest)
gtest_discover_tests(2DPhysicsEngineTest)
