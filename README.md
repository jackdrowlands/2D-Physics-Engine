# 2D Physics Engine

This project is a 2D physics engine implemented in C++. It provides a variety of features to simulate physics in a 2D environment.

## Features

- Basic physics entities like vectors, lines, and matrices.
- Rigid body dynamics.
- Collision detection using SAT (Separating Axis Theorem).
- Collision resolution using impulse-based methods.
- Force generators and force registry for applying forces to bodies.
- Intersection detection for various shapes like circles, boxes, and lines.
- Rendering capabilities using SFML.
- Unit tests using Google Test.
- A simple example application to demonstrate the engine's capabilities.

## Building the Project

This project uses CMake for building. The minimum required version of CMake is 3.14.

To build the project, navigate to the project directory and run the following commands:

```sh
mkdir build
cd build
cmake ..
make
```

This will create an executable named `2DPhysicsEngine`.

## Running the Tests

The project uses Google Test for unit testing. The tests are built along with the project.

To run the tests, navigate to the build directory and run the following command:

```sh
./2DPhysicsEngineTest
```