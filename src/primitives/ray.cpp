#include "../../include/primitives/ray.hpp"

// Constructor that initializes ray with given origin and direction
// The direction is normalized to ensure it's a unit vector
ray::ray(vector2d origin, vector2d direction)
    : origin(origin), direction(direction.normalise()) {}

// Destructor
ray::~ray() {}

// Function to get the origin of the ray
vector2d ray::getOrigin() { return origin; }

// Function to get the direction of the ray
vector2d ray::getDirection() { return direction; }