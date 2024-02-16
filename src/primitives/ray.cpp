#include "../../include/primitives/ray.hpp"

ray::ray(vector2d origin, vector2d direction)
    : origin(origin), direction(direction.normalise()) {}

ray::~ray() {}

vector2d ray::getOrigin() { return origin; }

vector2d ray::getDirection() { return direction; }