#include "include/primitives/circle.hpp"

circle::circle(float radius) : radius(radius) {}

circle::~circle() {}

float circle::get_radius() { return radius; }