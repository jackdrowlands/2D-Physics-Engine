#include "../../include/primitives/circle.hpp"

circle::circle(double radius) : radius(radius) {}

circle::~circle() {}

double circle::getRadius() { return radius; }

vector2d circle::getCentre() { return body.getPosition(); }