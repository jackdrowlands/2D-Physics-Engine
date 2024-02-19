#include "../../include/primitives/circle.hpp"

circle::circle(double radius) : radius(radius) {}

circle::circle(double radius, vector2d centre)
    : radius(radius), body(centre, 0) {}

circle::~circle() {}

double circle::getRadius() { return radius; }

vector2d circle::getCentre() { return body.getPosition(); }

void circle::setCentre(vector2d newCentre) { body.setPosition(newCentre); }

void circle::setRadius(double newRadius) { radius = newRadius; }

rigidBody& circle::getRigidBody() { return body; }

void circle::setRigidBody(rigidBody body) { this->body = body; }

std::string circle::getType() { return "circle"; }
