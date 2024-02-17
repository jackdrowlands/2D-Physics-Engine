#include "../../include/rigidbody/rigidBody.hpp"

rigidBody::rigidBody() : rotation(0.0f) {}

rigidBody::rigidBody(vector2d position, double rotation)
    : position(position), rotation(rotation) {}

rigidBody::~rigidBody() {}

vector2d rigidBody::getPosition() { return position; }

void rigidBody::setPosition(vector2d position) { this->position = position; }

double rigidBody::getRotation() { return rotation; }

void rigidBody::setRotation(double rotation) { this->rotation = rotation; }