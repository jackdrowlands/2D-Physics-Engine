#include "../../include/rigidbody/rigidBody.hpp"

rigidBody::rigidBody() : rotation(0.0f) {}

rigidBody::~rigidBody() {}

vector2f rigidBody::getPosition() { return position; }

void rigidBody::setPosition(vector2f position) { this->position = position; }

float rigidBody::getRotation() { return rotation; }

void rigidBody::setRotation(float rotation) { this->rotation = rotation; }