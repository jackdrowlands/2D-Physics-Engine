#include "../../include/primitives/circle.hpp"

// Constructor that initializes circle with given radius
circle::circle(double radius) : radius(radius) {}

// Constructor that initializes circle with given radius and centre
circle::circle(double radius, vector2d centre)
    : radius(radius), body(centre, 0) {}

// Destructor
circle::~circle() {}

// Function to get the radius of the circle
double circle::getRadius() { return radius; }

// Function to get the centre of the circle
vector2d circle::getCentre() { return body.getPosition(); }

// Function to set the centre of the circle
void circle::setCentre(vector2d newCentre) { body.setPosition(newCentre); }

// Function to set the radius of the circle
void circle::setRadius(double newRadius) { radius = newRadius; }

// Function to get the rigidBody of the circle
rigidBody& circle::getRigidBody() { return body; }

// Function to set the rigidBody of the circle
void circle::setRigidBody(rigidBody body) { this->body = body; }

// Function to get the type of the primitive
std::string circle::getType() { return "circle"; }