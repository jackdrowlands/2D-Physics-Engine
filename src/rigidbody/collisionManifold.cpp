#include "../../include/rigidbody/collisionManifold.hpp"

// Constructor that initializes collisionManifold with given normal, depth, and
// isColliding The normal is the direction of the collision The depth is the
// penetration depth of the collision isColliding is a boolean indicating
// whether a collision occurred
collisionManifold::collisionManifold(vector2d normal, double depth,
                                     bool isColliding)
    : normal(normal), depth(depth), isColliding(isColliding) {}

// Destructor
collisionManifold::~collisionManifold() {}

// Function to get the normal of the collision
vector2d collisionManifold::getNormal() { return normal; }

// Function to get the contact points of the collision
std::vector<vector2d> collisionManifold::getContactPoint() {
  return contactPoint;
}

// Function to get the penetration depth of the collision
double collisionManifold::getDepth() { return depth; }

// Function to check if a collision occurred
bool collisionManifold::getIsColliding() { return isColliding; }

// Function to add a contact point to the collision
void collisionManifold::addContactPoint(vector2d point) {
  contactPoint.push_back(point);
}