#include "../../include/rigidbody/collisionManifold.hpp"

collisionManifold::collisionManifold(vector2d normal,

                                     double depth, bool isColliding)
    : normal(normal), depth(depth), isColliding(isColliding) {}

collisionManifold::~collisionManifold() {}

vector2d collisionManifold::getNormal() { return normal; }

std::vector<vector2d> collisionManifold::getContactPoint() {
  return contactPoint;
}

double collisionManifold::getDepth() { return depth; }

bool collisionManifold::getIsColliding() { return isColliding; }

void collisionManifold::addContactPoint(vector2d point) {
  contactPoint.push_back(point);
}