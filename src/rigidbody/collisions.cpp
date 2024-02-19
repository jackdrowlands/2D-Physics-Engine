#include "../../include/rigidbody/collisions.hpp"

collisionManifold collisions::circleToCircle(circle& a, circle& b) {
  vector2d distance = b.getCentre() - a.getCentre();
  double radius = a.getRadius() + b.getRadius();

  if (b.getCentre().distanceSquared(a.getCentre()) > pow(radius, 2)) {
    return collisionManifold(vector2d(0, 0), 0, false);
  }

  double depth = abs(distance.magnitude() - radius) * 0.5;
  vector2d normal = distance.normalise();
  double distanceToMove = a.getRadius() - depth;
  vector2d contactPoint = a.getCentre() + normal * distanceToMove;
  collisionManifold res = collisionManifold(normal, depth, true);
  res.addContactPoint(contactPoint);
  return res;
}