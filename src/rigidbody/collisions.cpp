#include "../../include/rigidbody/collisions.hpp"

collisionManifold collisions::circleToCircle(circle& a, circle& b) {
  vector2d distance = b.getCentre() - a.getCentre();
  double radius = a.getRadius() + b.getRadius();

  if (b.getCentre().distanceSquared(a.getCentre()) > pow(radius, 2)) {
    return collisionManifold(vector2d(0, 0), 0, false);
  }
  std::cout << "Collision" << std::endl;
  double depth = (double)abs(distance.magnitude() - radius) * 0.5;
  vector2d normal = distance.normalise();
  double distanceToMove = a.getRadius() - depth;
  vector2d contactPoint = a.getCentre() + normal * distanceToMove;
  collisionManifold res = collisionManifold(normal, depth, true);
  res.addContactPoint(contactPoint);
  return res;
}

collisionManifold* collisions::findCollisionFeatures(collider* c1,
                                                     collider* c2) {
  // check if both colliders are circles
  if (c1->getType() == "circle" && c2->getType() == "circle") {
    circle* a = dynamic_cast<circle*>(c1);
    circle* b = dynamic_cast<circle*>(c2);
    return new collisionManifold(circleToCircle(*a, *b));
  }
  std::cout << "No collision detection for " << c1->getType() << " and "
            << c2->getType() << std::endl;
  return nullptr;
}