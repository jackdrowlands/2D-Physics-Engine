#include "../../include/rigidbody/collisions.hpp"

enum Axis { FACE_A_X, FACE_A_Y, FACE_B_X, FACE_B_Y };

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

collisionManifold collisions::boxToBox(box& a, box& b) {
  if (!intersectionDetector::boxBox(a, b)) {
    return collisionManifold(vector2d(0, 0), 0, false);
  }
  // TODO: Implement box to box collision detection
}

collisionManifold* collisions::findCollisionFeatures(collider* c1,
                                                     collider* c2) {
  // check if both colliders are circles
  if (c1->getType() == "circle" && c2->getType() == "circle") {
    circle* a = dynamic_cast<circle*>(c1);
    circle* b = dynamic_cast<circle*>(c2);
    return new collisionManifold(circleToCircle(*a, *b));
  }
  if (c1->getType() == "box" && c2->getType() == "box") {
    box* a = dynamic_cast<box*>(c1);
    box* b = dynamic_cast<box*>(c2);
    return new collisionManifold(boxToBox(*a, *b));
  }
  std::cout << "No collision detection for " << c1->getType() << " and "
            << c2->getType() << std::endl;
  return nullptr;
}