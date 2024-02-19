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
  vector2d n = b.getRigidBody().getPosition() - a.getRigidBody().getPosition();
  double aExtent = a.getHalfSize().x;
  double bExtent = b.getHalfSize().x;

  double aMax = aExtent;
  double bMax = bExtent;
  double aMin = -aExtent;
  double bMin = -bExtent;

  // Calculate the projection axis
  vector2d axis = n.normalise();

  // Project the polygons onto the axis
  double aProjection = aExtent * abs(axis.dot(a.getRigidBody().getPosition()));
  double bProjection = bExtent * abs(axis.dot(b.getRigidBody().getPosition()));

  double distance = abs(n.dot(axis));
  if (aProjection + bProjection < distance) {
    return collisionManifold(vector2d(0, 0), 0, false);
  }

  double overlap = aProjection + bProjection - distance;
  double depth = overlap * 0.5;
  vector2d normal;
  if (n.dot(axis) < 0) {
    normal = -axis;
  } else {
    normal = axis;
  }
  vector2d contactPoint = a.getRigidBody().getPosition() + normal * aProjection;
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
  if (c1->getType() == "box" && c2->getType() == "box") {
    box* a = dynamic_cast<box*>(c1);
    box* b = dynamic_cast<box*>(c2);
    return new collisionManifold(boxToBox(*a, *b));
  }
  std::cout << "No collision detection for " << c1->getType() << " and "
            << c2->getType() << std::endl;
  return nullptr;
}