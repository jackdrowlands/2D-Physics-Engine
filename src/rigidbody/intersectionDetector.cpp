#include "../../include/rigidbody/intersectionDetector.hpp"

intersectionDetector::intersectionDetector(/* args */) {}

intersectionDetector::~intersectionDetector() {}

bool intersectionDetector::pointOnLine(vector2d point, line line) {
  double dy = line.getTo().y - line.getFrom().y;
  double dx = line.getTo().x - line.getFrom().x;
  double slope = dy / dx;
  double yInt = line.getFrom().y - slope * line.getFrom().x;

  // Check line equation
  return utils::compare(point.y, slope * point.x + yInt, 0.0001);
}

bool intersectionDetector::pointInCircle(vector2d point, circle circle) {
  return point.distanceSquared(circle.getCentre()) <=
         pow(circle.getRadius(), 2);
}

bool intersectionDetector::pointInAABB(vector2d point, AABB aabb) {
  return point.x >= aabb.getMin().x && point.x <= aabb.getMax().x &&
         point.y >= aabb.getMin().y && point.y <= aabb.getMax().y;
}

bool intersectionDetector::pointInBox(vector2d point, box box) {
  // Translate point to box's local space

  vector2d pointLocalBox = point.rotate(box.getRigidBody().getRotation(),
                                        box.getRigidBody().getPosition());

  // Check if point is in box's local space
  return pointLocalBox.x <= box.getMax().x &&
         pointLocalBox.x >= box.getMin().x &&
         pointLocalBox.y <= box.getMax().y && pointLocalBox.y >= box.getMin().y;
}

bool intersectionDetector::lineCircle(line line, circle circle) {
  if (pointInCircle(line.getFrom(), circle) ||
      pointInCircle(line.getTo(), circle)) {
    return true;
  }

  vector2d lineVec = line.getTo() - line.getFrom();
  // Project circle's centre onto line
  vector2d circleCentre = circle.getCentre();
  vector2d centreToLineStart = (circleCentre - line.getFrom());
  double t = centreToLineStart.dot(lineVec) / lineVec.dot(lineVec);

  if (t < 0.0 || t > 1.0) {
    return false;
  }

  vector2d closestPoint = line.getFrom() + lineVec * t;

  return pointInCircle(closestPoint, circle);
}