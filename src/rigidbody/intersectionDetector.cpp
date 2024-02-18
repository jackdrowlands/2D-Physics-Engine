#include "../../include/rigidbody/intersectionDetector.hpp"

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
  return pointLocalBox.x <= box.getLocalMax().x &&
         pointLocalBox.x >= box.getLocalMin().x &&
         pointLocalBox.y <= box.getLocalMax().y &&
         pointLocalBox.y >= box.getLocalMin().y;
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

bool intersectionDetector::lineAABB(line line, AABB aabb) {
  // Check if either end of the line is inside the AABB
  if (pointInAABB(line.getFrom(), aabb) || pointInAABB(line.getTo(), aabb)) {
    return true;
  }

  // Calculate the unit vector of the line
  vector2d unitVec = (line.getTo() - line.getFrom()).normalise();
  // If the x or y component of the unit vector is not zero, take its reciprocal
  unitVec.x = unitVec.x != 0 ? 1.0f / unitVec.x : 0.0f;
  unitVec.y = unitVec.x != 0 ? 1.0f / unitVec.y : 0.0f;

  // Calculate the t values for the intersection of the line and the AABB
  vector2d t1 = (aabb.getMin() - line.getFrom()) * unitVec;
  vector2d t2 = (aabb.getMax() - line.getFrom()) * unitVec;

  // Find the minimum and maximum t values
  double tMin = std::min(std::min(std::max(t1.x, t2.x), std::max(t1.y, t2.y)),
                         std::max(std::max(t1.x, t2.x), std::min(t1.y, t2.y)));
  double tMax = std::max(std::max(std::min(t1.x, t2.x), std::min(t1.y, t2.y)),
                         std::min(std::min(t1.x, t2.x), std::max(t1.y, t2.y)));

  // If the maximum t value is less than zero or the minimum t value is greater
  // than the maximum t value, there is no intersection
  if (tMax < 0 || tMin > tMax) {
    return false;
  }

  // Calculate the t value for the intersection point
  double t = (tMin < 0) ? tMax : tMin;
  // Check if the intersection point is on the line segment
  return t > 0 && t * t < line.getTo().distanceSquared(line.getFrom());
}

bool intersectionDetector::lineBox(line myLine, box box) {
  double theta = -box.getRigidBody().getRotation();
  vector2d centre = box.getRigidBody().getPosition();
  vector2d localFrom = myLine.getFrom().rotate(theta, centre);
  vector2d localTo = myLine.getTo().rotate(theta, centre);

  line localLine(localFrom, localTo);
  AABB localAABB(box.getLocalMin(), box.getLocalMax());

  return lineAABB(localLine, localAABB);
}

bool intersectionDetector::raycast(ray ray, circle circle,
                                   raycastResult &result) {
  raycastResult::reset(result);

  vector2d originToCircle = circle.getCentre() - ray.getOrigin();
  double radiusSquared = pow(circle.getRadius(), 2);
  double originToCircleLengthSquared = originToCircle.dot(originToCircle);

  // project vector from origin to circle onto ray

  double a = originToCircle.dot(ray.getDirection());
  double bSq = originToCircleLengthSquared - (a * a);

  if (radiusSquared - bSq < 0.0) {
    return false;
  }

  double f = std::sqrt(radiusSquared - bSq);
  double t = originToCircle.dot(originToCircle) < radiusSquared ? a + f : a - f;

  if (t < 0.0) {
    return false;
  }
  vector2d point = ray.getOrigin() + ray.getDirection() * t;
  result.init(point, (point - circle.getCentre()).normalise(), t, true);

  return true;
}

bool intersectionDetector::raycast(ray ray, circle circle) {
  vector2d originToCircle = circle.getCentre() - ray.getOrigin();
  double radiusSquared = pow(circle.getRadius(), 2);
  double originToCircleLengthSquared = originToCircle.dot(originToCircle);

  // project vector from origin to circle onto ray

  double a = originToCircle.dot(ray.getDirection());
  double bSq = originToCircleLengthSquared - (a * a);

  if (radiusSquared - bSq < 0.0) {
    return false;
  }

  double f = std::sqrt(radiusSquared - bSq);
  double t = originToCircle.dot(originToCircle) < radiusSquared ? a + f : a - f;

  if (t < 0.0) {
    return false;
  }

  return true;
}

bool intersectionDetector::raycast(ray ray, AABB aabb, raycastResult &result) {
  raycastResult::reset(result);

  // Calculate the unit vector of the line
  vector2d unitVec = ray.getDirection();
  // If the x or y component of the unit vector is not zero, take its reciprocal
  unitVec.x = unitVec.x != 0 ? 1.0f / unitVec.x : 0.0f;
  unitVec.y = unitVec.x != 0 ? 1.0f / unitVec.y : 0.0f;

  // Calculate the t values for the intersection of the line and the AABB
  vector2d t1 = (aabb.getMin() - ray.getOrigin()) * unitVec;
  vector2d t2 = (aabb.getMax() - ray.getOrigin()) * unitVec;

  // Find the minimum and maximum t values
  double tMin = std::min(std::min(std::max(t1.x, t2.x), std::max(t1.y, t2.y)),
                         std::max(std::max(t1.x, t2.x), std::min(t1.y, t2.y)));
  double tMax = std::max(std::max(std::min(t1.x, t2.x), std::min(t1.y, t2.y)),
                         std::min(std::min(t1.x, t2.x), std::max(t1.y, t2.y)));

  // If the maximum t value is less than zero or the minimum t value is greater
  // than the maximum t value, there is no intersection
  if (tMax < 0 || tMin > tMax) {
    return false;
  }

  // Calculate the t value for the intersection point
  double t = (tMin < 0) ? tMax : tMin;
  bool hit = t > 0;
  if (!hit) {
    return false;
  }
  vector2d point = ray.getOrigin() + ray.getDirection() * t;
  result.init(point, (point - ray.getOrigin()).normalise(), t, true);

  return true;
}

bool intersectionDetector::raycast(ray ray, AABB aabb) {
  // Calculate the unit vector of the line
  vector2d unitVec = ray.getDirection();
  // If the x or y component of the unit vector is not zero, take its reciprocal
  unitVec.x = unitVec.x != 0 ? 1.0f / unitVec.x : 0.0f;
  unitVec.y = unitVec.x != 0 ? 1.0f / unitVec.y : 0.0f;

  // Calculate the t values for the intersection of the line and the AABB
  vector2d t1 = (aabb.getMin() - ray.getOrigin()) * unitVec;
  vector2d t2 = (aabb.getMax() - ray.getOrigin()) * unitVec;

  // Find the minimum and maximum t values
  double tMin = std::min(std::min(std::max(t1.x, t2.x), std::max(t1.y, t2.y)),
                         std::max(std::max(t1.x, t2.x), std::min(t1.y, t2.y)));
  double tMax = std::max(std::max(std::min(t1.x, t2.x), std::min(t1.y, t2.y)),
                         std::min(std::min(t1.x, t2.x), std::max(t1.y, t2.y)));

  // If the maximum t value is less than zero or the minimum t value is greater
  // than the maximum t value, there is no intersection
  if (tMax < 0 || tMin > tMax) {
    return false;
  }

  // Calculate the t value for the intersection point
  double t = (tMin < 0) ? tMax : tMin;
  bool hit = t > 0;
  if (!hit) {
    return false;
  }
  return true;
}

bool intersectionDetector::raycast(ray ray, box box, raycastResult &result) {
  raycastResult::reset(result);

  double theta = -box.getRigidBody().getRotation();
  vector2d xAxis = vector2d(1, 0).rotate(theta);
  vector2d yAxis = vector2d(0, 1).rotate(theta);
  vector2d p = box.getRigidBody().getPosition() - ray.getOrigin();

  vector2d f(xAxis.dot(ray.getDirection()), yAxis.dot(ray.getDirection()));
  vector2d e(xAxis.dot(p), yAxis.dot(p));

  std::vector<double> tVec(4, 0);
  for (int i = 0; i < 2; i++) {
    if (utils::compare(f[i], 0.0, 0.0001)) {
      if (-e[i] - box.getHalfSize()[i] > 0 ||
          -e[i] + box.getHalfSize()[i] < 0) {
        return false;
      }
      f[i] = 0.00001;
    }
    tVec[i * 2] = (e[i] + box.getHalfSize()[i]) / f[i];
    tVec[i * 2 + 1] = (e[i] - box.getHalfSize()[i]) / f[i];
  }

  double tMin = *std::min_element(tVec.begin(), tVec.end());
  double tMax = *std::max_element(tVec.begin(), tVec.end());

  if (tMax < 0 || tMin > tMax) {
    return false;
  }

  // Calculate the t value for the intersection point
  double t = (tMin < 0) ? tMax : tMin;
  bool hit = t > 0;
  if (!hit) {
    return false;
  }
  vector2d point = ray.getOrigin() + ray.getDirection() * t;
  result.init(point, (point - ray.getOrigin()).normalise(), t, true);
  return true;
}

bool intersectionDetector::raycast(ray ray, box box) {
  double theta = -box.getRigidBody().getRotation();
  vector2d xAxis = vector2d(1, 0).rotate(theta);
  vector2d yAxis = vector2d(0, 1).rotate(theta);
  vector2d p = box.getRigidBody().getPosition() - ray.getOrigin();

  vector2d f(xAxis.dot(ray.getDirection()), yAxis.dot(ray.getDirection()));
  vector2d e(xAxis.dot(p), yAxis.dot(p));

  std::vector<double> tVec(4, 0);
  for (int i = 0; i < 2; i++) {
    if (utils::compare(f[i], 0.0, 0.0001)) {
      if (-e[i] - box.getHalfSize()[i] > 0 ||
          -e[i] + box.getHalfSize()[i] < 0) {
        return false;
      }
      f[i] = 0.00001;
    }
    tVec[i * 2] = (e[i] + box.getHalfSize()[i]) / f[i];
    tVec[i * 2 + 1] = (e[i] - box.getHalfSize()[i]) / f[i];
  }

  double tMin = *std::min_element(tVec.begin(), tVec.end());
  double tMax = *std::max_element(tVec.begin(), tVec.end());

  if (tMax < 0 || tMin > tMax) {
    return false;
  }

  // Calculate the t value for the intersection point
  double t = (tMin < 0) ? tMax : tMin;
  bool hit = t > 0;
  if (!hit) {
    return false;
  }
  return true;
}

bool intersectionDetector::circleCircle(circle circle1, circle circle2) {
  return circle1.getCentre().distanceSquared(circle2.getCentre()) <=
         pow(circle1.getRadius() + circle2.getRadius(), 2);
}

bool intersectionDetector::circleAABB(circle circle, AABB aabb) {
  vector2d closestPoint =
      vector2d(std::max(aabb.getMin().x,
                        std::min(circle.getCentre().x, aabb.getMax().x)),
               std::max(aabb.getMin().y,
                        std::min(circle.getCentre().y, aabb.getMax().y)));

  return circle.getCentre().distanceSquared(closestPoint) <=
         pow(circle.getRadius(), 2);
}

bool intersectionDetector::circleBox(circle myCircle, box box) {
  // treat the box as an AABB
  vector2d minVec(0, 0);
  vector2d maxVec = box.getHalfSize() * 2;

  vector2d localCircleCentre =
      (myCircle.getCentre() - box.getRigidBody().getPosition())
          .rotate(-box.getRigidBody().getRotation()) +
      box.getHalfSize();

  vector2d closestPoint =
      vector2d(std::max(minVec.x, std::min(localCircleCentre.x, maxVec.x)),
               std::max(minVec.y, std::min(localCircleCentre.y, maxVec.y)));

  return localCircleCentre.distanceSquared(closestPoint) <=
         pow(myCircle.getRadius(), 2);
}

bool intersectionDetector::AABBAAABB(AABB aabb1, AABB aabb2) {
  std::vector<vector2d> axes = {vector2d(1, 0), vector2d(0, 1)};
  for (int i = 0; i < 2; i++) {
    if (!overlapOnAxis(aabb1, aabb2, axes[i])) {
      return false;
    }
  }
  return true;
}

bool intersectionDetector::AABBBox(AABB aabb, box box) {
  std::vector<vector2d> axes = {vector2d(1, 0), vector2d(0, 1), vector2d(1, 0),
                                vector2d(0, 1)};
  axes[2].rotate(box.getRigidBody().getRotation());
  axes[3].rotate(box.getRigidBody().getRotation());
  for (size_t i = 0; i < axes.size(); i++) {
    if (!overlapOnAxis(box, aabb, axes[i])) {
      return false;
    }
  }
  return true;
}

vector2d intersectionDetector::getInterval(AABB aabb, vector2d axis) {
  vector2d min = aabb.getMin();
  vector2d max = aabb.getMax();
  std::vector<vector2d> vertices = {
      vector2d(min.x, min.y), vector2d(min.x, max.y), vector2d(max.x, max.y),
      vector2d(max.x, min.y)};

  double minProj = axis.dot(vertices[0]);
  double maxProj = minProj;

  for (int i = 1; i < 4; i++) {
    double proj = axis.dot(vertices[i]);
    if (proj < minProj) {
      minProj = proj;
    } else if (proj > maxProj) {
      maxProj = proj;
    }
  }

  return vector2d(minProj, maxProj);
}

vector2d intersectionDetector::getInterval(box box, vector2d axis) {
  std::vector<vector2d> vertices = box.getVertices();

  double minProj = axis.dot(vertices[0]);
  double maxProj = minProj;

  for (int i = 1; i < 4; i++) {
    double proj = axis.dot(vertices[i]);
    if (proj < minProj) {
      minProj = proj;
    } else if (proj > maxProj) {
      maxProj = proj;
    }
  }

  return vector2d(minProj, maxProj);
}

bool intersectionDetector::overlapOnAxis(AABB aabb1, AABB aabb2,
                                         vector2d axis) {
  vector2d a = getInterval(aabb1, axis);
  vector2d b = getInterval(aabb2, axis);

  return (a.x <= b.y && a.y >= b.x);
}

bool intersectionDetector::overlapOnAxis(box box, AABB aabb2, vector2d axis) {
  vector2d a = getInterval(box, axis);
  vector2d b = getInterval(aabb2, axis);

  return (a.x <= b.y && a.y >= b.x);
}

bool intersectionDetector::overlapOnAxis(box box1, box box2, vector2d axis) {
  vector2d a = getInterval(box1, axis);
  vector2d b = getInterval(box2, axis);

  return (a.x <= b.y && a.y >= b.x);
}