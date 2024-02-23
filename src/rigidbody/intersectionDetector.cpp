#include "../../include/rigidbody/intersectionDetector.hpp"

// Function to check if a point lies on a given line
bool intersectionDetector::pointOnLine(vector2d point, line line) {
  // Calculate the differences in y and x coordinates of the line
  double dy = line.getTo().y - line.getFrom().y;
  double dx = line.getTo().x - line.getFrom().x;

  // Calculate the slope of the line
  double slope = dy / dx;

  // Calculate the y-intercept of the line
  double yInt = line.getFrom().y - slope * line.getFrom().x;

  // Check line equation to see if the point lies on the line
  return utils::compare(point.y, slope * point.x + yInt, 0.0001);
}

// Function to check if a point lies within a given circle
bool intersectionDetector::pointInCircle(vector2d point, circle circle) {
  // Check if the distance from the point to the center of the circle is less
  // than or equal to the radius of the circle
  return point.distanceSquared(circle.getCentre()) <=
         pow(circle.getRadius(), 2);
}

// Function to check if a point lies within a given axis-aligned bounding box
// (AABB)
bool intersectionDetector::pointInAABB(vector2d point, AABB aabb) {
  // Check if the x and y coordinates of the point are within the min and max
  // coordinates of the AABB
  return point.x >= aabb.getMin().x && point.x <= aabb.getMax().x &&
         point.y >= aabb.getMin().y && point.y <= aabb.getMax().y;
}

// Function to check if a point lies within a given box
bool intersectionDetector::pointInBox(vector2d point, box box) {
  // Translate point to box's local space
  vector2d pointLocalBox = point.rotate(box.getRigidBody().getRotation(),
                                        box.getRigidBody().getPosition());

  // Check if the translated point's x and y coordinates are within the min and
  // max coordinates of the box in its local space
  return pointLocalBox.x <= box.getLocalMax().x &&
         pointLocalBox.x >= box.getLocalMin().x &&
         pointLocalBox.y <= box.getLocalMax().y &&
         pointLocalBox.y >= box.getLocalMin().y;
}

// Function to check if a line intersects with a circle
bool intersectionDetector::lineCircle(line line, circle circle) {
  // Check if either end of the line is inside the circle
  if (pointInCircle(line.getFrom(), circle) ||
      pointInCircle(line.getTo(), circle)) {
    return true;
  }

  // Calculate the vector of the line
  vector2d lineVec = line.getTo() - line.getFrom();

  // Project circle's centre onto line
  vector2d circleCentre = circle.getCentre();
  vector2d centreToLineStart = (circleCentre - line.getFrom());
  double t = centreToLineStart.dot(lineVec) / lineVec.dot(lineVec);

  // If the projection is outside the line segment, there is no intersection
  if (t < 0.0 || t > 1.0) {
    return false;
  }

  // Calculate the closest point on the line to the circle's centre
  vector2d closestPoint = line.getFrom() + lineVec * t;

  // Check if the closest point is inside the circle
  return pointInCircle(closestPoint, circle);
}

// Function to check if a line intersects with an axis-aligned bounding box
// (AABB)
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
// Function to check if a line intersects with a box
bool intersectionDetector::lineBox(line myLine, box box) {
  // Get the rotation angle of the box and its centre
  double theta = -box.getRigidBody().getRotation();
  vector2d centre = box.getRigidBody().getPosition();

  // Rotate the line to the box's local space
  vector2d localFrom = myLine.getFrom().rotate(theta, centre);
  vector2d localTo = myLine.getTo().rotate(theta, centre);

  // Create a new line in the box's local space
  line localLine(localFrom, localTo);

  // Create an AABB in the box's local space
  AABB localAABB(box.getLocalMin(), box.getLocalMax());

  // Check if the line intersects with the AABB
  return lineAABB(localLine, localAABB);
}

// Function to perform a raycast on a circle and return the result
bool intersectionDetector::raycast(ray ray, circle circle,
                                   raycastResult &result) {
  // Reset the raycast result
  raycastResult::reset(result);

  // Calculate the vector from the ray's origin to the circle's centre
  vector2d originToCircle = circle.getCentre() - ray.getOrigin();

  // Calculate the square of the circle's radius and the length of the vector
  // from the ray's origin to the circle's centre
  double radiusSquared = pow(circle.getRadius(), 2);
  double originToCircleLengthSquared = originToCircle.dot(originToCircle);

  // Project the vector from the ray's origin to the circle onto the ray
  double a = originToCircle.dot(ray.getDirection());
  double bSq = originToCircleLengthSquared - (a * a);

  // If the square of the circle's radius minus bSq is less than zero, there is
  // no intersection
  if (radiusSquared - bSq < 0.0) {
    return false;
  }

  // Calculate the distance from the ray's origin to the intersection point
  double f = std::sqrt(radiusSquared - bSq);
  double t = originToCircle.dot(originToCircle) < radiusSquared ? a + f : a - f;

  // If t is less than zero, there is no intersection
  if (t < 0.0) {
    return false;
  }

  // Calculate the intersection point and the normal at the intersection point
  vector2d point = ray.getOrigin() + ray.getDirection() * t;
  result.init(point, (point - circle.getCentre()).normalise(), t, true);

  // There is an intersection
  return true;
}

// Function to perform a raycast on a circle
bool intersectionDetector::raycast(ray ray, circle circle) {
  // Calculate the vector from the ray's origin to the circle's centre
  vector2d originToCircle = circle.getCentre() - ray.getOrigin();

  // Calculate the square of the circle's radius and the length of the vector
  // from the ray's origin to the circle's centre
  double radiusSquared = pow(circle.getRadius(), 2);
  double originToCircleLengthSquared = originToCircle.dot(originToCircle);

  // Project the vector from the ray's origin to the circle onto the ray
  double a = originToCircle.dot(ray.getDirection());
  double bSq = originToCircleLengthSquared - (a * a);

  // If the square of the circle's radius minus bSq is less than zero, there is
  // no intersection
  if (radiusSquared - bSq < 0.0) {
    return false;
  }

  // Calculate the distance from the ray's origin to the intersection point
  double f = std::sqrt(radiusSquared - bSq);
  double t = originToCircle.dot(originToCircle) < radiusSquared ? a + f : a - f;

  // If t is less than zero, there is no intersection
  if (t < 0.0) {
    return false;
  }

  // There is an intersection
  return true;
}
// Function to perform a raycast on an AABB and return the result
bool intersectionDetector::raycast(ray ray, AABB aabb, raycastResult &result) {
  // Reset the raycast result
  raycastResult::reset(result);

  // Calculate the unit vector of the ray's direction
  vector2d unitVec = ray.getDirection();
  // If the x or y component of the unit vector is not zero, take its reciprocal
  // This is done to convert the direction vector into a direction coefficient
  unitVec.x = unitVec.x != 0 ? 1.0f / unitVec.x : 0.0f;
  unitVec.y = unitVec.x != 0 ? 1.0f / unitVec.y : 0.0f;

  // Calculate the t values for the intersection of the ray and the AABB
  // t values represent the scalar multiplier to get from the ray's origin to
  // the intersection point
  vector2d t1 = (aabb.getMin() - ray.getOrigin()) * unitVec;
  vector2d t2 = (aabb.getMax() - ray.getOrigin()) * unitVec;

  // Find the minimum and maximum t values
  // These represent the range within which the ray intersects the AABB
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
  // If tMin is less than zero, the ray starts inside the AABB, so use tMax
  double t = (tMin < 0) ? tMax : tMin;
  bool hit = t > 0;
  if (!hit) {
    return false;
  }

  // Calculate the intersection point and initialize the raycast result
  vector2d point = ray.getOrigin() + ray.getDirection() * t;
  result.init(point, (point - ray.getOrigin()).normalise(), t, true);

  // There is an intersection
  return true;
}

// Function to perform a raycast on an AABB
bool intersectionDetector::raycast(ray ray, AABB aabb) {
  // Calculate the unit vector of the ray's direction
  vector2d unitVec = ray.getDirection();
  // If the x or y component of the unit vector is not zero, take its reciprocal
  unitVec.x = unitVec.x != 0 ? 1.0f / unitVec.x : 0.0f;
  unitVec.y = unitVec.x != 0 ? 1.0f / unitVec.y : 0.0f;

  // Calculate the t values for the intersection of the ray and the AABB
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
  // If tMin is less than zero, the ray starts inside the AABB, so use tMax
  double t = (tMin < 0) ? tMax : tMin;
  bool hit = t > 0;
  if (!hit) {
    return false;
  }

  // There is an intersection
  return true;
}
// Function to perform a raycast on a box and return the result
bool intersectionDetector::raycast(ray ray, box box, raycastResult &result) {
  // Reset the raycast result
  raycastResult::reset(result);

  // Calculate the rotation of the box and the x and y axes
  double theta = -box.getRigidBody().getRotation();
  vector2d xAxis = vector2d(1, 0).rotate(theta);
  vector2d yAxis = vector2d(0, 1).rotate(theta);

  // Calculate the position of the box relative to the ray's origin
  vector2d p = box.getRigidBody().getPosition() - ray.getOrigin();

  // Calculate the dot product of the ray's direction with the x and y axes
  vector2d f(xAxis.dot(ray.getDirection()), yAxis.dot(ray.getDirection()));
  // Calculate the dot product of the relative position with the x and y axes
  vector2d e(xAxis.dot(p), yAxis.dot(p));

  // Initialize a vector to store the t values for the intersection of the ray
  // and the box
  std::vector<double> tVec(4, 0);
  for (int i = 0; i < 2; i++) {
    // If the dot product is nearly zero, check if the ray intersects the box
    if (utils::compare(f[i], 0.0, 0.0001)) {
      if (-e[i] - box.getHalfSize()[i] > 0 ||
          -e[i] + box.getHalfSize()[i] < 0) {
        return false;
      }
      // Avoid division by zero
      f[i] = 0.00001;
    }
    // Calculate the t values for the intersection of the ray and the box
    tVec[i * 2] = (e[i] + box.getHalfSize()[i]) / f[i];
    tVec[i * 2 + 1] = (e[i] - box.getHalfSize()[i]) / f[i];
  }

  // Find the minimum and maximum t values
  double tMin = *std::min_element(tVec.begin(), tVec.end());
  double tMax = *std::max_element(tVec.begin(), tVec.end());

  // If the maximum t value is less than zero or the minimum t value is greater
  // than the maximum t value, there is no intersection
  if (tMax < 0 || tMin > tMax) {
    return false;
  }

  // Calculate the t value for the intersection point
  // If tMin is less than zero, the ray starts inside the box, so use tMax
  double t = (tMin < 0) ? tMax : tMin;
  bool hit = t > 0;
  if (!hit) {
    return false;
  }

  // Calculate the intersection point and initialize the raycast result
  vector2d point = ray.getOrigin() + ray.getDirection() * t;
  result.init(point, (point - ray.getOrigin()).normalise(), t, true);

  // There is an intersection
  return true;
}

// Function to perform a raycast on a box
bool intersectionDetector::raycast(ray ray, box box) {
  // The code here is similar to the above function, but without initializing
  // the raycast result Please refer to the comments in the above function for
  // explanations
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

  double t = (tMin < 0) ? tMax : tMin;
  bool hit = t > 0;
  if (!hit) {
    return false;
  }

  return true;
}

// Function to check if two circles intersect
bool intersectionDetector::circleCircle(circle circle1, circle circle2) {
  // The circles intersect if the square of the distance between their centres
  // is less than or equal to the square of the sum of their radii
  return circle1.getCentre().distanceSquared(circle2.getCentre()) <=
         pow(circle1.getRadius() + circle2.getRadius(), 2);
}
// Function to detect intersection between a circle and an AABB
bool intersectionDetector::circleAABB(circle circle, AABB aabb) {
  // Find the closest point on the AABB to the circle
  vector2d closestPoint =
      vector2d(std::max(aabb.getMin().x,
                        std::min(circle.getCentre().x, aabb.getMax().x)),
               std::max(aabb.getMin().y,
                        std::min(circle.getCentre().y, aabb.getMax().y)));

  // If the distance from the circle center to the closest point is less than or
  // equal to the circle's radius, they intersect
  return circle.getCentre().distanceSquared(closestPoint) <=
         pow(circle.getRadius(), 2);
}

// Function to detect intersection between a circle and a box
bool intersectionDetector::circleBox(circle myCircle, box box) {
  // Treat the box as an AABB
  vector2d minVec(0, 0);
  vector2d maxVec = box.getHalfSize() * 2;

  // Calculate the local circle centre
  vector2d localCircleCentre =
      (myCircle.getCentre() - box.getRigidBody().getPosition())
          .rotate(-box.getRigidBody().getRotation()) +
      box.getHalfSize();

  // Find the closest point on the box to the circle
  vector2d closestPoint =
      vector2d(std::max(minVec.x, std::min(localCircleCentre.x, maxVec.x)),
               std::max(minVec.y, std::min(localCircleCentre.y, maxVec.y)));

  // If the distance from the circle center to the closest point is less than or
  // equal to the circle's radius, they intersect
  return localCircleCentre.distanceSquared(closestPoint) <=
         pow(myCircle.getRadius(), 2);
}

// Function to detect intersection between two AABBs
bool intersectionDetector::AABBAAABB(AABB aabb1, AABB aabb2) {
  // Define the axes to check for overlap
  std::vector<vector2d> axes = {vector2d(1, 0), vector2d(0, 1)};
  for (int i = 0; i < 2; i++) {
    // If there is no overlap on an axis, the AABBs do not intersect
    if (!overlapOnAxis(aabb1, aabb2, axes[i])) {
      return false;
    }
  }
  // If there is overlap on all axes, the AABBs intersect
  return true;
}

// Function to detect intersection between an AABB and a box
bool intersectionDetector::AABBBox(AABB aabb, box box) {
  // Define the axes to check for overlap
  std::vector<vector2d> axes = {vector2d(1, 0), vector2d(0, 1), vector2d(1, 0),
                                vector2d(0, 1)};
  // Rotate the axes according to the box's rotation
  axes[2].rotate(box.getRigidBody().getRotation());
  axes[3].rotate(box.getRigidBody().getRotation());
  for (size_t i = 0; i < axes.size(); i++) {
    // If there is no overlap on an axis, the AABB and box do not intersect
    if (!overlapOnAxis(box, aabb, axes[i])) {
      return false;
    }
  }
  // If there is overlap on all axes, the AABB and box intersect
  return true;
}

// Function to get the interval of an AABB on an axis
vector2d intersectionDetector::getInterval(AABB aabb, vector2d axis) {
  // Define the vertices of the AABB
  vector2d min = aabb.getMin();
  vector2d max = aabb.getMax();
  std::vector<vector2d> vertices = {
      vector2d(min.x, min.y), vector2d(min.x, max.y), vector2d(max.x, max.y),
      vector2d(max.x, min.y)};

  // Project the vertices onto the axis and find the min and max projections
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

  // Return the min and max projections as a 2D vector
  return vector2d(minProj, maxProj);
}

// Function to get the interval of a box on an axis
vector2d intersectionDetector::getInterval(box box, vector2d axis) {
  // Get the vertices of the box
  std::vector<vector2d> vertices = box.getVertices();

  // Project the vertices onto the axis and find the min and max projections
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

  // Return the min and max projections as a 2D vector
  return vector2d(minProj, maxProj);
}

// Function to check if two AABBs overlap on an axis
bool intersectionDetector::overlapOnAxis(AABB aabb1, AABB aabb2,
                                         vector2d axis) {
  // Get the intervals of the AABBs on the axis
  vector2d a = getInterval(aabb1, axis);
  vector2d b = getInterval(aabb2, axis);

  // If the intervals overlap, the AABBs overlap on this axis
  return (a.x <= b.y && a.y >= b.x);
}

// Function to check if a box and an AABB overlap on an axis
bool intersectionDetector::overlapOnAxis(box box, AABB aabb2, vector2d axis) {
  // Get the intervals of the box and AABB on the axis
  vector2d a = getInterval(box, axis);
  vector2d b = getInterval(aabb2, axis);

  // If the intervals overlap, the box and AABB overlap on this axis
  return (a.x <= b.y && a.y >= b.x);
}

// Function to check if two boxes overlap on an axis
bool intersectionDetector::overlapOnAxis(box box1, box box2, vector2d axis) {
  // Get the intervals of the boxes on the axis
  vector2d a = getInterval(box1, axis);
  vector2d b = getInterval(box2, axis);

  // If the intervals overlap, the boxes overlap on this axis
  return (a.x <= b.y && a.y >= b.x);
}

// Function to detect intersection between two boxes
bool intersectionDetector::boxBox(box box1, box box2) {
  // Define the axes to check for overlap
  std::vector<vector2d> axes = {vector2d(1, 0), vector2d(0, 1), vector2d(1, 0),
                                vector2d(0, 1)};
  // Rotate the axes according to the first box's rotation
  axes[2].rotate(box1.getRigidBody().getRotation());
  axes[3].rotate(box1.getRigidBody().getRotation());
  for (size_t i = 0; i < axes.size(); i++) {
    // If there is no overlap on an axis, the boxes do not intersect
    if (!overlapOnAxis(box1, box2, axes[i])) {
      return false;
    }
  }
  // If there is overlap on all axes, the boxes intersect
  return true;
}