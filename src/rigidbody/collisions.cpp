#include "../../include/rigidbody/collisions.hpp"

// Enum for the axes
enum Axis { FACE_A_X, FACE_A_Y, FACE_B_X, FACE_B_Y };

// Function to detect collision between two circles
collisionManifold collisions::circleToCircle(circle& a, circle& b) {
  vector2d distance = b.getCentre() - a.getCentre();
  double radius = a.getRadius() + b.getRadius();

  // If the distance between the centres of the circles is greater than the sum
  // of their radii, they are not colliding
  if (b.getCentre().distanceSquared(a.getCentre()) > pow(radius, 2)) {
    return collisionManifold(vector2d(0, 0), 0, false);
  }

  // If the circles are colliding, calculate the depth of the collision and the
  // contact point
  double depth = (double)abs(distance.magnitude() - radius) * 0.5;
  vector2d normal = distance.normalise();
  double distanceToMove = a.getRadius() - depth;
  vector2d contactPoint = a.getCentre() + normal * distanceToMove;
  collisionManifold res = collisionManifold(normal, depth, true);
  res.addContactPoint(contactPoint);
  return res;
}

// Function to detect collision between a circle and a box
collisionManifold collisions::circleToBox(circle& a, box& b) {
  // If the circle and box are not intersecting, they are not colliding
  if (!intersectionDetector::circleBox(a, b)) {
    return collisionManifold(vector2d(0, 0), 0, false);
  }

  // If the circle and box are colliding, calculate the depth of the collision
  // and the contact point
  vector2d circleCentre = a.getCentre();
  vector2d minVec(0, 0);
  vector2d maxVec = b.getHalfSize() * 2;
  vector2d localCircleCentre = (a.getCentre() - b.getRigidBody().getPosition())
                                   .rotate(-b.getRigidBody().getRotation()) +
                               b.getHalfSize();
  vector2d closestPoint =
      vector2d(std::max(minVec.x, std::min(localCircleCentre.x, maxVec.x)),
               std::max(minVec.y, std::min(localCircleCentre.y, maxVec.y)));
  vector2d distance = localCircleCentre - closestPoint;
  distance = distance.rotate(b.getRigidBody().getRotation());
  vector2d normal = distance.normalise();
  double depth = a.getRadius() - distance.magnitude();
  collisionManifold res = collisionManifold(normal, depth, true);
  vector2d contactPoint = (closestPoint + b.getRigidBody().getPosition())
                              .rotate(b.getRigidBody().getRotation());
  res.addContactPoint(contactPoint);
  return res;
}

collisionManifold collisions::boxToBox(box& a, box& b) {
  std::vector<vector2d> axesA = intersectionDetector::getAxes(a);
  std::vector<vector2d> axesB = intersectionDetector::getAxes(b);
  std::vector<vector2d> allAxes = axesA;
  allAxes.insert(allAxes.end(), axesB.begin(), axesB.end());

  double minOverlap = std::numeric_limits<double>::infinity();
  vector2d smallestAxis(0, 0);

  // Find the smallest axis of penetration
  for (const auto& axis : allAxes) {
    vector2d projectionA = intersectionDetector::getInterval(a, axis);
    vector2d projectionB = intersectionDetector::getInterval(b, axis);

    double overlap = std::min(projectionA.y, projectionB.y) -
                     std::max(projectionA.x, projectionB.x);
    // Debug statement
    // std::cout << "Axis: (" << axis.x << ", " << axis.y
    //           << "), Overlap: " << overlap << "\n";

    if (overlap < 0) {
      // No collision
      // std::cout << "No collision on this axis.\n";
      return collisionManifold(vector2d(0, 0), 0, false);
    }

    if (overlap < minOverlap) {
      minOverlap = overlap;
      smallestAxis = axis;

      // Determine the direction of the smallest axis
      vector2d centerA = a.getRigidBody().getPosition();
      vector2d centerB = b.getRigidBody().getPosition();
      vector2d centerVec = centerB - centerA;
      if (centerVec.dot(smallestAxis) < 0) {
        smallestAxis = -smallestAxis;
      }
    }
  }

  std::cout << "Smallest Axis: (" << smallestAxis.x << ", " << smallestAxis.y
            << "), Min Overlap: " << minOverlap << "\n";

  // Reference and Incident Box determination
  box& refBox = a;
  box& incBox = b;
  if (smallestAxis.dot(b.getRigidBody().getPosition() -
                       a.getRigidBody().getPosition()) < 0) {
    std::swap(refBox, incBox);
    smallestAxis = -smallestAxis;
  }

  // Reference face determination
  std::vector<vector2d> refVertices = refBox.getVertices();
  double maxDot = -std::numeric_limits<double>::infinity();
  int refFaceIndex = 0;
  for (int i = 0; i < 4; i++) {
    double dot = refVertices[i].dot(smallestAxis);
    if (dot > maxDot) {
      maxDot = dot;
      refFaceIndex = i;
    }
  }

  vector2d refFaceA = refVertices[refFaceIndex];
  vector2d refFaceB = refVertices[(refFaceIndex + 1) % 4];
  vector2d refEdge = refFaceB - refFaceA;

  // Align refNormal with smallestAxis
  vector2d refNormal = smallestAxis.normalise();

  // Incident face determination
  std::vector<vector2d> incVertices = incBox.getVertices();
  double minDotInc = std::numeric_limits<double>::infinity();
  int incFaceIndex = 0;
  for (int i = 0; i < 4; i++) {
    double dot = incVertices[i].dot(-smallestAxis);
    if (dot < minDotInc) {
      minDotInc = dot;
      incFaceIndex = i;
    }
  }

  vector2d incFaceA = incVertices[incFaceIndex];
  vector2d incFaceB = incVertices[(incFaceIndex + 1) % 4];

  // Clipping process
  auto clip = [&](vector2d n, double c, std::vector<vector2d>& face) -> int {
    std::vector<vector2d> out;
    double d1 = n.dot(face[0]) - c;
    double d2 = n.dot(face[1]) - c;

    std::cout << "Clipping against plane: n=(" << n.x << ", " << n.y
              << "), c=" << c << "\n";
    std::cout << "Point 0: (" << face[0].x << ", " << face[0].y
              << "), d1=" << d1 << "\n";
    std::cout << "Point 1: (" << face[1].x << ", " << face[1].y
              << "), d2=" << d2 << "\n";

    if (d1 >= 0) out.push_back(face[0]);
    if (d2 >= 0) out.push_back(face[1]);

    if (d1 * d2 < 0) {
      double t = d1 / (d1 - d2);
      vector2d intersection = face[0] + (face[1] - face[0]) * t;
      out.push_back(intersection);
      std::cout << "Intersection Point: (" << intersection.x << ", "
                << intersection.y << ")\n";
    }

    face = out;
    std::cout << "Clipped Face Size: " << face.size() << "\n";
    return out.size();
  };

  std::vector<vector2d> clipped = {incFaceA, incFaceB};

  // Clip against reference face side planes
  double refC = refNormal.dot(refFaceA);
  clip(refNormal, refC, clipped);
  if (clipped.empty()) {
    std::cout << "Clipping against reference face A resulted in no points.\n";
    return collisionManifold(vector2d(0, 0), 0, false);
  }

  vector2d refEdgePerp = (refFaceA - refFaceB).perp().normalise();
  double refBC = refEdgePerp.dot(refFaceB);
  clip(-refEdgePerp, -refBC, clipped);
  if (clipped.empty()) {
    std::cout << "Clipping against reference face B resulted in no points.\n";
    return collisionManifold(vector2d(0, 0), 0, false);
  }

  // Filter and add contact points
  std::vector<vector2d> contactPoints;
  double allowedPenetration = 0.01;  // Adjust this value as needed
  for (const auto& p : clipped) {
    double penetration = refNormal.dot(refFaceA - p);
    if (penetration >= -allowedPenetration) {
      contactPoints.push_back(p);
      std::cout << "Added Contact Point: (" << p.x << ", " << p.y
                << "), Penetration: " << penetration << "\n";
    } else {
      std::cout << "Discarded Point: (" << p.x << ", " << p.y
                << "), Penetration: " << penetration << "\n";
    }
  }

  if (contactPoints.empty()) {
    std::cout << "No valid contact points after filtering.\n";
    return collisionManifold(vector2d(0, 0), 0, false);
  }

  // Create collision manifold
  collisionManifold manifold(refNormal, minOverlap, true);
  for (const auto& cp : contactPoints) {
    manifold.addContactPoint(cp);
  }

  std::cout << "Collision Manifold Created with " << contactPoints.size()
            << " contact points.\n";

  return manifold;
}

// Function to find the collision features between two colliders
collisionManifold* collisions::findCollisionFeatures(collider* c1,
                                                     collider* c2) {
  // Check the types of the colliders and call the appropriate collision
  // detection function
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
  if (c1->getType() == "circle" && c2->getType() == "box") {
    circle* a = dynamic_cast<circle*>(c1);
    box* b = dynamic_cast<box*>(c2);
    return new collisionManifold(circleToBox(*a, *b));
  }
  if (c1->getType() == "box" && c2->getType() == "circle") {
    circle* a = dynamic_cast<circle*>(c2);
    box* b = dynamic_cast<box*>(c1);
    return new collisionManifold(circleToBox(*a, *b));
  }

  // If the types of the colliders are not supported, print an error message
  std::cout << "No collision detection for " << c1->getType() << " and "
            << c2->getType() << std::endl;
  return nullptr;
}