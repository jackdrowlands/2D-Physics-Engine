#include "../../include/rigidbody/intersectionDetector.hpp"

#include "gtest/gtest.h"

// Helper function to create a line
line makeLine(double x1, double y1, double x2, double y2) {
  return line(vector2d(x1, y1), vector2d(x2, y2));
}

// Helper function to create a circle
circle makeCircle(double x, double y, double radius) {
  circle c(radius);
  c.setCentre(
      vector2d(x, y));  // Assuming we can set the circle's position directly
  return c;
}

// Helper function to create an AABB
AABB makeAABB(double minX, double minY, double maxX, double maxY) {
  return AABB(vector2d(minX, minY), vector2d(maxX, maxY));
}

// Test point on line detection
TEST(IntersectionDetectorTest, PointOnLine) {
  line l = makeLine(0, 0, 10, 10);
  EXPECT_TRUE(intersectionDetector::pointOnLine(vector2d(5, 5), l));
  EXPECT_FALSE(intersectionDetector::pointOnLine(vector2d(5, 6), l));
}

// Test point in circle detection
TEST(IntersectionDetectorTest, PointInCircle) {
  circle c = makeCircle(0, 0, 5);
  EXPECT_TRUE(intersectionDetector::pointInCircle(vector2d(3, 4), c));
  EXPECT_FALSE(intersectionDetector::pointInCircle(vector2d(6, 0), c));
}

// Test point in AABB detection
TEST(IntersectionDetectorTest, PointInAABB) {
  AABB aabb = makeAABB(-5, -5, 5, 5);
  EXPECT_TRUE(intersectionDetector::pointInAABB(vector2d(0, 0), aabb));
  EXPECT_FALSE(intersectionDetector::pointInAABB(vector2d(6, 6), aabb));
}

// Test point in box detection
TEST(IntersectionDetectorTest, PointInBox) {
  box b = box(vector2d(0, 0), vector2d(5, 5));
  b.getRigidBody().setPosition(
      vector2d(5, 5));  // Assuming we can set the box's position directly
  b.getRigidBody().setRotation(M_PI / 4);  // 45 degrees in radians
  EXPECT_TRUE(intersectionDetector::pointInBox(vector2d(5, 5), b));
  EXPECT_FALSE(intersectionDetector::pointInBox(vector2d(10, 10), b));
}

// Test line-circle intersection detection
TEST(IntersectionDetectorTest, LineCircleIntersection) {
  circle c = makeCircle(0, 0, 5);
  line l1 = makeLine(-10, 0, 10, 0);     // Line through circle's center
  line l2 = makeLine(-10, -10, 10, 10);  // Line touching circle's edge
  line l3 = makeLine(-10, -10, -5, -5);  // Line outside circle

  EXPECT_TRUE(intersectionDetector::lineCircle(l1, c));
  EXPECT_TRUE(intersectionDetector::lineCircle(l2, c));
  EXPECT_FALSE(intersectionDetector::lineCircle(l3, c));
}

// Test line-AABB intersection detection
TEST(IntersectionDetectorTest, LineAABBIntersection) {
  AABB aabb = makeAABB(-5, -5, 5, 5);
  line l1 = makeLine(0, 0, 10, 10);      // Line through AABB's center
  line l2 = makeLine(-10, -10, 0, 0);    // Line touching AABB's edge
  line l3 = makeLine(-10, -10, -6, -5);  // Line outside AABB

  EXPECT_TRUE(intersectionDetector::lineAABB(l1, aabb));
  EXPECT_TRUE(intersectionDetector::lineAABB(l2, aabb));
  EXPECT_FALSE(intersectionDetector::lineAABB(l3, aabb));
}

// Test raycast-circle intersection detection

TEST(IntersectionDetectorTest, RaycastCircleIntersection) {
  circle c = makeCircle(0, 0, 5);
  ray r1 =
      ray(vector2d(-10, 0), vector2d(1, 0));  // Ray through circle's center
  ray r2 = ray(vector2d(-10, -10),
               vector2d(1, 1).normalise());  // Ray touching circle's edge
  ray r3 = ray(vector2d(-10, -10),
               vector2d(-1, -1).normalise());  // Ray outside circle

  EXPECT_TRUE(intersectionDetector::raycast(r1, c));
  EXPECT_TRUE(intersectionDetector::raycast(r2, c));
  EXPECT_FALSE(intersectionDetector::raycast(r3, c));
}

// Test raycast-AABB intersection detection
TEST(IntersectionDetectorTest, RaycastAABBIntersection) {
  AABB aabb = makeAABB(-5, -5, 5, 5);
  ray r1 =
      ray(vector2d(0, 0), vector2d(1, 1).normalise());  // Ray through AABB's
  // center
  ray r2 = ray(vector2d(-10, -10),
               vector2d(1, 1).normalise());  // Ray touching AABB's edge
  ray r3 = ray(vector2d(-10, -10),
               vector2d(-1, -1).normalise());  // Ray outside AABB

  EXPECT_TRUE(intersectionDetector::raycast(r1, aabb));
  EXPECT_TRUE(intersectionDetector::raycast(r2, aabb));
  EXPECT_FALSE(intersectionDetector::raycast(r3, aabb));
}

// Test raycast-box intersection detection
TEST(IntersectionDetectorTest, RaycastBoxIntersection) {
  box b = box(vector2d(0, 0), vector2d(5, 5));
  b.getRigidBody().setPosition(
      vector2d(5, 5));  // Assuming we can set the box's position directly
  b.getRigidBody().setRotation(M_PI / 4);  // 45 degrees in radians
  ray r1 =
      ray(vector2d(5, 5), vector2d(1, 1).normalise());  // Ray through box's
  // center
  ray r2 = ray(vector2d(-10, -10),
               vector2d(1, 1).normalise());  // Ray touching box's edge
  ray r3 =
      ray(vector2d(-10, -10), vector2d(-1, -1).normalise());  // Ray outside box

  EXPECT_TRUE(intersectionDetector::raycast(r1, b));
  EXPECT_TRUE(intersectionDetector::raycast(r2, b));
  EXPECT_FALSE(intersectionDetector::raycast(r3, b));
}

// Test raycast-circle intersection detection with result
TEST(IntersectionDetectorTest, RaycastCircleIntersectionResult) {
  circle c = makeCircle(0, 0, 5);
  ray r1 =
      ray(vector2d(-10, 0), vector2d(1, 0));  // Ray through circle's center
  ray r2 = ray(vector2d(-10, -10),
               vector2d(1, 1).normalise());  // Ray touching circle's edge
  ray r3 = ray(vector2d(-10, -10),
               vector2d(-1, -1).normalise());  // Ray outside circle
  raycastResult result;

  EXPECT_TRUE(intersectionDetector::raycast(r1, c, result));
  EXPECT_TRUE(intersectionDetector::raycast(r2, c, result));
  EXPECT_FALSE(intersectionDetector::raycast(r3, c, result));
}

// Test raycast-AABB intersection detection with result
TEST(IntersectionDetectorTest, RaycastAABBIntersectionResult) {
  AABB aabb = makeAABB(-5, -5, 5, 5);
  ray r1 =
      ray(vector2d(0, 0), vector2d(1, 1).normalise());  // Ray through AABB's
  // center
  ray r2 = ray(vector2d(-10, -10),
               vector2d(1, 1).normalise());  // Ray touching AABB's edge
  ray r3 = ray(vector2d(-10, -10),
               vector2d(-1, -1).normalise());  // Ray outside AABB
  raycastResult result;

  EXPECT_TRUE(intersectionDetector::raycast(r1, aabb, result));
  EXPECT_TRUE(intersectionDetector::raycast(r2, aabb, result));
  EXPECT_FALSE(intersectionDetector::raycast(r3, aabb, result));
}

// Test raycast-box intersection detection with result
TEST(IntersectionDetectorTest, RaycastBoxIntersectionResult) {
  box b = box(vector2d(0, 0), vector2d(5, 5));
  b.getRigidBody().setPosition(
      vector2d(5, 5));  // Assuming we can set the box's position directly
  b.getRigidBody().setRotation(M_PI / 4);  // 45 degrees in radians
  ray r1 =
      ray(vector2d(5, 5), vector2d(1, 1).normalise());  // Ray through box's
  // center
  ray r2 = ray(vector2d(-10, -10),
               vector2d(1, 1).normalise());  // Ray touching box's edge
  ray r3 =
      ray(vector2d(-15, -15), vector2d(-1, -1).normalise());  // Ray outside box
  raycastResult result;

  EXPECT_TRUE(intersectionDetector::raycast(r1, b, result));
  EXPECT_TRUE(intersectionDetector::raycast(r2, b, result));
  EXPECT_FALSE(intersectionDetector::raycast(r3, b, result));
}
