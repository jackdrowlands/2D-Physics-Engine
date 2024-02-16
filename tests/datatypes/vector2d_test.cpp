#include "../../include/datatypes/vector2d.hpp"

#include "gtest/gtest.h"

// Test constructors
TEST(Vector2dTest, Constructors) {
  vector2d defaultVec;
  EXPECT_EQ(defaultVec.x, 0);
  EXPECT_EQ(defaultVec.y, 0);

  vector2d customVec(5.5, -3.3);
  EXPECT_EQ(customVec.x, 5.5);
  EXPECT_EQ(customVec.y, -3.3);
}

// Test operator overloads for arithmetic
TEST(Vector2dTest, ArithmeticOperators) {
  vector2d vec1(1, 2);
  vector2d vec2(3, 4);

  // Test addition
  vector2d sum = vec1 + vec2;
  EXPECT_EQ(sum.x, 4);
  EXPECT_EQ(sum.y, 6);

  // Test subtraction
  vector2d diff = vec2 - vec1;
  EXPECT_EQ(diff.x, 2);
  EXPECT_EQ(diff.y, 2);

  // Test multiplication by scalar
  vector2d scaled = vec1 * 2;
  EXPECT_EQ(scaled.x, 2);
  EXPECT_EQ(scaled.y, 4);

  // Test division by scalar
  vector2d div = vec2 / 2;
  EXPECT_EQ(div.x, 1.5);
  EXPECT_EQ(div.y, 2);
}

// Test operator overloads for assignment arithmetic
TEST(Vector2dTest, AssignmentArithmeticOperators) {
  vector2d vec(3, 4);

  // Test addition assignment
  vec += vector2d(1, 1);
  EXPECT_EQ(vec.x, 4);
  EXPECT_EQ(vec.y, 5);

  // Test subtraction assignment
  vec -= vector2d(1, 1);
  EXPECT_EQ(vec.x, 3);
  EXPECT_EQ(vec.y, 4);

  // Test multiplication assignment
  vec *= 2;
  EXPECT_EQ(vec.x, 6);
  EXPECT_EQ(vec.y, 8);

  // Test division assignment
  vec /= 2;
  EXPECT_EQ(vec.x, 3);
  EXPECT_EQ(vec.y, 4);
}

// Test comparison operators
TEST(Vector2dTest, ComparisonOperators) {
  vector2d vec1(1, 2);
  vector2d vec2(1, 2);
  vector2d vec3(2, 3);

  // Test equality
  EXPECT_TRUE(vec1 == vec2);
  EXPECT_FALSE(vec1 == vec3);

  // Test inequality
  EXPECT_TRUE(vec1 != vec3);
  EXPECT_FALSE(vec1 != vec2);
}

// Test magnitude and normalization
TEST(Vector2dTest, MagnitudeAndNormalization) {
  vector2d vec(3, 4);

  // Test magnitude
  EXPECT_EQ(vec.magnitude(), 5);

  // Test normalization
  vector2d normalized = vec.normalise();
  EXPECT_NEAR(normalized.x, 0.6, 1e-9);
  EXPECT_NEAR(normalized.y, 0.8, 1e-9);
}

// Test utility methods such as dot, cross, distance, etc.
TEST(Vector2dTest, UtilityMethods) {
  vector2d vec1(1, 0);
  vector2d vec2(0, 1);

  // Test dot product
  EXPECT_EQ(vec1.dot(vec2), 0);

  // Test cross product
  EXPECT_EQ(vec1.cross(vec2), 1);

  // Test distance
  EXPECT_EQ(vec1.distance(vec2), std::sqrt(2));

  // Test angle
  EXPECT_NEAR(vec1.angle(vec2), M_PI / 2, 1e-9);

  // Test projection
  vector2d projected = vec1.project(vec2);
  EXPECT_EQ(projected.x, 0);
  EXPECT_EQ(projected.y, 0);

  // Test reflection
  vector2d reflected = vec1.reflect(vec2);
  EXPECT_EQ(reflected.x, 1);
  EXPECT_EQ(reflected.y, 0);

  // Test rotation
  vector2d rotated = vec1.rotate(M_PI / 2);
  EXPECT_NEAR(rotated.x, 0, 1e-9);
  EXPECT_NEAR(rotated.y, 1, 1e-9);
}

// Note: This is a basic set of tests. Depending on the requirements,
// you may need to write more tests to cover all edge cases and ensure
// the correctness of all methods, especially for methods like `lerp`, `slerp`,
// and various rotation operations.

// Test lerp
TEST(Vector2dTest, Lerp) {
  vector2d vec1(1, 1);
  vector2d vec2(3, 3);

  // Test lerp
  vector2d lerped = vec1.lerp(vec2, 0.5);
  EXPECT_EQ(lerped.x, 2);
  EXPECT_EQ(lerped.y, 2);
}

// Test slerp
TEST(Vector2dTest, Slerp) {
  vector2d vec1(1, 0);
  vector2d vec2(0, 1);

  // Test slerp
  vector2d slerped = vec1.slerp(vec2, 0.5);
  EXPECT_NEAR(slerped.x, 0.70710678118, 1e-9);
  EXPECT_NEAR(slerped.y, 0.70710678118, 1e-9);
}

// Test rotate
TEST(Vector2dTest, Rotate) {
  vector2d vec(1, 0);

  // Test rotate
  vector2d rotated = vec.rotate(M_PI / 2);
  EXPECT_NEAR(rotated.x, 0, 1e-9);
  EXPECT_NEAR(rotated.y, 1, 1e-9);
}

// Test rotate with origin
TEST(Vector2dTest, RotateWithOrigin) {
  vector2d vec(1, 0);
  vector2d origin(1, 1);

  // Test rotate with origin
  vector2d rotated = vec.rotate(M_PI / 2, origin);
  EXPECT_NEAR(rotated.x, 2, 1e-9);
  EXPECT_NEAR(rotated.y, 1, 1e-9);
}

// Test perp
TEST(Vector2dTest, Perp) {
  vector2d vec(1, 2);

  // Test perp
  vector2d perp = vec.perp();
  EXPECT_EQ(perp.x, -2);
  EXPECT_EQ(perp.y, 1);
}

// Test destructor
TEST(Vector2dTest, Destructor) {
  vector2d* vec = new vector2d(1, 2);
  delete vec;
}

// Test friend functions
TEST(Vector2dTest, FriendFunctions) {
  vector2d vec(1, 2);

  // Test friend functions
  vector2d scaled = 2 * vec;
  EXPECT_EQ(scaled.x, 2);
  EXPECT_EQ(scaled.y, 4);

  vector2d div = 2 / vec;
  EXPECT_EQ(div.x, 0.5);
  EXPECT_EQ(div.y, 1);
}

// Test abs
TEST(Vector2dTest, Abs) {
  vector2d vec(-1, -2);

  // Test abs
  vector2d absVec = abs(vec);
  EXPECT_EQ(absVec.x, 1);
  EXPECT_EQ(absVec.y, 2);
}
