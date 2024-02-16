#include "../../include/rigidbody/rigidBody.hpp"

#include "gtest/gtest.h"

// Test default construction
TEST(RigidBodyTest, DefaultConstruction) {
  rigidBody body;
  EXPECT_EQ(body.getPosition().x, 0);
  EXPECT_EQ(body.getPosition().y, 0);
  EXPECT_EQ(body.getRotation(), 0.0);
}

// Test setting and getting position
TEST(RigidBodyTest, SetGetPosition) {
  rigidBody body;
  vector2d newPosition(5, -3);
  body.setPosition(newPosition);

  EXPECT_EQ(body.getPosition().x, newPosition.x);
  EXPECT_EQ(body.getPosition().y, newPosition.y);
}

// Test setting and getting rotation
TEST(RigidBodyTest, SetGetRotation) {
  rigidBody body;
  double newRotation = M_PI / 4;  // 45 degrees in radians
  body.setRotation(newRotation);

  EXPECT_EQ(body.getRotation(), newRotation);
}

// Test negative rotation values
TEST(RigidBodyTest, NegativeRotation) {
  rigidBody body;
  double negativeRotation = -M_PI / 2;  // -90 degrees in radians
  body.setRotation(negativeRotation);

  EXPECT_EQ(body.getRotation(), negativeRotation);
}

// Test large rotation values beyond 2*PI
TEST(RigidBodyTest, LargeRotationValues) {
  rigidBody body;
  double largeRotation =
      3 * M_PI;  // 540 degrees in radians, beyond a full rotation
  body.setRotation(largeRotation);

  // Depending on how you wish to handle rotations beyond full circles, adjust
  // expectations accordingly This test assumes rotations are not normalized to
  // [0, 2*PI)
  EXPECT_EQ(body.getRotation(), largeRotation);
}
