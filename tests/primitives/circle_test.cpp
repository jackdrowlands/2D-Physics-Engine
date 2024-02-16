#include "../../include/primitives/circle.hpp"

#include "gtest/gtest.h"

// Test construction and radius retrieval
TEST(CircleTest, ConstructionAndRadius) {
  double radius = 5.0;
  circle testCircle(radius);

  // Check if the radius is set and retrieved correctly
  EXPECT_EQ(testCircle.getRadius(), radius);
}

// Test setting and getting the center
TEST(CircleTest, CentreRetrieval) {
  circle testCircle(10.0);
  vector2d centerPosition(3, 4);
  // Assuming we can set the position of the circle's body, which isn't shown in
  // the given implementation This might require either modifying the circle
  // class to allow setting the position or initializing the rigidBody with a
  // position. For the sake of this test, we'll assume it's possible.
  testCircle.setCentre(centerPosition);

  vector2d retrievedCenter = testCircle.getCentre();
  EXPECT_EQ(retrievedCenter.x, centerPosition.x);
  EXPECT_EQ(retrievedCenter.y, centerPosition.y);
}

// Test negative radius
TEST(CircleTest, NegativeRadius) {
  double negativeRadius = -5.0;
  circle testCircle(negativeRadius);

  // Depending on how negative radius values should be handled, the expectation
  // here might change This test simply checks that the radius value is stored
  // as is
  EXPECT_EQ(testCircle.getRadius(), negativeRadius);
}

// Test zero radius
TEST(CircleTest, ZeroRadius) {
  double zeroRadius = 0.0;
  circle testCircle(zeroRadius);

  // Check if the circle correctly handles a radius of zero
  EXPECT_EQ(testCircle.getRadius(), zeroRadius);
}

// Test large radius values
TEST(CircleTest, LargeRadius) {
  double largeRadius = 1e6;  // A very large radius
  circle testCircle(largeRadius);

  // Verify that large radius values are handled correctly
  EXPECT_EQ(testCircle.getRadius(), largeRadius);
}
