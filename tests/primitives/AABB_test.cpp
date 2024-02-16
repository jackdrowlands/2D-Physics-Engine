#include "../../include/primitives/AABB.hpp"

#include "gtest/gtest.h"

// Test default constructor
TEST(AABBTest, DefaultConstructor) {
  AABB box;
  EXPECT_EQ(box.getMin(), vector2d(0, 0));
  EXPECT_EQ(box.getMax(), vector2d(0, 0));
}

// Test constructor with size and body
TEST(AABBTest, ConstructorWithSizeAndBody) {
  vector2d size(10, 20);
  rigidBody body;
  body.setPosition(vector2d(
      15, 25));  // Assuming setPosition exists for the purpose of this test
  AABB box(size, body);

  EXPECT_EQ(box.getMin(), vector2d(10, 15));
  EXPECT_EQ(box.getMax(), vector2d(20, 35));
}

// Test constructor with min and max vectors
TEST(AABBTest, ConstructorWithMinMax) {
  vector2d min(5, 10);
  vector2d max(15, 30);
  AABB box(min, max);
  EXPECT_EQ(box.getMin(), min);
  EXPECT_EQ(box.getMax(), max);
}

// Test getMin and getMax for edge case (zero size)
TEST(AABBTest, ZeroSize) {
  vector2d size(0, 0);
  rigidBody body;
  body.setPosition(vector2d(5, 5));  // Assuming setPosition exists
  AABB box(size, body);

  EXPECT_EQ(box.getMin(), vector2d(5, 5));
  EXPECT_EQ(box.getMax(), vector2d(5, 5));
}