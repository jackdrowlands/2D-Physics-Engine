#include "../../include/datatypes/vector2d.hpp"

#include <gtest/gtest.h>

TEST(Vector2dTest, Constructor) {
  vector2d v(1.0, 2.0);
  EXPECT_EQ(v.x, 1.0);
  EXPECT_EQ(v.y, 2.0);
  // Add default constructor test and edge cases.
}

// Repeat for other vector2d functions
