#include <gtest/gtest.h>

#include "../include/utils2D.hpp"

TEST(UtilsTest, CompareDouble) {
  utils2D u;
  EXPECT_TRUE(u.compare(1.0, 1.0));
  EXPECT_FALSE(u.compare(1.0, 2.0));
  EXPECT_TRUE(u.compare(1.0, 1.00001, 0.001));
  EXPECT_FALSE(u.compare(1.0, 1.002, 0.001));
}

TEST(UtilsTest, CompareVector2d) {
  utils2D u;
  EXPECT_TRUE(u.compare(vector2d(1.0, 1.0), vector2d(1.0, 1.0)));
  EXPECT_FALSE(u.compare(vector2d(1.0, 1.0), vector2d(2.0, 2.0)));
  EXPECT_TRUE(u.compare(vector2d(1.0, 1.0001), vector2d(1.0, 1.0002), 0.001));
  EXPECT_FALSE(u.compare(vector2d(1.0, 1.0), vector2d(1.0, 1.01), 0.001));
}
