#include "../include/utils.hpp"

#include <gtest/gtest.h>

TEST(UtilsTest, CompareDouble) {
  EXPECT_TRUE(utils::compare(1.0, 1.0));
  EXPECT_FALSE(utils::compare(1.0, 2.0));
  EXPECT_TRUE(utils::compare(1.0, 1.00001, 0.001));
  EXPECT_FALSE(utils::compare(1.0, 1.002, 0.001));
}

TEST(UtilsTest, CompareVector2d) {
  EXPECT_TRUE(utils::compare(vector2d(1.0, 1.0), vector2d(1.0, 1.0)));
  EXPECT_FALSE(utils::compare(vector2d(1.0, 1.0), vector2d(2.0, 2.0)));
  EXPECT_TRUE(
      utils::compare(vector2d(1.0, 1.0001), vector2d(1.0, 1.0002), 0.001));
  EXPECT_FALSE(utils::compare(vector2d(1.0, 1.0), vector2d(1.0, 1.01), 0.001));
}
