#include <gtest/gtest.h>

#include "../include/datatypes/line.hpp"
#include "../include/datatypes/vector2d.hpp"
#include "../include/rigidbody/intersectionDetector.hpp"
#include "../include/rigidbody/rigidBody.hpp"
#include "../include/utils.hpp"

class CollisionDetectorTests : public ::testing::Test {
 protected:
  const double EPSILON = 0.000001;

  void SetUp() override {
    {
      // Setup code if needed
    }
  }

  void TearDown() override {
    {
      // Teardown code if needed
    }
  }
};

TEST_F(CollisionDetectorTests, PointOnLine2DShouldReturnTrue) {
  line line1(vector2d(0, 0), vector2d(12, 4));
  vector2d point1(0, 0);
  EXPECT_TRUE(intersectionDetector::pointOnLine(point1, line1));

  line line2(vector2d(0, 0), vector2d(12, 4));
  vector2d point2(12, 4);
  EXPECT_TRUE(intersectionDetector::pointOnLine(point2, line2));

  line line3(vector2d(0, 0), vector2d(0, 10));
  vector2d point3(0, 5);
  EXPECT_TRUE(intersectionDetector::pointOnLine(point3, line3));

  line line4(vector2d(0, 0), vector2d(12, 4));
  vector2d point4(6, 2);
  EXPECT_TRUE(intersectionDetector::pointOnLine(point4, line4));

  line line5(vector2d(0, 0), vector2d(12, 4));
  vector2d point5(4, 2);
  EXPECT_FALSE(intersectionDetector::pointOnLine(point5, line5));

  line line6(vector2d(10, 10), vector2d(22, 14));
  vector2d point6(16, 12);
  EXPECT_TRUE(intersectionDetector::pointOnLine(point6, line6));

  line line7(vector2d(10, 10), vector2d(22, 14));
  vector2d point7(14, 12);
  EXPECT_FALSE(intersectionDetector::pointOnLine(point7, line7));
}
