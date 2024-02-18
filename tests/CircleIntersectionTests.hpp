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

TEST_F(CollisionDetectorTests, PointInCircleShouldReturnTrueTestOne) {
  circle circle1;
  circle1.setRadius(5);
  rigidBody body1;
  circle1.setRigidbody(&body1);
  vector2d point1(3, -2);
  EXPECT_TRUE(intersectionDetector::pointInCircle(point1, circle1));

  circle circle2;
  circle2.setRadius(5);
  rigidBody body2;
  circle2.setRigidbody(&body2);
  vector2d point2(-4.9, 0);
  EXPECT_TRUE(intersectionDetector::pointInCircle(point2, circle2));

  circle circle3;
  circle3.setRadius(5);
  rigidBody body3;
  circle3.setRigidbody(&body3);
  vector2d point3(-6, -6);
  EXPECT_FALSE(intersectionDetector::pointInCircle(point3, circle3));

  circle circle4;
  circle4.setRadius(5);
  rigidBody body4;
  body4.setTransform(vector2d(10));
  circle4.setRigidbody(&body4);
  vector2d point4(3 + 10, -2 + 10);
  EXPECT_TRUE(intersectionDetector::pointInCircle(point4, circle4));

  circle circle5;
  circle5.setRadius(5);
  rigidBody body5;
  body5.setTransform(vector2d(10));
  circle5.setRigidbody(&body5);
  vector2d point5(-4.9f + 10, 0 + 10);
  EXPECT_TRUE(intersectionDetector::pointInCircle(point5, circle5));

  circle circle6;
  circle6.setRadius(5);
  rigidBody body6;
  body6.setTransform(vector2d(10));
  circle6.setRigidbody(&body6);
  vector2d point6(-6 + 10, -6 + 10);
  EXPECT_FALSE(intersectionDetector::pointInCircle(point6, circle6));
}