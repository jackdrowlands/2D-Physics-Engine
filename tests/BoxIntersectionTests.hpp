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

TEST_F(CollisionDetectorTests, PointInBox2DShouldReturnTrueTestOne) {
  box box1;
  box1.setSize(vector2d(10,
                        10));  // Fix: Pass the correct arguments to the
                               // constructor of vector2d.
  rigidBody body1;
  box1.setRigidbody(&body1);
  vector2d point1(4, 4.3);
  EXPECT_TRUE(intersectionDetector::pointInBox2D(point1, box1));

  box box2;
  box2.setSize(vector2d(10));
  rigidBody body2;
  box2.setRigidbody(&body2);
  vector2d point2(-4.9f, -4.9f);
  EXPECT_TRUE(intersectionDetector::pointInBox2D(point2, box2));

  box box3;
  box3.setSize(vector2d(10));
  rigidBody body3;
  box3.setRigidbody(&body3);
  vector2d point3(0, 5.1f);
  EXPECT_FALSE(intersectionDetector::pointInBox2D(point3, box3));

  box box4;
  box4.setSize(vector2d(10));
  rigidBody body4;
  body4.setTransform(vector2d(10));
  box4.setRigidbody(&body4);
  vector2d point4(4 + 10, 4.3f + 10);
  EXPECT_TRUE(intersectionDetector::pointInBox2D(point4, box4));

  box box5;
  box5.setSize(vector2d(10));
  rigidBody body5;
  body5.setTransform(vector2d(10));
  box5.setRigidbody(&body5);
  vector2d point5(-4.9f + 10, -4.9f + 10);
  EXPECT_TRUE(intersectionDetector::pointInBox2D(point5, box5));

  box box6;
  box6.setSize(vector2d(10));
  rigidBody body6;
  body6.setTransform(vector2d(10));
  box6.setRigidbody(&body6);
  vector2d point6(0 + 10, 5.1f + 10);
  EXPECT_FALSE(intersectionDetector::pointInBox2D(point6, box6));
}