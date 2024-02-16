#include "../../include/datatypes/line.hpp"

#include "gtest/gtest.h"

// Test the construction of a line and retrieval of its points
TEST(LineTest, ConstructionAndGetters) {
  vector2d start(1, 2);
  vector2d end(3, 4);
  line testLine(start, end);

  // Test getFrom method
  vector2d lineStart = testLine.getFrom();
  EXPECT_EQ(lineStart.x, start.x);
  EXPECT_EQ(lineStart.y, start.y);

  // Test getTo method
  vector2d lineEnd = testLine.getTo();
  EXPECT_EQ(lineEnd.x, end.x);
  EXPECT_EQ(lineEnd.y, end.y);
}

// Test the line with the same start and end point
TEST(LineTest, LineWithIdenticalPoints) {
  vector2d point(5, 5);
  line testLine(point, point);

  // Both getFrom and getTo should return the same point
  EXPECT_EQ(testLine.getFrom().x, point.x);
  EXPECT_EQ(testLine.getFrom().y, point.y);
  EXPECT_EQ(testLine.getTo().x, point.x);
  EXPECT_EQ(testLine.getTo().y, point.y);
}

// Test the line with negative coordinates
TEST(LineTest, LineWithNegativeCoordinates) {
  vector2d start(-1, -2);
  vector2d end(-3, -4);
  line testLine(start, end);

  // Ensure the line correctly handles negative coordinates
  EXPECT_EQ(testLine.getFrom().x, start.x);
  EXPECT_EQ(testLine.getFrom().y, start.y);
  EXPECT_EQ(testLine.getTo().x, end.x);
  EXPECT_EQ(testLine.getTo().y, end.y);
}

// Test the line with large coordinates
TEST(LineTest, LineWithLargeCoordinates) {
  vector2d start(1e9, 1e9);
  vector2d end(-1e9, -1e9);
  line testLine(start, end);

  // Ensure the line can handle large magnitude coordinates
  EXPECT_EQ(testLine.getFrom().x, start.x);
  EXPECT_EQ(testLine.getFrom().y, start.y);
  EXPECT_EQ(testLine.getTo().x, end.x);
  EXPECT_EQ(testLine.getTo().y, end.y);
}
