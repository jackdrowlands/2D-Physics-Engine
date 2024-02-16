// FILEPATH: /home/jack/2D-Physics-Engine/tests/datatypes/line_test.cpp
#include "../../include/datatypes/line.hpp"

#include "../../include/datatypes/vector2d.hpp"
#include "gtest/gtest.h"

class LineTest : public ::testing::Test {
 protected:
  vector2d from = vector2d(1.0, 2.0);
  vector2d to = vector2d(3.0, 4.0);
  line testLine = line(from, to);
};

TEST_F(LineTest, ConstructorTest) {
  EXPECT_EQ(testLine.getFrom(), from);
  EXPECT_EQ(testLine.getTo(), to);
}

TEST_F(LineTest, GetFromTest) { EXPECT_EQ(testLine.getFrom(), from); }

TEST_F(LineTest, GetToTest) { EXPECT_EQ(testLine.getTo(), to); }