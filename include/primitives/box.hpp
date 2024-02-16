#ifndef BOX_HPP
#define BOX_HPP

#include <vector>

#include "include/datatypes/vector2f.hpp"
#include "include/rigidbody/rigidBody.hpp"

class box {
 private:
  vector2f size;
  rigidBody body;

 public:
  box();
  box(vector2f size);
  box(vector2f min, vector2f max);
  ~box();
  vector2f getMin();
  vector2f getMax();

  std::vector<vector2f> getVertices();
};

#endif  // BOX_HPP