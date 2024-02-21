#ifndef BOX_HPP
#define BOX_HPP

#include <vector>

#include "../../include/datatypes/vector2d.hpp"
#include "../../include/rigidbody/rigidBody.hpp"
#include "../render/renderer.hpp"
#include "collider.hpp"

class box : public collider {
 private:
  vector2d size;
  rigidBody body;

 public:
  box();
  box(vector2d size);
  box(vector2d min, vector2d max);
  ~box();
  vector2d getLocalMin();
  vector2d getLocalMax();
  std::vector<vector2d> getVertices();
  // need to return a reference to the rigidBody
  rigidBody& getRigidBody();
  vector2d getSize();
  vector2d getHalfSize();
  void setRigidBody(rigidBody body);
  void setRigidBody(rigidBody* body);
  void setSize(vector2d size);
  std::string getType();
};

#endif  // BOX_HPP