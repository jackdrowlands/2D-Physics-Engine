#ifndef COLLIDER_HPP
#define COLLIDER_HPP

#include "../datatypes/vector2d.hpp"
#include "../render/renderer.hpp"

class collider {
 private:
  /* data */
 protected:
  vector2d offset;

 public:
  collider(/* args */);
  ~collider();
  virtual std::string getType() = 0;
};

#endif  // COLLIDER_HPP