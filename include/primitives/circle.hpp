#ifndef CIRCLE_HPP
#define CIRCLE_HPP

class circle {
 private:
  float radius;

 public:
  circle(float);
  ~circle();

  float get_radius();
};

#endif  // CIRCLE_HPP