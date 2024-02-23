#include <iostream>

#include "../include/physicsSystem.hpp"
#include "../include/primitives/box.hpp"
#include "../include/primitives/circle.hpp"
#include "../include/render/renderer.hpp"

void createBodies(std::vector<box*>& bodies, physicsSystem& physics) {
  box* body1 = new box(vector2d(10, 900));
  box* body2 = new box(vector2d(10, 900));
  box* body3 = new box(vector2d(800, 10));

  body1->getRigidBody().setMass(100);
  body1->getRigidBody().setPosition(vector2d(950, 500));
  body2->getRigidBody().setMass(100);
  body2->getRigidBody().setPosition(vector2d(50, 500));
  body3->getRigidBody().setMass(100000);
  body3->getRigidBody().setPosition(vector2d(500, 500));
  body3->getRigidBody().setRotation(0.05);
  body1->getRigidBody().setCollider(body1);
  body2->getRigidBody().setCollider(body2);
  body3->getRigidBody().setCollider(body3);

  physics.addRigidBody(&body1->getRigidBody(), false);
  physics.addRigidBody(&body2->getRigidBody(), false);
  physics.addRigidBody(&body3->getRigidBody(), false);

  bodies.emplace_back(body1);
  bodies.emplace_back(body2);
  bodies.emplace_back(body3);
}

void createCircles(std::vector<circle*>& circles, physicsSystem& physics) {
  circle* circle1 = new circle(10);
  circle* circle2 = new circle(10);

  circle1->getRigidBody().setMass(1);
  circle1->getRigidBody().setPosition(vector2d(510, 0));
  circle2->getRigidBody().setMass(1);
  circle2->getRigidBody().setPosition(vector2d(500, 200));
  circle1->getRigidBody().setCollider(circle1);
  circle2->getRigidBody().setCollider(circle2);

  physics.addRigidBody(&circle1->getRigidBody(), true);
  physics.addRigidBody(&circle2->getRigidBody(), true);

  circles.emplace_back(circle1);
  circles.emplace_back(circle2);
}

void drawBodies(const std::vector<box*>& bodies, renderer& renderer) {
  for (const auto& body : bodies) {
    sf::RectangleShape rectangle(
        sf::Vector2f(body->getSize().x, body->getSize().y));
    rectangle.setPosition(body->getRigidBody().getPosition().x,
                          body->getRigidBody().getPosition().y);
    rectangle.setOrigin(body->getSize().x / 2, body->getSize().y / 2);
    rectangle.setRotation(body->getRigidBody().getRotation() * 180 / M_PI);
    rectangle.setFillColor(sf::Color(255, 0, 0));
    renderer.draw(rectangle);
  }
}

void drawCircles(const std::vector<circle*>& circles, renderer& renderer) {
  for (const auto& body : circles) {
    sf::CircleShape circle(body->getRadius());
    circle.setPosition(body->getRigidBody().getPosition().x,
                       body->getRigidBody().getPosition().y);
    circle.setOrigin(body->getRadius(), body->getRadius());
    circle.setFillColor(sf::Color(0, 255, 0));
    renderer.draw(circle);
  }
}

int main() {
  renderer renderer(1000, 1000);
  physicsSystem physics(1.0 / 60.0, vector2d(0, 9.8));

  std::vector<box*> bodies;
  createBodies(bodies, physics);

  std::vector<circle*> circles;
  createCircles(circles, physics);

  while (renderer.isOpen()) {
    sf::Event event;
    while (renderer.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        renderer.close();
      }
    }

    physics.fixedUpdate();
    renderer.clear();

    drawBodies(bodies, renderer);
    drawCircles(circles, renderer);

    renderer.display();
  }

  return 0;
}