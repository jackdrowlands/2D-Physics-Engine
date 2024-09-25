#include <iostream>

#include "../include/physicsSystem.hpp"
#include "../include/primitives/box.hpp"
#include "../include/primitives/circle.hpp"
#include "../include/render/renderer.hpp"

struct BodyConfig {
  vector2d size;
  double mass;
  vector2d position;
  double rotation;
  bool affectedByGravity;
};

struct CircleConfig {
  double radius;
  double mass;
  vector2d position;
  bool affectedByGravity;
};

void createBodies(std::vector<box*>& bodies, physicsSystem& physics) {
  std::vector<BodyConfig> configs = {
      {vector2d{10, 900}, 100, vector2d{950, 500}, 0.0, false},
      {vector2d{10, 900}, 100, vector2d{50, 500}, 0.0, false},
      {vector2d{800, 10}, 100000, vector2d{500, 500}, 0.05, false},
      {vector2d{10, 10}, 1, vector2d{510, 0}, 0.0, true},
      {vector2d{10, 10}, 1, vector2d{500, 200}, 0.0, true}};
  for (const auto& config : configs) {
    // Create and initialize the box
    auto body = new box(config.size);
    body->getRigidBody().setMass(config.mass);
    body->getRigidBody().setPosition(config.position);
    body->getRigidBody().setRotation(config.rotation);

    // Set the collider to itself or as required
    body->getRigidBody().setCollider(body);

    // Add the body to the physics system based on gravity flag
    physics.addRigidBody(&body->getRigidBody(), config.affectedByGravity);

    // Add the body to the bodies vector
    bodies.emplace_back(std::move(body));
  }
}

void createCircles(std::vector<circle*>& circles, physicsSystem& physics) {
  std::vector<CircleConfig> configs = {};
  for (const auto& config : configs) {
    // Create and initialize the circle
    auto circle = new class circle(config.radius);
    circle->getRigidBody().setMass(config.mass);
    circle->getRigidBody().setPosition(config.position);

    // Set the collider to itself or as required
    circle->getRigidBody().setCollider(circle);

    // Add the body to the physics system based on gravity flag
    physics.addRigidBody(&circle->getRigidBody(), config.affectedByGravity);

    // Add the body to the bodies vector
    circles.emplace_back(std::move(circle));
  }
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
  // createCircles(circles, physics);

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
    // drawCircles(circles, renderer);

    renderer.display();
  }

  return 0;
}