#include <iostream>

#include "../include/physicsSystem.hpp"
#include "../include/primitives/circle.hpp"
#include "../include/render/renderer.hpp"

int main() {
  // Renderer
  renderer renderer(1000, 1000);

  // Physics
  physicsSystem physics(1.0 / 60.0, vector2d(0, 9.8));

  std::vector<circle*> bodies;
  // Rigid body
  circle* body1 = new circle(10, vector2d(500, 0));
  circle* body2 = new circle(20, vector2d(500, 500));
  circle* body3 = new circle(5, vector2d(500, 200));

  body1->getRigidBody().setMass(1);
  body2->getRigidBody().setMass(1000);
  body3->getRigidBody().setMass(2);
  body1->getRigidBody().setCollider(body1);
  body2->getRigidBody().setCollider(body2);
  body3->getRigidBody().setCollider(body3);

  physics.addRigidBody(&body1->getRigidBody(), true);
  physics.addRigidBody(&body2->getRigidBody(), false);
  physics.addRigidBody(&body3->getRigidBody(), true);

  bodies.push_back(body1);
  bodies.push_back(body2);
  bodies.push_back(body3);

  // Main loop
  while (renderer.isOpen()) {
    sf::Event event;
    while (renderer.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        renderer.close();
      }
    }

    physics.fixedUpdate();

    renderer.clear();

    // Draw all rigid bodies -> circles
    for (auto& body : bodies) {
      sf::CircleShape circle(body->getRadius());
      circle.setPosition(body->getCentre().x - body->getRadius(),
                         body->getCentre().y - body->getRadius());
      circle.setFillColor(sf::Color::Red);
      renderer.draw(circle);
    }

    renderer.display();
  }

  return 0;
}