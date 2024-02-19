#include <iostream>

#include "../include/physicsSystem.hpp"
#include "../include/primitives/box.hpp"
#include "../include/primitives/circle.hpp"
#include "../include/render/renderer.hpp"

int main() {
  // Renderer
  renderer renderer(1000, 1000);

  // Physics
  physicsSystem physics(1.0 / 60.0, vector2d(0, 9.8));

  std::vector<box*> bodies;
  // Rigid body
  box* body1 = new box(vector2d(10, 10));
  box* body2 = new box(vector2d(10, 10));
  box* body3 = new box(vector2d(20, 10));

  body1->getRigidBody().setMass(1);
  body1->getRigidBody().setPosition(vector2d(510, 0));
  body2->getRigidBody().setMass(3);
  body2->getRigidBody().setPosition(vector2d(500, 200));
  body3->getRigidBody().setMass(100000);
  body3->getRigidBody().setPosition(vector2d(500, 500));
  body1->getRigidBody().setCollider(body1);
  body2->getRigidBody().setCollider(body2);
  body3->getRigidBody().setCollider(body3);

  physics.addRigidBody(&body1->getRigidBody(), true);
  physics.addRigidBody(&body2->getRigidBody(), true);
  physics.addRigidBody(&body3->getRigidBody(), false);

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

    // Draw all rigid bodies -> box
    for (auto& body : bodies) {
      sf::RectangleShape rectangle(
          sf::Vector2f(body->getSize().x, body->getSize().y));
      rectangle.setPosition(body->getRigidBody().getPosition().x,
                            body->getRigidBody().getPosition().y);
      rectangle.setOrigin(body->getSize().x / 2, body->getSize().y / 2);
      rectangle.setRotation(body->getRigidBody().getRotation());
      rectangle.setFillColor(sf::Color::Green);
      renderer.draw(rectangle);
    }

    renderer.display();
  }

  return 0;
}