#include <iostream>

#include "../include/physicsSystem.hpp"
#include "../include/render/renderer.hpp"

int main() {
  // Renderer
  renderer renderer(1000, 1000);

  // Physics
  physicsSystem physics(1.0 / 60.0, vector2d(0, 9.8));

  // Rigid body
  rigidBody body(vector2d(500, 500), 0, 1);

  // Add rigid body to physics system
  physics.addRigidBody(&body);

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

    // Draw rigid body
    sf::CircleShape circle(10);
    circle.setFillColor(sf::Color::Red);
    circle.setPosition(body.getPosition().x, body.getPosition().y);
    renderer.draw(circle);

    renderer.display();
  }

  return 0;
}