#include <iostream>

#include "../include/physicsSystem.hpp"
// SFML
#include <SFML/Graphics.hpp>

int main() {
  // SFML window
  sf::RenderWindow window(sf::VideoMode(1000, 1000), "Physics Engine");

  // Physics system
  physicsSystem physics(1.0 / 60.0, vector2d(0, -9.81));

  // Rigid body
  rigidBody* body = new rigidBody({1.0, 1.0}, 0, 1.0);
  body->setPosition(vector2d(400, 300));

  // Add rigid body to physics system
  physics.addRigidBody(body);

  // Main loop
  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }

    // Update physics
    physics.fixedUpdate();

    // Clear window
    window.clear(sf::Color::White);

    // Draw rigid body
    sf::RectangleShape rectangle(sf::Vector2f(10, 10));
    rectangle.setFillColor(sf::Color::Red);
    // correct position
    rectangle.setPosition(body->getPosition().x, body->getPosition().y);
    std::cout << "Position: " << body->getPosition().x << ", "
              << body->getPosition().y << std::endl;
    std::cout << "Rectangle position: " << rectangle.getPosition().x << ", "
              << rectangle.getPosition().y << std::endl;
    window.draw(rectangle);

    // Display window
    window.display();
  }

  return 0;
}