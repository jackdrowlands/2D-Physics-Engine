#ifndef RENDERER_HPP
#define RENDERER_HPP

#include <SFML/Graphics.hpp>

class renderer {
 private:
  sf::RenderWindow window;

 public:
  renderer(unsigned int width, unsigned int height);

  void draw(const sf::Drawable& drawable);

  void clear();

  void display();

  bool isOpen();

  bool pollEvent(sf::Event& event);

  void close();
};

#endif  // RENDERER_HPP