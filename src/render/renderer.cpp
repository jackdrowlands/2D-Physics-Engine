#include "../../include/render/renderer.hpp"

renderer::renderer(unsigned int width, unsigned int height)
    : window(sf::VideoMode(width, height), "Physics Engine") {}

void renderer::draw(const sf::Drawable& drawable) { window.draw(drawable); }

void renderer::clear() { window.clear(sf::Color::White); }

void renderer::display() { window.display(); }

bool renderer::isOpen() { return window.isOpen(); }

bool renderer::pollEvent(sf::Event& event) { return window.pollEvent(event); }

void renderer::close() { window.close(); }