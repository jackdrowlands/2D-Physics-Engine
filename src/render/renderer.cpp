#include "../../include/render/renderer.hpp"

// Constructor that initializes renderer with given width and height
// Creates a window with the specified dimensions
renderer::renderer(unsigned int width, unsigned int height)
    : window(sf::VideoMode(width, height), "Physics Engine") {}

// Function to draw a drawable object to the window
void renderer::draw(const sf::Drawable& drawable) { window.draw(drawable); }

// Function to clear the window with white color
void renderer::clear() { window.clear(sf::Color::White); }

// Function to display the contents of the window
void renderer::display() { window.display(); }

// Function to check if the window is open
bool renderer::isOpen() { return window.isOpen(); }

// Function to poll the event queue of the window
// Returns true if there is an event to process, false otherwise
bool renderer::pollEvent(sf::Event& event) { return window.pollEvent(event); }

// Function to close the window
void renderer::close() { window.close(); }