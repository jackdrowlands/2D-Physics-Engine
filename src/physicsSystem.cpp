#include "../include/physicsSystem.hpp"

// Constructor that initializes physicsSystem with given fixedDeltaTime and
// gravity
physicsSystem::physicsSystem(double fixedDeltaTime, vector2d gravity)
    : gravityForce(gravity), fixedDeltaTime(fixedDeltaTime) {
  // make quadtree
  this->quadtree = collisions::createQuadtree(colliders);
}

// Destructor
physicsSystem::~physicsSystem() {}

// Function to update the physics system
void physicsSystem::update(double dt) {
  // TODO: Probably not needed anymore.
}

// Function for multithreaded collision detection in a quadtree section
void detectCollisionsInSection(const std::vector<rigidBody*>& section,
                               std::vector<rigidBody*>& bodies1,
                               std::vector<rigidBody*>& bodies2,
                               std::vector<collisionManifold>& manifolds,
                               std::mutex& manifoldLock) {
  for (const auto& body : section) {
    for (const auto& other : section) {
      if (body != other) {
        collisionManifold* res = nullptr;
        if (body->getCollider() != nullptr && other->getCollider() != nullptr &&
            !body->hasInfiniteMass() && !other->hasInfiniteMass()) {
          res = collisions::findCollisionFeatures(body->getCollider(),
                                                  other->getCollider());
        }

        if (res != nullptr && res->getIsColliding()) {
          // Lock the mutex before modifying shared resources
          std::lock_guard<std::mutex> lock(manifoldLock);
          bodies1.push_back(body);
          bodies2.push_back(other);
          manifolds.push_back(*res);
        }
      }
    }
  }
}

// Function to update the physics system at fixed intervals
void physicsSystem::fixedUpdate() {
  bodies1.clear();
  bodies2.clear();
  manifolds.clear();
  // update octotree
  collisions::updateQuadtree(quadtree, colliders);

  // Vector to store the threads
  std::vector<std::thread> threads;
  std::mutex manifoldMutex;

  // Process each section of the quadtree in a separate thread
  for (const auto& section : quadtree) {
    if (section.size() == 0) {
      continue;
    }
    // Launch a thread to handle collision detection in this section
    threads.emplace_back(std::thread(
        detectCollisionsInSection, std::ref(section), std::ref(bodies1),
        std::ref(bodies2), std::ref(manifolds), std::ref(manifoldMutex)));
  }

  // Wait for all threads to finish
  for (auto& thread : threads) {
    if (thread.joinable()) {
      thread.join();
    }
  }

  // update forces
  registry.updateForces(fixedDeltaTime);

  // resolve collisions via iterative impulses
  for (int i = 0; i < impulseIterations; i++) {
    for (size_t j = 0; j < manifolds.size(); j++) {
      for (size_t k = 0; k < manifolds[j].getContactPoint().size(); k++) {
        applyImpulse(*bodies1[j], *bodies2[j], manifolds[j]);
      }
    }
  }

  // update velocities
  for (auto& collider : colliders) {
    collider->getRigidBody().physicsUpdate(fixedDeltaTime);
  }
}

// Function to add a rigidBody to the physics system
void physicsSystem::addCollider(collider* collider, bool addGravity) {
  colliders.push_back(collider);
  if (addGravity) registry.add(collider->getRigidBody(), &gravityForce);
}

// Function to apply an impulse to two rigidBodies
void physicsSystem::applyImpulse(rigidBody& a, rigidBody& b,
                                 collisionManifold& m) {
  double invMassA = a.getInverseMass();
  double invMassB = b.getInverseMass();
  double invMassSum = invMassA + invMassB;
  if (invMassSum == 0) {
    return;
  }

  vector2d relVel = b.getLinearVelocity() - a.getLinearVelocity();
  vector2d relNormal = m.getNormal().normalise();
  if (relVel.dot(relNormal) > 0) {
    return;
  }

  float e = (std::min(a.getCor(), b.getCor()));
  double numerator = -(1.0 + e) * relVel.dot(m.getNormal());

  // Calculate the radius from the center of mass to the contact point
  vector2d radiusA = m.getContactPoint()[0] - a.getCentreOfMass();
  vector2d radiusB = m.getContactPoint()[0] - b.getCentreOfMass();

  // Calculate the rotational inertia
  double invInertiaA = 0.00000001;
  double invInertiaB = 0.00000001;

  // Calculate the angular impulse denominator
  double angularImpulseDenominator =
      invMassSum +
      radiusA.cross(relNormal) * radiusA.cross(relNormal) * invInertiaA +
      radiusB.cross(relNormal) * radiusB.cross(relNormal) * invInertiaB;

  double j = numerator / invMassSum;
  if (m.getContactPoint().size() > 0 && j != 0.0) {
    j /= m.getContactPoint().size();
  }
  vector2d impulse = relNormal * j;
  a.setLinearVelocity(a.getLinearVelocity() - impulse * invMassA);
  b.setLinearVelocity(b.getLinearVelocity() + impulse * invMassB);

  // Apply the angular impulse
  a.setAngularVelocity(a.getAngularVelocity() -
                       radiusA.cross(impulse) * invInertiaA);
  b.setAngularVelocity(b.getAngularVelocity() +
                       radiusB.cross(impulse) * invInertiaB);
}