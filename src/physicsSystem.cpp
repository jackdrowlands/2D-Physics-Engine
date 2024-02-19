#include "../include/physicsSystem.hpp"

physicsSystem::physicsSystem(double fixedDeltaTime, vector2d gravity)
    : gravityForce(gravity), fixedDeltaTime(fixedDeltaTime) {}

physicsSystem::~physicsSystem() {
  for (auto body : rigidBodies) {
    delete body;
  }
}

void physicsSystem::update(double dt) {
  // TODO:
}

void physicsSystem::fixedUpdate() {
  bodies1.clear();
  bodies2.clear();
  manifolds.clear();

  // find collisions
  for (size_t i = 0; i < rigidBodies.size(); i++) {
    for (size_t j = i + 1; j < rigidBodies.size(); j++) {
      collisionManifold* res = nullptr;
      if (rigidBodies[i]->getCollider() != nullptr &&
          rigidBodies[j]->getCollider() != nullptr &&
          !rigidBodies[i]->hasInfiniteMass() &&
          !rigidBodies[j]->hasInfiniteMass()) {
        res = collisions::findCollisionFeatures(rigidBodies[i]->getCollider(),
                                                rigidBodies[j]->getCollider());
      }

      if (res != nullptr && res->getIsColliding()) {
        bodies1.push_back(rigidBodies[i]);
        bodies2.push_back(rigidBodies[j]);
        manifolds.push_back(*res);
      }
    }
  }

  // update forces
  registry.updateForces(fixedDeltaTime);

  // resolve collisions via iterative impulses
  for (size_t i = 0; i < impulseIterations; i++) {
    for (size_t j = 0; j < manifolds.size(); j++) {
      for (int k = 0; k < manifolds[j].getContactPoint().size(); k++) {
        applyImpulse(*bodies1[j], *bodies2[j], manifolds[j]);
      }
    }
  }

  // update velocities
  for (auto& body : rigidBodies) {
    body->physicsUpdate(fixedDeltaTime);
  }
}

void physicsSystem::addRigidBody(rigidBody* body, bool addGravity) {
  rigidBodies.push_back(body);
  if (addGravity) registry.add(body, &gravityForce);
}

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
  double invInertiaA = a.getInverseMass();
  double invInertiaB = b.getInverseMass();

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