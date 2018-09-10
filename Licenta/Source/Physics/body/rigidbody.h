#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>

#include <Physics/settings.h>

class RigidBody {
  friend class SequentialImpulseContactResolver;
  friend class Contact;

 public:
  glm::vec3 position;    /* position in world coordinates */
  glm::quat orientation; /* orientation quaternion */

  glm::vec3 scale; /* scaling factor in each of the 3 global axes */
  float mass;      /* body's mass */
  float invMass;   /* body's inverted mass */

  bool isStatic; /* tells if the object is immovable for collision resolution
                    purposes */
  bool isAwake;  /* tells if the object is subject to integration */

  glm::mat4 transform; /* local to world transform matrix */

  float frictionCoef;    /* body's friction coefficient */
  float restitutionCoef; /* body's restitution coefficient */

 private:
  glm::vec3 linVelocity; /* linear velocity in world space */
  glm::vec3 angVelocity; /* angular velocity in world space */

  glm::vec3 linAcceleration; /* linear acceleration in world space */
  glm::vec3 angAcceleration; /* angular acceleration in world space */
  glm::vec3
      lastFrameAcceleration; /* linear acceleration in world space during the
                                last frame */

  glm::vec3
      forceAccumulator; /* total linear force applied to the body's center of
                           mass in world space */
  glm::vec3 torqueAccumulator; /* total torque in world space */
  float motion;                /* recent linear + angular motion */

  float linDamping;        /* linear damping factor for simulated friction */
  float angDamping;        /* angular damping factor for simulated friction */
  glm::mat3 inertiaTensor; /* moment of inertia tensor in body's local space,
                              with mass factored out */
  glm::mat3 invInertiaTensorWorld; /* inverse inertia tensor in world space,
                                      used in calculations */

 public:
  RigidBody(const glm::vec3& position = glm::vec3(0),
            const glm::vec3& scale = glm::vec3(1),
            const glm::quat& orientation =
                glm::normalize(glm::angleAxis(0.0f, glm::vec3(0, 1, 0))));

  /* updates the transform matrix from state data(position, orientation, scale)
   */
  void updateTransformMatrix();
  /* updates the inverse inertia tensor in world coordinates from state
   * data(position, orientation) */
  void updateInvInertiaTensorWorld();

  /* sets the body's mass and inverse mass properties */
  void setMass(float mass, bool isInverse = false);
  /* sets the body's inertia tensor and inverse inertia tensor */
  void setInertiaTensor(const glm::mat3& inertiaTensor);

  /* clears force and torque accumulators */
  void clearAccumulators();
  /* resets all of the body's movement */
  void resetMovement();

  /* performs integration */
  void integrate(float deltaTime);

  /* Applies a force to the center of mass (linear only) */
  void applyForce(const glm::vec3& force);
  /* applies a force to a world space point (linear + angular) */
  void applyForceAtWorldPoint(const glm::vec3& force, const glm::vec3& point);
  /* applies a force to a local space point (linear + angular) */
  void applyForceAtLocalPoint(const glm::vec3& force, const glm::vec3& point);
  /* applies a torque to a local space point (angular only) */
  void applyTorqueAtLocalPoint(const glm::vec3& torque, const glm::vec3& point);
  /* applies a linear impulse to the center of mass (linear velocity change) */
  void applyLinearImpulse(const glm::vec3& impulse);

  /* basic moment of inertia of a cube(with mass factored out) */
  static glm::mat3 inertiaTensorCube(
      const glm::vec3& halfsizes = PhysicsSettings::get().shapes.box.halfSizes);
  /* basic moment of inertia of a sphere(with mass factored out) */
  static glm::mat3 inertiaTensorSphere(
      const float radius = PhysicsSettings::get().shapes.sphere.radius);
  /* basic moment of inertia of a cylinder(with mass factored out) */
  static glm::mat3 inertiaTensorCylinder(
      const float height = PhysicsSettings::get().shapes.cylinder.height,
      const float radius = PhysicsSettings::get().shapes.cylinder.radius);
  /* basic moment of inertia of a cone(with mass factored out) NOT SUPPORTED YET
   */
  static glm::mat3 inertiaTensorCone(const float height = 1.0f,
                                     const float radius = 0.5f);
  /* basic moment of inertia of a capsule(with mass factored out) */
  static glm::mat3 inertiaTensorCapsule(
      const float height = PhysicsSettings::get().shapes.capsule.height,
      const float radius = PhysicsSettings::get().shapes.capsule.radius);

  std::string toString();
  std::string toStringPrivateFields();
};
