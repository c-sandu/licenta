#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <iostream>

class RigidBody
{
public:
	glm::vec3 position;
	glm::quat orientation;

	glm::vec3 linVelocity;
	glm::vec3 angVelocity;

	glm::vec3 linAcceleration;
	glm::vec3 angAcceleration;

	glm::vec3 forceAccumulator;
	glm::vec3 torqueAccumulator;

	/* constant values */
	glm::vec3 scale;
	float invMass;
	float mass;
	glm::mat3 inertiaTensor;
	glm::mat3 invInertiaTensorWorld;
	float linDamping;
	float angDamping;

	glm::mat4 transform;

	RigidBody(const glm::vec3 & position = glm::vec3(0), glm::vec3 scale = glm::vec3(1));

	/* Updates the transform matrix from primary data(position, orientation, scale). */
	void updateTransformMatrix();
	/* Updates the inverse inertia tensor in world coordinates. */
	void updateInvInertiaTensorWorld();

	void setMass(float mass, bool isInverse = false);
	void setInertiaTensor(const glm::mat3 & inertiaTensor);

	void integrate(float deltaTime);

	/* Applies a force to the center of mass. */
	void applyForce(const glm::vec3 & force);
	void applyForceAtWorldPoint(const glm::vec3 & force, const glm::vec3 & point);
	void applyForceAtLocalPoint(const glm::vec3 & force, const glm::vec3 & point);
	void applyTorqueAtLocalPoint(const glm::vec3 & torque, const glm::vec3 & point);

	/* basic moments of inertia(with mass factored out) */
	static glm::mat3 inertiaTensorCube(const glm::vec3 & halfsizes = glm::vec3(0.5));

	std::string toString();
};

