#include "rigidbody.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <Physics/settings.h>

RigidBody::RigidBody(const glm::vec3 & position, const glm::vec3 & scale, const glm::quat & orientation)
	: position(position),
	scale(scale),
	orientation(orientation)
{
	linVelocity = glm::vec3(0);
	angVelocity = glm::vec3(0);
	linAcceleration = glm::vec3(0);
	angAcceleration = glm::vec3(0);
	forceAccumulator = glm::vec3(0);
	torqueAccumulator = glm::vec3(0);
	linDamping = PhysicsSettings::get().damping.linear;
	angDamping = PhysicsSettings::get().damping.angular;

	isStatic = false;
	isAwake = false;

	mass = PhysicsSettings::get().rigidBodies.defaultMass;
	invMass = 1.0f / mass;

	restitutionCoef = PhysicsSettings::get().rigidBodies.defaultRestitutionCoef;
	frictionCoef = PhysicsSettings::get().rigidBodies.defaultRestitutionCoef;

	motion = 4.0f;
}

void RigidBody::updateTransformMatrix()
{
	transform = glm::translate(glm::mat4(1), position) * glm::mat4_cast(orientation) * glm::scale(glm::mat4(1), scale) * glm::mat4(1);
	/*transform = glm::translate(transform, position);
	transform *= glm::mat4_cast(orientation);
	transform = glm::scale(transform, scale);*/
}

void RigidBody::updateInvInertiaTensorWorld()
{
	glm::mat3 rotMatrix = glm::mat3(1);
	rotMatrix *= glm::mat3_cast(orientation);

	invInertiaTensorWorld = rotMatrix * inertiaTensor * glm::inverse(rotMatrix);
}

void RigidBody::setMass(float mass, bool isInverse)
{
	if (isInverse) {
		this->mass = mass == 0.0f ? -1.0f : 1.0f / mass;
		this->invMass = mass;
		this->isStatic = mass == 0.0f ? true : false;
	}
	else {
		this->mass = mass;
		this->invMass = 1.0f / mass;
	}
}

void RigidBody::setInertiaTensor(const glm::mat3 & inertiaTensor)
{
	this->inertiaTensor = inertiaTensor;
	updateInvInertiaTensorWorld();
}

void RigidBody::clearAccumulators()
{
	forceAccumulator = glm::vec3(0);
	torqueAccumulator = glm::vec3(0);
}

void RigidBody::resetMovement()
{
	linVelocity = glm::vec3(0);
	angVelocity = glm::vec3(0);
	linAcceleration = glm::vec3(0);
	angAcceleration = glm::vec3(0);
	clearAccumulators();
	motion = 4.0f;
}

void RigidBody::integrate(float deltaTime)
{
	if (invMass != 0 && isAwake) {

		if (motion < PhysicsSettings::get().rigidBodies.sleepMotionThreshold) {
			resetMovement();
			isAwake = false;
			return;
		}
		lastFrameAcceleration = PhysicsSettings::get().gravity;
		lastFrameAcceleration += forceAccumulator * invMass;
		lastFrameAcceleration *= deltaTime;

		linAcceleration = lastFrameAcceleration + forceAccumulator * invMass;
		linVelocity += linAcceleration * deltaTime;
		position += linVelocity * deltaTime;

		angAcceleration = invInertiaTensorWorld * torqueAccumulator * invMass;
		angVelocity += angAcceleration * deltaTime;
		//glm::vec3 axis = glm::normalize(angVelocity);
		//float angle = glm::length(angVelocity);
		//orientation = glm::rotate(orientation, angle, angVelocity);
		//orientation = glm::normalize(orientation);
		orientation = orientation + deltaTime * 0.5f * glm::quat(0.0f, angVelocity) * orientation;
		orientation = glm::normalize(orientation);

		linVelocity *= glm::pow(linDamping, deltaTime);
		angVelocity *= glm::pow(angDamping, deltaTime);

		updateTransformMatrix();
		updateInvInertiaTensorWorld();
		clearAccumulators();

		/* compute the relative motion using a recency weighted average */
		float currentMotion = glm::dot(linVelocity, linVelocity) + glm::dot(angVelocity, angVelocity);
		float baseBias = 0.8f;
		float bias = glm::pow(baseBias, deltaTime);
		motion = bias * motion + (1.0f - bias) * currentMotion;
		glm::clamp(motion, 0.0f, 10.0f * PhysicsSettings::get().rigidBodies.sleepMotionThreshold);
	}
}

void RigidBody::applyForce(const glm::vec3 & force)
{
	glm::vec3 adjustedForce = force * PhysicsSettings::get().forceMultipliers.force;
	forceAccumulator += adjustedForce;
}

void RigidBody::applyForceAtWorldPoint(const glm::vec3 & force, const glm::vec3 & point)
{
	glm::vec3 adjustedForce = force * PhysicsSettings::get().forceMultipliers.force;
	forceAccumulator += adjustedForce;
	torqueAccumulator += glm::cross(adjustedForce, point - position);
}

void RigidBody::applyForceAtLocalPoint(const glm::vec3 & force, const glm::vec3 & point)
{
	glm::vec3 adjustedForce = force * PhysicsSettings::get().forceMultipliers.force;
	glm::vec3 pointWorld = transform * glm::vec4(point, 1);
	applyForceAtWorldPoint(adjustedForce, pointWorld);
}

void RigidBody::applyTorqueAtLocalPoint(const glm::vec3 & torque, const glm::vec3 & point)
{
	glm::vec3 adjustedTorque = torque * PhysicsSettings::get().forceMultipliers.torque;
	glm::vec3 pointWorld = transform * glm::vec4(point, 1);
	glm::vec3 torqueWorld = transform * glm::vec4(adjustedTorque, 1);
	torqueAccumulator += glm::cross(torqueWorld, pointWorld - position);
}

void RigidBody::applyLinearImpulse(const glm::vec3 & impulse)
{
	isAwake = true;
	glm::vec3 adjustedImpulse = impulse * PhysicsSettings::get().forceMultipliers.impulse;
	linVelocity += adjustedImpulse;
}


glm::mat3 RigidBody::inertiaTensorCube(const glm::vec3 & halfSizes)
{
	/* mass is factored out */
	return 1.0f / 3.0f * glm::mat3(
		halfSizes.y * halfSizes.y + halfSizes.z * halfSizes.z, 0, 0,
		0, halfSizes.x * halfSizes.x + halfSizes.z * halfSizes.z, 0,
		0, 0, halfSizes.y * halfSizes.y + halfSizes.x * halfSizes.x);
}

glm::mat3 RigidBody::inertiaTensorSphere(const float radius)
{
	/* mass is factored out */
	return 2.0f / 5.0f * glm::mat3(
		radius * radius, 0, 0,
		0, radius * radius, 0,
		0, 0, radius * radius);
}

glm::mat3 RigidBody::inertiaTensorCylinder(const float height, const float radius)
{
	/* mass is factored out */
	return 1.0f / 12.0f * glm::mat3(
		3.0f * radius * radius + height * height, 0, 0,
		0, 6.0f * radius * radius, 0,
		0, 0, 3.0f * radius * radius + height * height);
}

glm::mat3 RigidBody::inertiaTensorCone(const float height, const float radius)
{
	return 1.0f / 20.0f * glm::mat3(
		12.0f * height * height + 3.0f * radius * radius, 0, 0,
		0, 6.0f * radius * radius, 0,
		0, 0, 12.0f * height * height + 3.0f * radius * radius);
}

glm::mat3 RigidBody::inertiaTensorCapsule(const float height, const float radius)
{
	return inertiaTensorCylinder(height + radius + radius, radius);
}

std::string RigidBody::toString()
{
	return std::string("") + "\tRigidBody {" + "\n\t\t"
		+ "position = " + to_string(position) + "\n\t\t"
		+ "orientation = " + to_string(orientation) + "\n\n\t\t"
		+ "transform = " + to_string(transform) + "\n\t\t"
		+ "scale = " + to_string(scale) + ", mass = " + std::to_string(mass) + ", invMass = " + std::to_string(invMass) + "\n\t\t"
		+ "invInertiaTensorWorld = " + to_string(invInertiaTensorWorld) + "\n\n\t\t"
		+ "linVelocity = " + to_string(linVelocity) + "\n\t\t"
		+ "angVelocity = " + to_string(angVelocity) + "\n\t\t"
		+ "linAcceleration = " + to_string(linAcceleration) + "\n\t\t"
		+ "angAcceleration = " + to_string(angAcceleration) + "\n\n\t\t"
		+ "lastFrameAcceleration = " + to_string(lastFrameAcceleration) + "\n\n\t\t"
		+ "forceAccumulator = " + to_string(forceAccumulator) + "\n\t\t"
		+ "torqueAccumulator = " + to_string(torqueAccumulator) + "\n\t}\n";
}

std::string RigidBody::toStringPrivateFields()
{
	return std::string("") + "Additional info {" + "\n\t\t"
		+ "recentMotion = " + std::to_string(motion) + "\n\t\t"
		+ "linVelocity = " + to_string(linVelocity) + "\n\t\t"
		+ "angVelocity = " + to_string(angVelocity) + "\n\t\t"
		+ "linAcceleration = " + to_string(linAcceleration) + "\n\t\t"
		+ "angAcceleration = " + to_string(angAcceleration) + "\n\n\t\t"
		+ "lastFrameAcceleration = " + to_string(lastFrameAcceleration) + "\n\n\t\t"
		+ "forceAccumulator = " + to_string(forceAccumulator) + "\n\t\t"
		+ "torqueAccumulator = " + to_string(torqueAccumulator) + "\n\t}\n";
}
