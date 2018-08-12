#include "rigidbody.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>

RigidBody::RigidBody(const glm::vec3 & position, glm::vec3 scale)
	: position(position)
	, scale(scale)
{
	orientation = glm::angleAxis(0.0f, glm::vec3(0, 1, 0));
	orientation = glm::normalize(orientation);

	linVelocity = glm::vec3(0);
	angVelocity = glm::vec3(0);
	linAcceleration = glm::vec3(0);
	angAcceleration = glm::vec3(0);
	forceAccumulator = glm::vec3(0);
	torqueAccumulator = glm::vec3(0);
	linDamping = 0.9f;
	angDamping = 0.9f;

	isStatic = false;
	isAwake = false;
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

void RigidBody::reset()
{
	linVelocity = glm::vec3(0);
	angVelocity = glm::vec3(0);
	linAcceleration = glm::vec3(0);
	angAcceleration = glm::vec3(0);
	clearAccumulators();
}

void RigidBody::integrate(float deltaTime)
{
	if (invMass != 0 && isAwake) {
		lastFrameAcceleration = glm::vec3(0, -9.8, 0);
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
	}
}

void RigidBody::applyForce(const glm::vec3 & force)
{
	forceAccumulator += force;
}

void RigidBody::applyForceAtWorldPoint(const glm::vec3 & force, const glm::vec3 & point)
{
	forceAccumulator += force;
	torqueAccumulator += glm::cross(force, point - position);
}

void RigidBody::applyForceAtLocalPoint(const glm::vec3 & force, const glm::vec3 & point)
{
	glm::vec3 pointWorld = transform * glm::vec4(point, 1);
	applyForceAtWorldPoint(force, pointWorld);
}

void RigidBody::applyTorqueAtLocalPoint(const glm::vec3 & torque, const glm::vec3 & point)
{
	glm::vec3 pointWorld = transform * glm::vec4(point, 1);
	glm::vec3 torqueWorld = transform * glm::vec4(torque * 10000.0f, 1);
	torqueAccumulator += glm::cross(torqueWorld, pointWorld - position);
}


glm::mat3 RigidBody::inertiaTensorCube(const glm::vec3 & halfSizes)
{
	/* mass is factored out */
	return 1.0f / 3.0f * glm::mat3(halfSizes.y * halfSizes.y + halfSizes.z * halfSizes.z, 0, 0,
								   0, halfSizes.x * halfSizes.x + halfSizes.z * halfSizes.z, 0,
								   0, 0, halfSizes.y * halfSizes.y + halfSizes.x * halfSizes.x);
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
