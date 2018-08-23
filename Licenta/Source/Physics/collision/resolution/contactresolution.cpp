#include "contactresolution.h"
#include <Physics/body/rigidbody.h>
#include <Physics/scenes/object.h>

#include <glm/gtx/matrix_cross_product.hpp>
#include <Physics/debug.h>


float SequentialImpulseContactResolver::computeDesiredDeltaVelocity(Contact *contact)
{
	float velocityFromAcceleration = 0;

	const glm::vec3 &lastFrameAccelerationA = contact->objects[0]->body != nullptr ? contact->objects[0]->body->lastFrameAcceleration : glm::vec3(0);
	const glm::vec3 &lastFrameAccelerationB = contact->objects[1]->body != nullptr ? contact->objects[1]->body->lastFrameAcceleration : glm::vec3(0);

	velocityFromAcceleration = glm::dot(lastFrameAccelerationA, contact->normal) - glm::dot(lastFrameAccelerationB, contact->normal);

	const float effectiveRestitution = contact->closingVelocityContact.x - velocityFromAcceleration >= PhysicsSettings::get().collisionResolution.minVelocityForRestitution ? contact->restitutionCoef : 0.0f;

	return -contact->closingVelocityContact.x - (effectiveRestitution * (contact->closingVelocityContact.x - velocityFromAcceleration));
}

glm::vec3 SequentialImpulseContactResolver::computeImpulse(Contact *contact)
{
	float totalInvMass = 0;
	glm::mat3 rotationalInertiaInvWorld(0);

	for (uint8_t i = 0; i < 2; i++) {
		if (contact->objects[i]->body->invMass == 0)
			continue;

		glm::mat3 impulseToTorque = glm::matrixCross3(contact->relativeContactPositions[i]);
		glm::mat3 rotationalInertiaInvLocal = -1.0f * impulseToTorque * contact->objects[i]->body->invInertiaTensorWorld * impulseToTorque;
		rotationalInertiaInvWorld += rotationalInertiaInvLocal;
		totalInvMass += contact->objects[i]->body->invMass;
	}


	/* build change in velocity due to rotation and linear motion */
	glm::mat3 totalInertiaInvContact = (contact->matWorldToContact * rotationalInertiaInvWorld) * contact->matContactToWorld;
	totalInertiaInvContact += glm::mat3(totalInvMass);

	const glm::mat3 impulsePerUnitVelocity = glm::inverse(totalInertiaInvContact);

	const glm::vec3 velocityToKill = glm::vec3(contact->desiredDeltaVelocity, -contact->closingVelocityContact.y, -contact->closingVelocityContact.z);

	glm::vec3 impulseContact = -impulsePerUnitVelocity * velocityToKill;

	const float planarImpulse = glm::sqrt(impulseContact.y * impulseContact.y + impulseContact.z * impulseContact.z);
	if (planarImpulse > impulseContact.x * contact->frictionCoef) {
		PRINT_WARN("into planar if\n");
		impulseContact.y /= planarImpulse;
		impulseContact.z /= planarImpulse;
		impulseContact.x =
			totalInertiaInvContact[0][0] +
			totalInertiaInvContact[1][0] * contact->frictionCoef * impulseContact.y +
			totalInertiaInvContact[2][0] * contact->frictionCoef * impulseContact.z;
		impulseContact.x = contact->desiredDeltaVelocity / impulseContact.x;
		impulseContact.y *= contact->frictionCoef * impulseContact.x;
		impulseContact.z *= contact->frictionCoef * impulseContact.x;

		impulseContact *= -1.0f;
	}

	return contact->matContactToWorld * impulseContact;
	//return impulseContact;
}


void SequentialImpulseContactResolver::solveContactManifold(ContactManifold &manifold)
{
	unsigned int penetrationIterations = 0;
	unsigned int velocityIterations = 0;

	for (auto & contact : manifold.contacts) {
		for (uint8_t i = 0; i < 2; i++) {
			if (contact->objects[i]->collider->body != nullptr)
				contact->objects[i]->collider->body->isAwake = true;
		}

		contact->computeDerivedData();
		contact->desiredDeltaVelocity = computeDesiredDeltaVelocity(contact);
	}

	const float penetrationQuotaPerIteration = 1.0f;


	/* fix interpenetration */
	for (penetrationIterations = 0; penetrationIterations < PhysicsSettings::get().collisionResolution.PEN_MAX_ITERATIONS; penetrationIterations++) {
		/* get the contact with deepest penetration */
		Contact *deepestContact = nullptr;
		for (auto & contact : manifold.contacts)
			if (deepestContact == nullptr || contact->penetration > deepestContact->penetration)
				deepestContact = contact;

		if (deepestContact == nullptr || deepestContact->penetration < PhysicsSettings::get().epsilons.PEN_EPSILON) {
			penetrationIterations++;
			break;
		}

		float totalInertia = 0;
		float linearInertias[2] = { 0, 0 };
		float angularInertias[2] = { 0, 0 };

		for (uint8_t i = 0; i < 2; i++) {
			if (deepestContact->objects[i]->body == nullptr || deepestContact->objects[i]->body->isStatic)
				continue;

			glm::vec3 angularInertiaWorld = glm::cross(deepestContact->relativeContactPositions[i], deepestContact->normal);
			angularInertiaWorld = deepestContact->objects[i]->body->invInertiaTensorWorld * angularInertiaWorld;
			angularInertiaWorld = glm::cross(angularInertiaWorld, deepestContact->relativeContactPositions[i]);
			angularInertias[i] = glm::dot(angularInertiaWorld, deepestContact->normal);

			linearInertias[i] = deepestContact->objects[i]->body->invMass;

			totalInertia += linearInertias[i] + angularInertias[i];
		}

		for (uint8_t i = 0; i < 2; i++) {

			if (deepestContact->objects[i]->body->isStatic)
				continue;

			float linearMovementMagnitude = penetrationQuotaPerIteration * (i == 0 ? 1.0f : -1.0f) * deepestContact->penetration * linearInertias[i] / totalInertia;
			float angularMovementMagnitude = penetrationQuotaPerIteration * (i == 0 ? 1.0f : -1.0f) * deepestContact->penetration * angularInertias[i] / totalInertia;

			/* restrict angular movement for stability reasons */
			glm::vec3 relativeContactPositionInNormalDirection =
				deepestContact->relativeContactPositions[i] + deepestContact->normal * -glm::dot(deepestContact->relativeContactPositions[i], deepestContact->normal);
			float angularMovementLimit = PhysicsSettings::get().collisionResolution.angularMovementLimitFactor * glm::length(relativeContactPositionInNormalDirection);

			if (glm::abs(angularMovementMagnitude) > angularMovementLimit) {
				float totalMove = linearMovementMagnitude + angularMovementMagnitude;
				angularMovementMagnitude = glm::clamp(angularMovementMagnitude, -angularMovementLimit, angularMovementLimit);
				linearMovementMagnitude = totalMove - angularMovementMagnitude;
			}

			/* compute linear and angular deltas */
			glm::vec3 linearDelta = deepestContact->normal * linearMovementMagnitude;
			glm::vec3 angularDelta = glm::vec3(0);

			if (angularMovementMagnitude != 0) {
				glm::vec3 rotationDirection = glm::cross(deepestContact->relativeContactPositions[i], deepestContact->normal);
				angularDelta = (deepestContact->objects[i]->body->invInertiaTensorWorld * rotationDirection) * (angularMovementMagnitude / angularInertias[i]);
			}

			/* apply the changes */
			deepestContact->objects[i]->body->position += linearDelta;
			glm::quat aux = glm::quat(0, angularDelta); aux *= deepestContact->objects[i]->body->orientation;
			aux = deepestContact->objects[i]->body->orientation + 0.5f * aux;
			deepestContact->objects[i]->body->orientation = glm::normalize(aux);
			/*deepestContact->objects[i]->body->updateTransformMatrix();
			deepestContact->objects[i]->body->updateInvInertiaTensorWorld();*/

			/* TODO: other affected contacts??????? */
			for (auto &contact : manifold.contacts) {

				const uint8_t colliderIdxInOtherContact = contact->objects[0] == deepestContact->objects[i] ? 0 : 1;
				const float sign = colliderIdxInOtherContact == 0 ? -1.0f : 1.0f;

				const glm::vec3 deltaPosition = linearDelta + glm::cross(angularDelta, contact->relativeContactPositions[colliderIdxInOtherContact]);
				contact->penetration += sign * glm::dot(deltaPosition, contact->normal);

				contact->relativeContactPositions[1] += deltaPosition * sign;

				if (colliderIdxInOtherContact == 0) {
					contact->points[0] += deltaPosition;
					contact->points[1] -= deltaPosition;
				}

			}
		}
		
	}

	/* fix velocities */
	for (velocityIterations = 0; velocityIterations < PhysicsSettings::get().collisionResolution.VEL_MAX_ITERATIONS; velocityIterations++) {
		Contact *slowestContact = nullptr;
		for (auto & contact : manifold.contacts) {
			if (contact->penetration < -PhysicsSettings::get().epsilons.PEN_EPSILON)
				continue;
			if (slowestContact == nullptr || contact->desiredDeltaVelocity < slowestContact->desiredDeltaVelocity)
				slowestContact = contact;
		}

		if (slowestContact == nullptr || slowestContact->desiredDeltaVelocity >= PhysicsSettings::get().epsilons.VEL_EPSILON) {
			velocityIterations++;
			break;
		}

		PRINT_WARN(slowestContact->toString());

		glm::vec3 impulse = computeImpulse(slowestContact);
		PRINT_WARN("impulse = " << impulse << "\n");

		for (uint8_t i = 0; i < 2; i++) {
			
			if (slowestContact->objects[i]->body->isStatic)
				continue;

			const glm::vec3 velDelta = (i == 0 ? 1.0f : -1.0f) * impulse * slowestContact->objects[i]->body->invMass;
			slowestContact->objects[i]->body->linVelocity += velDelta;

			glm::vec3 angVelDelta = slowestContact->objects[i]->body->invInertiaTensorWorld * (i == 0 ? 1.0f : -1.0f) * glm::cross(slowestContact->relativeContactPositions[i], impulse);
			slowestContact->objects[i]->body->angVelocity += angVelDelta;
			PRINT_WARN("angVelDelta = " << angVelDelta << "\nbody angVelocity = " << slowestContact->objects[i]->body->angVelocity << "\n");

			/* other affected contact */
			for (auto &contact : manifold.contacts) {
				const uint8_t colliderIdxInOtherContact = contact->objects[0] == slowestContact->objects[i] ? 0 : 1;
				const float sign = colliderIdxInOtherContact == 0 ? -1.0f : 1.0f;

				const glm::vec3 delta = velDelta + glm::cross(angVelDelta, contact->relativeContactPositions[colliderIdxInOtherContact]);
				contact->closingVelocityWorld += sign * delta;
				contact->closingVelocityContact = contact->matWorldToContact * contact->closingVelocityWorld;
				contact->desiredDeltaVelocity = computeDesiredDeltaVelocity(contact);
			}
		}
	}
	PRINT_WARN("AFTER " << penetrationIterations << "penIters and " << velocityIterations << "velIters\n");
}

void SequentialImpulseContactResolver::solve(const std::vector<CollisionPoint*> &collisions)
{
	updateContacts(collisions);

	for (auto &manifold : manifolds) {
		solveContactManifold(manifold);
	}
}

void SequentialImpulseContactResolver::updateContacts(const std::vector<CollisionPoint*> & collisions)
{
	timestamp++;

	for (unsigned int i = 0; i < collisions.size(); i++) {
		const CollisionPoint *collision = collisions[i];

		ContactManifold *manifold = nullptr;

		for (auto & manIt : manifolds) {
			if (manIt.obj1 == collision->objects[0] && manIt.obj2 == collision->objects[1]) {
				manifold = &manIt;
			}
		}

		/* if there's no manifold for this pair of objects, try again with reversed order */
		if (manifold == nullptr) {
			for (auto & manIt : manifolds) {
				if (manIt.obj1 == collision->objects[1] && manIt.obj2 == collision->objects[0]) {
					manifold = &manIt;
					CollisionPoint reversedContact = collision->reverse();

					addContactToManifold(*manifold, reversedContact);
					continue;
				}
			}
			if (manifold == nullptr) {
				/* create new manifold for this object pair */
				manifolds.emplace_back(collision->objects[0], collision->objects[1]);
				manifold = &manifolds.back();
			}
		}

		addContactToManifold(*manifold, *collision);
	}

	/* remove old contact manifolds that were not updated */
	for (auto manIt = manifolds.begin(); manIt != manifolds.end();) {

		if (manIt->timestamp < timestamp) {
			for (uint8_t c = 0; c < manIt->contacts.size(); c++) {
				contacts.remove(*(manIt->contacts[c]));
			}

			manIt = manifolds.erase(manIt);
			continue;
		}

		PhysicsObject *obj1 = manIt->obj1;
		PhysicsObject *obj2 = manIt->obj2;

		/* check for old contacts in this manifold*/
		for (uint8_t c = 0; c < manIt->contacts.size();) {
			Contact *contact = manIt->contacts[c];

			if (contact->timestamp < timestamp) {
				/* new world positions */
				glm::vec3 worldPoint1 = glm::vec3(obj1->getTransformMatrix() * glm::vec4(contact->localPoints[0], 1));
				glm::vec3 worldPoint2 = glm::vec3(obj2->getTransformMatrix() * glm::vec4(contact->localPoints[1], 1));
			
				/* recompute penetration distance */
				const float penetration = glm::dot(worldPoint2 - worldPoint1, contact->normal);

				/* compute the ﻿﻿projection of the world-space point#2 on the normal */
				glm::vec3 projectedPoint2 = worldPoint1 + contact->normal * penetration;
				glm::vec3 tangentVector = projectedPoint2 - worldPoint2;
				float tangentDistanceSquared = glm::dot(tangentVector, tangentVector);

				/* Check to see if this contact should be removed.
				This occurrs if the distance becomes positive (objects are no
				longer colliding) or if the distance between the collision points in the plane
				perpendicular to the normal vector becomes too large. */
				if (penetration > PhysicsSettings::get().epsilons.globalEpsilon || tangentDistanceSquared > PhysicsSettings::get().epsilons.globalEpsilon) {
					contacts.remove(*(manIt->contacts[c]));

					manIt->contacts.erase(manIt->contacts.begin() + c);
					continue;
				}

				/* update the contact with its new data */
				contact->points[0] = worldPoint1;
				contact->points[1] = worldPoint2;
				contact->penetration = penetration;
				contact->timestamp = timestamp;
			}
			c++;
		}
		manIt++;
	}
}


void SequentialImpulseContactResolver::addContactToManifold(ContactManifold & manifold, const CollisionPoint & newContact)
{
	manifold.timestamp = timestamp;
	const PhysicsObject *obj1 = manifold.obj1;
	const PhysicsObject *obj2 = manifold.obj2;

	const glm::vec3 worldPoint1 = newContact.points[0];
	const glm::vec3 worldPoint2 = newContact.points[1];

	const glm::vec3 localPoint1 = glm::vec3(glm::inverse(obj1->getTransformMatrix()) * glm::vec4(worldPoint1, 1));
	const glm::vec3 localPoint2 = glm::vec3(glm::inverse(obj2->getTransformMatrix()) * glm::vec4(worldPoint2, 1));

	const float toleranceSquared = PhysicsSettings::get().collisionResolution.PersistentContactDistanceThreshold;

	for (uint8_t i = 0; i < manifold.contacts.size(); i++) {
		Contact *contact = manifold.contacts[i];

		const glm::vec3 delta1 = localPoint1 - contact->localPoints[0];
		const glm::vec3 delta2 = localPoint2 - contact->localPoints[1];
		const float distance1Squared = glm::dot(delta1, delta1);
		const float distance2Squared = glm::dot(delta2, delta2);
		const int closeEnough = (distance1Squared < toleranceSquared) & (distance2Squared < toleranceSquared);

		if (closeEnough)
		{
			contact->setContactInfo(newContact);
			contact->timestamp = timestamp;
			return;
		}
	}

	Contact *contact;

	if (manifold.contacts.size() == PhysicsSettings::get().collisionResolution.maxContactsPerManifold) {
		/* Replace the worst contact point from the manifold with the new one */
		uint8_t index = findWorstContact(manifold);
		manifold.contacts.erase(manifold.contacts.begin() + index);
	} 
	contacts.emplace_back();
	contact = &contacts.back();
	manifold.contacts.push_back(contact);

	contact->setContactInfo(newContact);
	contact->timestamp = timestamp;
}

uint8_t SequentialImpulseContactResolver::findWorstContact(ContactManifold & manifold)
{
	uint8_t lowestPenetrationIdx = 0;
	float lowestPenetration = manifold.contacts[0]->penetration;

	for (uint8_t i = 1; i < manifold.contacts.size(); i++) {
		Contact *contact = manifold.contacts[i];

		if (contact->penetration < lowestPenetration) {
			lowestPenetration = contact->penetration;
			lowestPenetrationIdx = i;
		}
	}

	// Compute the areas of all possible triangle permutations.

	const glm::vec3 v0v1 = manifold.contacts[0]->points[0] - manifold.contacts[1]->points[0];
	const glm::vec3 v0v2 = manifold.contacts[0]->points[0] - manifold.contacts[2]->points[0];
	const glm::vec3 v0v3 = manifold.contacts[0]->points[0] - manifold.contacts[3]->points[0];
	const glm::vec3 v1v2 = manifold.contacts[1]->points[0] - manifold.contacts[2]->points[0];
	const glm::vec3 v1v3 = manifold.contacts[1]->points[0] - manifold.contacts[3]->points[0];
	const glm::vec3 v2v3 = manifold.contacts[2]->points[0] - manifold.contacts[3]->points[0];

	float *areas = new float[PhysicsSettings::get().collisionResolution.maxContactsPerManifold];

	// Compute the areas for each triangle formed by excluding the vertex at the area index.
	areas[0] = (lowestPenetrationIdx != 0) ? glm::length(glm::cross(v1v2, v1v3)) : 0.0f;
	areas[1] = (lowestPenetrationIdx != 1) ? glm::length(glm::cross(v0v2, v0v3)) : 0.0f;
	areas[2] = (lowestPenetrationIdx != 2) ? glm::length(glm::cross(v0v1, v0v3)) : 0.0f;
	areas[3] = (lowestPenetrationIdx != 3) ? glm::length(glm::cross(v0v1, v0v2)) : 0.0f;

	// Find the index of the largest area and remove the vertex at that index.
	uint8_t maxAreaIndex = 0;
	float maxArea = areas[0];

	for (uint8_t i = 1; i < manifold.contacts.size(); i++)
	{
		if (areas[i] > maxArea)
		{
			maxAreaIndex = i;
			maxArea = areas[i];
		}
	}

	delete areas;
	return maxAreaIndex;
}
