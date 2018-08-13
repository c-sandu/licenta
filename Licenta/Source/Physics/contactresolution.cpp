#include "contactresolution.h"
#include "contact.h"
#include "object.h"

#include <glm/gtx/matrix_cross_product.hpp>
#include "debug.h"

void ImpulseContactResolver::buildContactManifolds()
{
	contactManifolds.clear();

	for (auto & contact : contacts) {

		if (contact->invalid)
			continue;

		for (uint8_t i = 0; i < 2; i++)
			contact->objects[i]->collider->contactManifold = NULL;
	}
	
	for (auto & contact : contacts) {
		if (contact->invalid)
			continue;

		for (uint8_t i = 0; i < 2; i++) {
			if (contact->objects[i]->collider->contactManifold == NULL && !contact->objects[i]->body->isStatic) {
				contactManifolds.emplace_back();
				contact->objects[i]->collider->contactManifold = &contactManifolds.back();
			}
		}

		std::list<Contact*> *manifoldA = contact->objects[0]->collider->contactManifold;
		std::list<Contact*> *manifoldB = contact->objects[1]->collider->contactManifold;

		if (manifoldA == NULL || manifoldB == NULL) {
			if (manifoldA != NULL)
				manifoldA->push_back(contact);
			if (manifoldB != NULL)
				manifoldB->push_back(contact);
		}
		else {
			if (manifoldA != manifoldB) {
				manifoldA->splice(manifoldA->end(), *manifoldB);
				contact->objects[1]->collider->contactManifold = manifoldB = manifoldA;
			}
			manifoldA->push_back(contact);
		}

		contactManifolds.remove_if([](const std::list<Contact*> &item)->bool {
			return item.size() == 0;
		});
	};
}

static float computeDesiredDeltaVelocity(Contact *contact)
{
	const float MIN_VELOCITY_FOR_RESTITUTION = 0.1f;

	float velocityFromAcceleration = 0;

	const glm::vec3 &lastFrameAccelerationA = contact->objects[0]->body != NULL ? contact->objects[0]->body->lastFrameAcceleration : glm::vec3(0);
	const glm::vec3 &lastFrameAccelerationB = contact->objects[1]->body != NULL ? contact->objects[1]->body->lastFrameAcceleration : glm::vec3(0);

	velocityFromAcceleration = glm::dot(lastFrameAccelerationA, contact->normal) - glm::dot(lastFrameAccelerationB, contact->normal);

	const float effectiveRestitution = contact->closingVelocityContact.x - velocityFromAcceleration >= MIN_VELOCITY_FOR_RESTITUTION ? contact->restitutionCoef : 0.0f;

	return -contact->closingVelocityContact.x - (effectiveRestitution * (contact->closingVelocityContact.x - velocityFromAcceleration));
}

static glm::vec3 computeImpulse(Contact *contact)
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
	glm::mat3 totalInertiaInvContact = contact->matContactToWorld * rotationalInertiaInvWorld * contact->matWorldToContact;
	totalInertiaInvContact += glm::mat3(totalInvMass);

	const glm::mat3 impulsePerUnitVelocity = glm::inverse(totalInertiaInvContact);

	const glm::vec3 velocityToKill = glm::vec3(contact->desiredDeltaVelocity, -contact->closingVelocityContact.y, -contact->closingVelocityContact.z);

	glm::vec3 impulseContact = impulsePerUnitVelocity * velocityToKill;

	const float planarImpulse = glm::sqrt(impulseContact.y * impulseContact.y + impulseContact.z * impulseContact.z);
	if (planarImpulse > impulseContact.x * contact->frictionCoef) {
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
}


void ImpulseContactResolver::solveContactManifold(const std::list<Contact*> &manifold)
{
	const unsigned int PEN_MAX_ITERATIONS = 5;
	const unsigned int VEL_MAX_ITERATIONS = 5;

	const float PEN_EPSILON = 0.01;
	const float VEL_EPSILON = 0.025;

	unsigned int penetrationIterations = 0;
	unsigned int velocityIterations = 0;

	for (auto & contact : manifold) {
		for (uint8_t i = 0; i < 2; i++) {
			if (contact->objects[i]->collider->body != NULL)
				contact->objects[i]->collider->body->isAwake = true;
		}

		contact->computeDerivedData();
		contact->desiredDeltaVelocity = computeDesiredDeltaVelocity(contact);
	}

	const float penetrationQuotaPerIteration = 1.0f;


	/* fix interpenetration */
	for (penetrationIterations = 0; penetrationIterations < PEN_MAX_ITERATIONS; penetrationIterations++) {
		/* get the contact with deepest penetration */
		Contact *deepestContact = NULL;
		for (auto & contact : manifold)
			if (deepestContact == NULL || contact->penetration > deepestContact->penetration)
				deepestContact = contact;

		if (deepestContact == NULL || deepestContact->penetration < PEN_EPSILON) {
			penetrationIterations++;
			break;
		}

		float totalInertia = 0;
		float linearInertias[2] = { 0, 0 };
		float angularInertias[2] = { 0, 0 };

		for (uint8_t i = 0; i < 2; i++) {
			if (deepestContact->objects[i]->body == NULL || deepestContact->objects[i]->body->isStatic)
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
			float angularMovementLimit = 0.2f * relativeContactPositionInNormalDirection.length();

			if (glm::abs(angularMovementMagnitude > angularMovementLimit)) {
				float totalMove = linearMovementMagnitude + angularMovementMagnitude;
				angularMovementMagnitude = glm::clamp(angularMovementMagnitude, -angularMovementLimit, angularMovementLimit);
				linearMovementMagnitude = totalMove - angularMovementMagnitude;
			}

			/* compute linear and angular deltas */
			glm::vec3 linearDelta = deepestContact->normal * linearMovementMagnitude;
			glm::vec3 angularDelta = glm::vec3(0);

			if (angularMovementMagnitude != 0) {
				glm::vec3 rotationDirection = glm::cross(deepestContact->relativeContactPositions[i], deepestContact->normal);
				angularDelta = deepestContact->objects[i]->body->invInertiaTensorWorld * rotationDirection * angularMovementMagnitude / angularInertias[i];
			}

			/* apply the changes */
			deepestContact->objects[i]->body->position += linearDelta;
			deepestContact->objects[i]->body->orientation = glm::normalize(deepestContact->objects[i]->body->orientation * 0.5f * glm::quat(0, angularDelta) + deepestContact->objects[i]->body->orientation);
			deepestContact->objects[i]->body->updateTransformMatrix();
			deepestContact->objects[i]->body->updateInvInertiaTensorWorld();

			/* TODO: other affected contacts??????? */
			for (auto &contact : manifold) {

				const uint8_t colliderIdxInOtherContact = contact->objects[0] == deepestContact->objects[i] ? 0 : 1;
				const float sign = colliderIdxInOtherContact == 0 ? -1 : 1;

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
	for (velocityIterations = 0; velocityIterations < VEL_MAX_ITERATIONS; velocityIterations++) {
		Contact *slowestContact = NULL;
		for (auto & contact : manifold) {
			if (contact->penetration < -0.1f)
				continue;
			if (slowestContact == NULL || contact->desiredDeltaVelocity < slowestContact->desiredDeltaVelocity)
				slowestContact = contact;
		}

		if (slowestContact == NULL || slowestContact->desiredDeltaVelocity >= -VEL_EPSILON) {
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

			/* TODO: other affected contacts??????? */
			for (auto &contact : manifold) {
				const uint8_t colliderIdxInOtherContact = contact->objects[0] == slowestContact->objects[i] ? 0 : 1;
				const float sign = colliderIdxInOtherContact == 0 ? -1 : 1;

				const glm::vec3 delta = velDelta + glm::cross(angVelDelta, contact->relativeContactPositions[colliderIdxInOtherContact]);
				contact->closingVelocityWorld += sign * delta;
				contact->closingVelocityContact = contact->matWorldToContact * contact->closingVelocityWorld;
				contact->desiredDeltaVelocity = computeDesiredDeltaVelocity(contact);
			}
		}
	}
	PRINT_WARN("AFTER " << penetrationIterations << "penIters and " << velocityIterations << "velIters\n");
}

void ImpulseContactResolver::solve()
{
	if (contacts.size() > 0) {
		buildContactManifolds();

		for (auto &manifold : contactManifolds) {
			solveContactManifold(manifold);
		}
	}
}


void addContactToManifold(ContactManifold & manifold, const Contact & newContact)
{
	manifold.timestamp = glfwGetTime();
	const PhysicsObject *obj1 = manifold.obj1;
	const PhysicsObject *obj2 = manifold.obj2;

	const glm::vec3 worldPoint1 = newContact.points[0];
	const glm::vec3 worldPoint2 = newContact.points[1];
	const float toleranceSquared = 0.001f;

	for (uint8_t i = 0; i < manifold.contacts.size(); i++) {
		Contact *contact = manifold.contacts[i];

		const glm::vec3 delta1 = worldPoint1 - contact->points[0];
		const glm::vec3 delta2 = worldPoint2 - contact->points[1];
		const float distance1Squared = glm::dot(delta1, delta1);
		const float distance2Squared = glm::dot(delta2, delta2);
		const int closeEnough = distance1Squared < toleranceSquared & distance2Squared < toleranceSquared;

		if (closeEnough)
		{
			contact->points[0] = worldPoint1;
			contact->points[1] = worldPoint2;
			contact->normal = newContact.normal;
			contact->penetration = newContact.penetration;
			contact->objects[0] = newContact.objects[0];
			contact->objects[1] = newContact.objects[1];

			contact->restitutionCoef = 0.25f;
			contact->frictionCoef = 0.00001f;
			contact->invalid = false;

			contact->timestamp = glfwGetTime();
			return;
		}
	}

	Contact *contact;

	if (manifold.contacts.size() == MAX_MANIFOLD_SIZE) {
		/* Replace the worst contact point from the manifold with the new one */
		uint8_t index = findWorstContact(manifold);
		contact = manifold.contacts[index];
	} else{
		manifold.contacts.emplace_back();
		contact = manifold.contacts.back();
	}

	contact->points[0] = worldPoint1;
	contact->points[1] = worldPoint2;
	contact->normal = newContact.normal;
	contact->penetration = newContact.penetration;
	contact->objects[0] = newContact.objects[0];
	contact->objects[1] = newContact.objects[1];

	contact->restitutionCoef = 0.25f;
	contact->frictionCoef = 0.00001f;
	contact->invalid = false;

	contact->timestamp = glfwGetTime();
}

uint8_t findWorstContact(ContactManifold & manifold)
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

	float areas[MAX_MANIFOLD_SIZE];

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

	return maxAreaIndex;
}
