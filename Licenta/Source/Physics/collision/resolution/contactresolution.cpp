#include "contactresolution.h"
#include <Physics/body/rigidbody.h>
#include <Physics/scenes/object.h>

#include <glm/gtx/matrix_cross_product.hpp>
#include <Physics/debug.h>


float SequentialImpulseContactResolver::computeDesiredDeltaVelocity(Contact *contact)
{
	/* velocity built from acceleration should be removed to prevent bounciness of resting contacts 
	before correction: v_sep = -(1 + restitution) * v_closing
	after correction:  v_sep = -v_acc - (1 + restitution) * (v_closing - v_acc) */
	float velocityFromAcceleration = 0;

	const glm::vec3 &lastFrameAccelerationA = contact->objects[0]->body != nullptr ? contact->objects[0]->body->lastFrameAcceleration : glm::vec3(0);
	const glm::vec3 &lastFrameAccelerationB = contact->objects[1]->body != nullptr ? contact->objects[1]->body->lastFrameAcceleration : glm::vec3(0);

	/* final velocity from acceleration, in the direction of the contact's normal */
	velocityFromAcceleration = glm::dot(lastFrameAccelerationA, contact->normal) - glm::dot(lastFrameAccelerationB, contact->normal);

	float effectiveRestitution = 0.0f;
	if (contact->closingVelocityContact.x - velocityFromAcceleration >= PhysicsSettings::get().collisionResolution.minVelocityForRestitution) {
		/* apply restitution only if the remaining closing velocity is greater than a threshold */
		effectiveRestitution = contact->restitutionCoef;
	}

	return -velocityFromAcceleration - (1.0f + effectiveRestitution) * (contact->closingVelocityContact.x - velocityFromAcceleration);
}

// // impulse calculation for further reference - not working
//{
//	float denoms[3];
//	for (uint8_t axis = 0; axis < 3; axis++) {
//		float totalInvMass = 0;
//		float totalRotationalInertiaWorld = 0;
//		for (uint8_t i = 0; i < 2; i++) {
//			if (contact->objects[i]->body->invMass == 0)
//				continue;
//
//			totalInvMass += contact->objects[i]->body->invMass;
//			totalRotationalInertiaWorld += glm::dot(
//				glm::cross(contact->relativeContactPositions[i], contact->matContactToWorld[axis]),
//				contact->objects[i]->body->invInertiaTensorWorld * glm::cross(contact->relativeContactPositions[i], contact->matContactToWorld[axis])
//			);
//		}
//		denoms[axis] = totalInvMass + totalRotationalInertiaWorld;
//	}
//	float nums[3];
//	nums[0] = -contact->desiredDeltaVelocity;
//	nums[1] = contact->closingVelocityContact.y;
//	nums[2] = contact->closingVelocityContact.z;
//
//	float impulseMultipliers[3] = { nums[0] / denoms[0], nums[1] / denoms[1], nums[2] / denoms[2] };
//	glm::vec3 impulseWorld =
//		contact->matContactToWorld[0] * impulseMultipliers[0] +
//		contact->matContactToWorld[1] * impulseMultipliers[1] +
//		contact->matContactToWorld[2] * impulseMultipliers[2];
//
//	const float planarImpulse = glm::sqrt(impulseMultipliers[1] * impulseMultipliers[1] + impulseMultipliers[2] * impulseMultipliers[2]);
//	if (planarImpulse > impulseMultipliers[0] * contact->frictionCoef) {
//		impulseMultipliers[1] /= planarImpulse; /* normalize y component */
//		impulseMultipliers[2] /= planarImpulse; /* normalize z component */
//		impulseMultipliers[0] =
//			(contact->matContactToWorld[0] * denoms[0]).x +
//			(contact->matContactToWorld[0] * denoms[0]).y * contact->frictionCoef * impulseMultipliers[1] +
//			(contact->matContactToWorld[0] * denoms[0]).z * contact->frictionCoef * impulseMultipliers[2];
//
//		impulseMultipliers[0] = contact->desiredDeltaVelocity / impulseMultipliers[0];
//		impulseMultipliers[1] *= contact->frictionCoef * impulseMultipliers[0];
//		impulseMultipliers[2] *= contact->frictionCoef * impulseMultipliers[0];
//	}
//
//	glm::vec3 newImpulseWorld =
//		contact->matContactToWorld[0] * impulseMultipliers[0] +
//		contact->matContactToWorld[1] * impulseMultipliers[1] +
//		contact->matContactToWorld[2] * impulseMultipliers[2];
//	return newImpulseWorld;
//}

/*
	                              -(1 + restitution) *  v_closing_linear
	linear impulse magnitude j = ----------------------------------------
	                                      inv_mass_a + inv_mass_b

	v_relative_a_to_b = v_closing_linear + ang_velocity_a cross r_a - ang_velocity_b cross r_b
	                                                     -(1 + restitution) * v_relative_a_to_b
	impulse magnitude(incl. angular change) j = -------------------------------------------------------------------------------------------------------------------
	                                            (   1        1    )   (                                                                                           )
	                                            ( ------ + ------ ) + ( dot(r_a cross n, invI_a^-1 * (r_a cross n)) + dot(r_b cross n, invI_b^-1 * (r_b cross n)) )
	                                            ( mass_a   mass_b )   (                                                                                           )

	when working with frictions, in the denominator, the normal n would be replaced by all the three axes of the contact basis (matWorldToContact)
*/
glm::vec3 SequentialImpulseContactResolver::computeImpulse(Contact *contact)
{	
	float totalInvMass = 0; /* used for computing denominator left term */
	glm::mat3 totalRotationalInertiaWorld(0); /* actual denominator right term */


	/* start computing the denominator in world space */
	for (uint8_t i = 0; i < 2; i++) {
		if (contact->objects[i]->body->invMass == 0)
			continue;

		totalInvMass += contact->objects[i]->body->invMass;
		
		/* instead of solving for each axis separately, use the matrix cross product trick */
		/* impulseToTorque is actually holding all the (r_a cross n) products from the above formula for the 3 different contact basis axes:*/
		glm::mat3 impulseToTorque = glm::matrixCross3(contact->relativeContactPositions[i]);
		glm::mat3 tmpRotationalInertiaWorld = impulseToTorque * contact->objects[i]->body->invInertiaTensorWorld;
		tmpRotationalInertiaWorld = -tmpRotationalInertiaWorld * impulseToTorque;
		totalRotationalInertiaWorld += tmpRotationalInertiaWorld;
	}


	/* transform to contact space to add the linear component along each of the 3 axes */
	glm::mat3 totalInertiaContact = (contact->matWorldToContact * totalRotationalInertiaWorld) * contact->matContactToWorld;
	/* add linear component of the demominator */
	totalInertiaContact += glm::mat3(totalInvMass);

	/* get inverse of the denominator so that we can multiply the numerator */
	const glm::mat3 impulsePerUnitVelocity = glm::inverse(totalInertiaContact);

	/* this is the numerator */
	const glm::vec3 velocityToKill = glm::vec3(contact->desiredDeltaVelocity, -contact->closingVelocityContact.y, -contact->closingVelocityContact.z);

	/* final form of the impulse in contact space */
	glm::vec3 impulseContact = -impulsePerUnitVelocity * velocityToKill;

	/* total planar impulse */
	const float planarImpulse = glm::sqrt(impulseContact.y * impulseContact.y + impulseContact.z * impulseContact.z);
	/* check if we have dynamic friction  */
	if (planarImpulse > impulseContact.x * contact->frictionCoef) {
		PRINT_WARN("into planar if\n");
		impulseContact.y /= planarImpulse; /* normalize y component */
		impulseContact.z /= planarImpulse; /* normalize z component */
		impulseContact.x =
			totalInertiaContact[0][0] +
			totalInertiaContact[0][1] * contact->frictionCoef * impulseContact.y +
			totalInertiaContact[0][2] * contact->frictionCoef * impulseContact.z;
		impulseContact.x = contact->desiredDeltaVelocity / impulseContact.x;
		impulseContact.y *= contact->frictionCoef * impulseContact.x;
		impulseContact.z *= contact->frictionCoef * impulseContact.x;

		impulseContact *= -1.0f;
	}

	return contact->matContactToWorld * impulseContact;
}

void SequentialImpulseContactResolver::applyPositionUpdate(ContactManifold &manifold, Contact *deepestContact)
{
	/* compute linear and angular inertia for the deepest contact */
	float totalInertia = 0;
	float linearInertias[2] = { 0, 0 };
	float angularInertias[2] = { 0, 0 };
	for (uint8_t i = 0; i < 2; i++) {
		if (deepestContact->objects[i]->body == nullptr || deepestContact->objects[i]->body->isStatic)
			continue;

		/* angInertia = dot(r_a cross n, invI_a^-1 * (r_a cross n)) */
		angularInertias[i] = glm::dot(
			glm::cross(deepestContact->relativeContactPositions[i], deepestContact->normal),
			deepestContact->objects[i]->body->invInertiaTensorWorld * glm::cross(deepestContact->relativeContactPositions[i], deepestContact->normal)
		);

		linearInertias[i] = deepestContact->objects[i]->body->invMass;

		totalInertia += linearInertias[i] + angularInertias[i];
	}

	/* compute the desired movement */
	float inverseInertia = 1.0f / totalInertia;
	float linearMovement[2] = {
		deepestContact->penetration * linearInertias[0] * inverseInertia,
		-deepestContact->penetration * linearInertias[1] * inverseInertia
	};
	float angularMovement[2] = {
		deepestContact->penetration * angularInertias[0] * inverseInertia,
		-deepestContact->penetration * angularInertias[1] * inverseInertia
	};

	/* compute delta position and delta orientation and apply them */
	for (uint8_t i = 0; i < 2; i++) {
		if (deepestContact->objects[i]->body->isStatic)
			continue;

		/* restrict angular movement for stability reasons -- smaller objects should have smaller rotation */
		/* limit wrt to the length of the relative contact position as an approximation of object's size */
		float angularMovementLimit = PhysicsSettings::get().collisionResolution.angularMovementLimitFactor * glm::length(deepestContact->relativeContactPositions[i]);
		if (glm::abs(angularMovement[i]) > angularMovementLimit) {
			float totalMove = linearMovement[i] + angularMovement[i];
			angularMovement[i] = glm::clamp(angularMovement[i], -angularMovementLimit, angularMovementLimit);
			linearMovement[i] = totalMove - angularMovement[i];
		}

		/* compute linear and angular deltas */
		glm::vec3 linearDelta = deepestContact->normal * linearMovement[i];
		glm::vec3 angularDelta = glm::vec3(0);
		glm::vec3 rotationDirection = glm::cross(deepestContact->relativeContactPositions[i], deepestContact->normal);
		angularDelta = (deepestContact->objects[i]->body->invInertiaTensorWorld * rotationDirection) * (angularMovement[i] / angularInertias[i]);

		/* apply the changes to the object */
		deepestContact->objects[i]->body->position += linearDelta;
		glm::quat aux = glm::quat(0, angularDelta); aux *= deepestContact->objects[i]->body->orientation;
		aux = deepestContact->objects[i]->body->orientation + 0.5f * aux;
		deepestContact->objects[i]->body->orientation = glm::normalize(aux);
		/*deepestContact->objects[i]->body->updateTransformMatrix();
		deepestContact->objects[i]->body->updateInvInertiaTensorWorld();*/

		/* apply the changes to this contact itself and all the other contacts in the manifold */
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

void SequentialImpulseContactResolver::applyVelocityUpdate(ContactManifold & manifold, Contact * fastestContact)
{
	/* compute the required impulse to solve this contact */
	glm::vec3 impulse = computeImpulse(fastestContact);

	/* compute the delta velocities and apply them */
	for (uint8_t i = 0; i < 2; i++) {
		if (fastestContact->objects[i]->body->isStatic)
			continue;

		/* linear velocity delta */
		const glm::vec3 velDelta = (i == 0 ? 1.0f : -1.0f) * impulse * fastestContact->objects[i]->body->invMass;
		fastestContact->objects[i]->body->linVelocity += velDelta;

		/* angular velocity delta */
		glm::vec3 angVelDelta = fastestContact->objects[i]->body->invInertiaTensorWorld * (i == 0 ? 1.0f : -1.0f) * glm::cross(fastestContact->relativeContactPositions[i], impulse);
		fastestContact->objects[i]->body->angVelocity += angVelDelta;

		/* apply the changes to this contact itself and all the other contacts in the manifold */
		for (auto &contact : manifold.contacts) {
			const uint8_t colliderIdxInOtherContact = contact->objects[0] == fastestContact->objects[i] ? 0 : 1;
			const float sign = colliderIdxInOtherContact == 0 ? -1.0f : 1.0f;

			const glm::vec3 delta = velDelta + glm::cross(angVelDelta, contact->relativeContactPositions[colliderIdxInOtherContact]);
			contact->closingVelocityWorld += sign * delta;
			contact->closingVelocityContact = contact->matWorldToContact * contact->closingVelocityWorld;
			contact->desiredDeltaVelocity = computeDesiredDeltaVelocity(contact);
		}
	}
}

void SequentialImpulseContactResolver::solveContactManifold(ContactManifold &manifold, float deltaTime)
{
	unsigned int penetrationIterations = 0;
	unsigned int velocityIterations = 0;

	for (auto & contact : manifold.contacts) {
		for (uint8_t i = 0; i < 2; i++) {
			if (contact->objects[i]->collider->body != nullptr)
				contact->objects[i]->collider->body->isAwake = true;
		}

		contact->computeDerivedData(deltaTime);
		contact->desiredDeltaVelocity = computeDesiredDeltaVelocity(contact);
	}



	/* fix interpenetration */
	for (penetrationIterations = 0; penetrationIterations < PhysicsSettings::get().collisionResolution.PEN_MAX_ITERATIONS; penetrationIterations++) {
		/* get the contact with deepest penetration */
		Contact *deepestContact = manifold.getDeepestContact();

		if (deepestContact == nullptr || deepestContact->penetration < PhysicsSettings::get().epsilons.PEN_EPSILON) {
			penetrationIterations++;
			break;
		}

		/* compute and apply position update */
		applyPositionUpdate(manifold, deepestContact);
	}

	/* fix velocities */
	for (velocityIterations = 0; velocityIterations < PhysicsSettings::get().collisionResolution.VEL_MAX_ITERATIONS; velocityIterations++) {
		/* get the contact with the lowest desired delta velocity (aka the fastest one) */
		Contact *fastestContact = manifold.getFastestContact();

		if (fastestContact == nullptr || fastestContact->desiredDeltaVelocity >= PhysicsSettings::get().epsilons.VEL_EPSILON) {
			velocityIterations++;
			break;
		}

		/* compute and apply velocity update */
		applyVelocityUpdate(manifold, fastestContact);
	}
}

void SequentialImpulseContactResolver::solve(const std::vector<CollisionPoint*> &collisions, float deltaTime)
{
	updateContacts(collisions);

	for (auto &manifold : manifolds) {
		solveContactManifold(manifold, deltaTime);
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

	/* Compute the areas of all possible triangle permutations. */

	const glm::vec3 v0v1 = manifold.contacts[0]->points[0] - manifold.contacts[1]->points[0];
	const glm::vec3 v0v2 = manifold.contacts[0]->points[0] - manifold.contacts[2]->points[0];
	const glm::vec3 v0v3 = manifold.contacts[0]->points[0] - manifold.contacts[3]->points[0];
	const glm::vec3 v1v2 = manifold.contacts[1]->points[0] - manifold.contacts[2]->points[0];
	const glm::vec3 v1v3 = manifold.contacts[1]->points[0] - manifold.contacts[3]->points[0];
	const glm::vec3 v2v3 = manifold.contacts[2]->points[0] - manifold.contacts[3]->points[0];

	float *areas = new float[PhysicsSettings::get().collisionResolution.maxContactsPerManifold];

	/* Compute the areas for each triangle formed by excluding the vertex at the area index. */
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

Contact * ContactManifold::getDeepestContact()
{
	Contact *deepestContact = nullptr;
	for (auto & contact : contacts)
		if (deepestContact == nullptr || contact->penetration > deepestContact->penetration)
			deepestContact = contact;
	
	return deepestContact;
}

Contact * ContactManifold::getFastestContact()
{
	Contact *fastestContact = nullptr;
	for (auto & contact : contacts) {
		if (contact->penetration < -PhysicsSettings::get().epsilons.PEN_EPSILON)
			continue;
		if (fastestContact == nullptr || contact->desiredDeltaVelocity < fastestContact->desiredDeltaVelocity)
			fastestContact = contact;
	}
	return fastestContact;
}
