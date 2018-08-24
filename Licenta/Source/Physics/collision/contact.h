#pragma once

#include <glm/glm.hpp>
#include <GLFW/glfw3.h>
#include <vector>
#include <string>
#include <Physics/scenes/object.h>

/* describes a collision point between two objects */
class CollisionPoint
{
public:
	glm::vec3 points[2]; /* contact points on the 2 objects in world coordinates */
	glm::vec3 normal; /* collision normal (from first object to the second object) */
	float penetration; /* penetration depth */

	PhysicsObject *objects[2]; /* {objA, objB} */

	CollisionPoint() {}
	CollisionPoint(const CollisionPoint &other)
		: points{ other.points[0], other.points[1] },
		normal(other.normal),
		penetration(other.penetration),
		objects{ other.objects[0], other.objects[1] }
	{}
	CollisionPoint(const glm::vec3 point1, const glm::vec3 point2, const glm::vec3 normal, const float penetration)
		: points{ point1, point2 },
			normal(normal),
			penetration(penetration)
	{}

	/* reverses the direction of the collision */
	CollisionPoint reverse() const {
		return CollisionPoint(points[1], points[0], -normal, penetration);
	}
	std::string toString();
};

static unsigned int globalContactID = 0; /* global contact id */

/* describes a contact as it is used by the contact resolver */
class Contact : public CollisionPoint
{
	friend class ContactManifold;
	friend class SequentialImpulseContactResolver;
private:
	glm::vec3 localPoints[2]; /* points in the individual objects' local spaces */
	/* derived data below */
public:
	glm::mat3 matContactToWorld; /* transform matrix from contact space to world space */
private:
	glm::mat3 matWorldToContact; /* transform matrix from world space to contact space */
	glm::vec3 closingVelocityWorld; /* closing velocity in world space */
	glm::vec3 closingVelocityContact; /* closing velocity in contact space */
	glm::vec3 relativeContactPositions[2];

	bool invalid;

	unsigned int ContactID;

	float restitutionCoef;
	float frictionCoef;
	float desiredDeltaVelocity;

	unsigned int timestamp;

public:
	Contact()
	{
		this->restitutionCoef = PhysicsSettings::get().rigidBodies.defaultRestitutionCoef;
		this->frictionCoef = PhysicsSettings::get().rigidBodies.defaultFrictionCoef;
		this->invalid = false;

		this->ContactID = globalContactID++;
		timestamp = 0;
	}

	Contact(const CollisionPoint &collisionPoint)
	{
		this->points[0] = collisionPoint.points[0];
		this->points[1] = collisionPoint.points[1];
		this->localPoints[0] = glm::vec3(glm::inverse(collisionPoint.objects[0]->getTransformMatrix()) * glm::vec4(this->points[0], 1));
		this->localPoints[1] = glm::vec3(glm::inverse(collisionPoint.objects[1]->getTransformMatrix()) * glm::vec4(this->points[1], 1));

		this->normal = collisionPoint.normal;
		this->penetration = collisionPoint.penetration;
		this->objects[0] = collisionPoint.objects[0];
		this->objects[1] = collisionPoint.objects[1];

		this->restitutionCoef = glm::mix(
			std::min(objects[0]->body->restitutionCoef, objects[1]->body->restitutionCoef),
			std::max(objects[0]->body->restitutionCoef, objects[1]->body->restitutionCoef),
			PhysicsSettings::get().collisionResolution.coefInterpAlpha);
		this->frictionCoef = glm::mix(
			std::min(objects[0]->body->frictionCoef, objects[1]->body->frictionCoef),
			std::max(objects[0]->body->frictionCoef, objects[1]->body->frictionCoef),
			PhysicsSettings::get().collisionResolution.coefInterpAlpha);
		this->invalid = false;

		this->ContactID = globalContactID++;

		timestamp = 0;
	}

	bool operator==(const Contact &other) {
		return this->ContactID == other.ContactID;
	}

	void setContactInfo(const glm::vec3 point1, const glm::vec3 point2,
		const glm::vec3 normal,
		const float penetration,
		PhysicsObject *obj1, PhysicsObject *obj2);
	void setContactInfo(const CollisionPoint &contactInfo);

	
	std::string toString();

private:
	void computeDerivedData(float deltaTime);
};