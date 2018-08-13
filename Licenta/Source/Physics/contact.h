#pragma once

#include <glm/glm.hpp>
#include <GLFW/glfw3.h>
#include <vector>
#include <string>
#include "object.h"

class ContactInfo
{
public:
	glm::vec3 points[2]; /* contact points on the 2 objects in world coordinates */
	glm::vec3 normal; /* contact normal */
	float penetration; /* penetration depth */

	PhysicsObject *objects[2]; /* {objA, objB} */

	ContactInfo() {}
	ContactInfo(const ContactInfo &other)
		: points{ other.points[0], other.points[1] },
		normal(other.normal),
		penetration(other.penetration),
		objects{ other.objects[0], other.objects[1] }
	{}
	ContactInfo(const glm::vec3 point1, const glm::vec3 point2, const glm::vec3 normal, const float penetration)
		: points{ point1, point2 },
			normal(normal),
			penetration(penetration)
	{}

	ContactInfo reverse() const {
		return ContactInfo(points[1], points[0], -normal, penetration);
	}
	std::string toString();
};

static unsigned int globalContactID = 0;

class Contact : public ContactInfo
{
public:
	glm::vec3 localPoints[2];
	/* derived data below */
	glm::mat3 matContactToWorld;
	glm::mat3 matWorldToContact;
	glm::vec3 closingVelocityWorld;
	glm::vec3 closingVelocityContact;
	glm::vec3 relativeContactPositions[2];

	bool invalid;

	unsigned int ContactID;

	float restitutionCoef;
	float frictionCoef;
	float desiredDeltaVelocity;

	double timestamp;

	std::vector<Contact*> *contactManifold;

	Contact()
	{
		this->restitutionCoef = 0.25f;
		this->frictionCoef = 0.00001f;
		this->invalid = false;

		this->ContactID = globalContactID++;
		timestamp = 0;
	}

	Contact(const ContactInfo &contactInfo)
	{
		this->points[0] = contactInfo.points[0];
		this->points[1] = contactInfo.points[1];
		this->localPoints[0] = glm::vec3(glm::inverse(contactInfo.objects[0]->getTransformMatrix()) * glm::vec4(this->points[0], 1));
		this->localPoints[1] = glm::vec3(glm::inverse(contactInfo.objects[1]->getTransformMatrix()) * glm::vec4(this->points[1], 1));

		this->normal = contactInfo.normal;
		this->penetration = contactInfo.penetration;
		this->objects[0] = contactInfo.objects[0];
		this->objects[1] = contactInfo.objects[1];

		this->restitutionCoef = 0.25f;
		this->frictionCoef = 0.00001f;
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
	void setContactInfo(const ContactInfo &contactInfo);

	void computeDerivedData();
	
	std::string toString();
};