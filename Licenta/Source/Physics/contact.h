#pragma once

#include <glm/glm.hpp>
#include <vector>
#include "object.h"

class ContactInfo
{
public:
	glm::vec3 points[2]; /* {pointA, pointB} in world coordinates*/
	glm::vec3 normal; /* contact normal */
	float penetration; /* penetration depth */

	PhysicsObject *objects[2]; /* {objA, objB} */
};

class Contact : public ContactInfo
{
public:
	/* derived data below */
	glm::mat3 matContactToWorld;
	glm::mat3 matWorldToContact;
	glm::vec3 closingVelocityWorld;
	glm::vec3 closingVelocityContact;
	glm::vec3 relativeContactPositions[2];

	bool invalid;

	float restitutionCoef;
	float frictionCoef;
	float desiredDeltaVelocity;

	std::vector<Contact*> *contactManifold;

	Contact(const ContactInfo &contactInfo)
	{
		this->points[0] = contactInfo.points[0];
		this->points[1] = contactInfo.points[1];
		this->normal = contactInfo.normal;
		this->penetration = contactInfo.penetration;
		this->objects[0] = contactInfo.objects[0];
		this->objects[1] = contactInfo.objects[1];

		this->restitutionCoef = 1.0f;
		this->frictionCoef = 1.0f;
		this->invalid = false;
	}

	void computeDerivedData();
};