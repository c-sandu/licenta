#include "contact.h"

#include <Physics/body/rigidbody.h>
#include <glm/gtx/string_cast.hpp>


/* From Ian Millington - Game Physics Engine Development (2007) - chapter 14.1.2 */
static glm::mat3 makeOrthonormalBasis(const glm::vec3 & contactNormal)
{
	glm::vec3 contactTangent[2];

	if (glm::abs(contactNormal.x) > glm::abs(contactNormal.y)) {
		const float s = 1.0f / glm::sqrt(contactNormal.z * contactNormal.z + contactNormal.x * contactNormal.x);

		contactTangent[0].x = contactNormal.z * s;
		contactTangent[0].y = 0;
		contactTangent[0].z = -contactNormal.x * s;

		contactTangent[1].x = contactNormal.y * contactTangent[0].x;
		contactTangent[1].y = contactNormal.z * contactTangent[0].x - contactNormal.x * contactTangent[0].z;
		contactTangent[1].z = -contactNormal.y * contactTangent[0].x;
	} else {
		const float s = 1.0f / glm::sqrt(contactNormal.z * contactNormal.z + contactNormal.y * contactNormal.y);

		contactTangent[0].x = 0;
		contactTangent[0].y = -contactNormal.z * s;
		contactTangent[0].z = contactNormal.y * s;

		contactTangent[1].x = contactNormal.y * contactTangent[0].z - contactNormal.z * contactTangent[0].y;
		contactTangent[1].y = -contactNormal.x * contactTangent[0].z;
		contactTangent[1].z = contactNormal.x * contactTangent[0].y;
	}
	return glm::mat3(contactNormal, contactTangent[0], contactTangent[1]);
}
void Contact::setContactInfo(const glm::vec3 point1, const glm::vec3 point2,
	const glm::vec3 normal,
	const float penetration,
	PhysicsObject *obj1, PhysicsObject *obj2)
{
	this->points[0] = point1;
	this->points[1] = point2;
	this->localPoints[0] = glm::vec3(glm::inverse(obj1->getTransformMatrix()) * glm::vec4(this->points[0], 1));
	this->localPoints[1] = glm::vec3(glm::inverse(obj2->getTransformMatrix()) * glm::vec4(this->points[1], 1));

	this->normal = normal;
	this->penetration = penetration;
	this->objects[0] = obj1;
	this->objects[1] = obj2;
}
void Contact::setContactInfo(const CollisionPoint & contactInfo)
{
	this->points[0] = contactInfo.points[0];
	this->points[1] = contactInfo.points[1];
	this->localPoints[0] = glm::vec3(glm::inverse(contactInfo.objects[0]->getTransformMatrix()) * glm::vec4(this->points[0], 1));
	this->localPoints[1] = glm::vec3(glm::inverse(contactInfo.objects[1]->getTransformMatrix()) * glm::vec4(this->points[1], 1));
	this->normal = contactInfo.normal;
	this->penetration = contactInfo.penetration;
	this->objects[0] = contactInfo.objects[0];
	this->objects[1] = contactInfo.objects[1];
}
void Contact::computeDerivedData(float deltaTime)
{
	this->restitutionCoef = glm::mix(
		std::min(objects[0]->body->restitutionCoef, objects[1]->body->restitutionCoef),
		std::max(objects[0]->body->restitutionCoef, objects[1]->body->restitutionCoef),
		PhysicsSettings::get().collisionResolution.coefInterpAlpha);
	this->frictionCoef = glm::mix(
		std::min(objects[0]->body->frictionCoef, objects[1]->body->frictionCoef),
		std::max(objects[0]->body->frictionCoef, objects[1]->body->frictionCoef),
		PhysicsSettings::get().collisionResolution.coefInterpAlpha);

	matContactToWorld = makeOrthonormalBasis(normal);
	matWorldToContact = glm::transpose(matContactToWorld);

	closingVelocityWorld = glm::vec3(0);
	closingVelocityContact = glm::vec3(0);

	for (uint8_t i = 0; i < 2; i++) {
		relativeContactPositions[i] = points[i] - objects[i]->body->position;
		if (objects[i]->body == nullptr)
			return;

		glm::vec3 lastFramePlanarAcceleration = objects[i]->body->lastFrameAcceleration - glm::dot(objects[i]->body->lastFrameAcceleration, normal) * normal;
		glm::vec3 lastFramePlanarVelocityFromAcc = lastFramePlanarAcceleration * deltaTime;
		glm::vec3 localVelocityWorld = glm::cross(objects[i]->body->angVelocity, relativeContactPositions[i]) + objects[i]->body->linVelocity + lastFramePlanarVelocityFromAcc;
		
		closingVelocityWorld += (i == 0 ? -1.0f : 1.0f) * localVelocityWorld;
	}
	closingVelocityContact = matWorldToContact * closingVelocityWorld;
}

std::string Contact::toString()
{
	return std::string("") + "\tContact {" + "\n\t\t"
		"ContactInfo {" + "\n\t\t"
		+ "pointA = " + to_string(points[0]) + "\n\t\t"
		+ "pointB = " + to_string(points[1]) + "\n\t\t"
		+ "normal = " + to_string(normal) + "\n\t\t"
		+ "penetration = " + std::to_string(penetration) + "\n\t\t"
		+ "objects = { " + objects[0]->name + ", " + objects[1]->name + " }\n}\n\t\t"
		+ "matContactToWorld = " + to_string(matContactToWorld) + "\n\t\t"
		+ "matWorldToContact = " + to_string(matWorldToContact) + "\n\n\t\t"
		+ "closingVelocityWorld = " + to_string(closingVelocityWorld) + "\n\t\t"
		+ "closingVelocityContact = " + to_string(closingVelocityContact) + "\n\t\t"
		+ "relativeContactPositions = { " + to_string(relativeContactPositions[0]) + ", " + to_string(relativeContactPositions[1]) + " }\n\n\t\t"
		+ "desiredDeltaVelocity = " + std::to_string(desiredDeltaVelocity) + "\n\t}\n";
}

std::string CollisionPoint::toString()
{
	return std::string("") + "CollisionPoint {" + "\n\t"
		+ "pointA = " + to_string(points[0]) + "\n\t"
		+ "pointB = " + to_string(points[1]) + "\n\t"
		+ "normal = " + to_string(normal) + "\n\t"
		+ "penetration = " + std::to_string(penetration) + "\n\t"
		+ "objects = { " + objects[0]->name + ", " + objects[1]->name + " }\n}\n";
}
