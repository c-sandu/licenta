#include "contact.h"

#include "rigidbody.h"

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
void Contact::computeDerivedData()
{
	matContactToWorld = makeOrthonormalBasis(normal);
	matWorldToContact = glm::transpose(matContactToWorld);

	closingVelocityWorld = glm::vec3(0);
	closingVelocityContact = glm::vec3(0);

	for (uint8_t i = 0; i < 2; i++) {
		relativeContactPositions[i] = points[i] - objects[i]->body->position;
		if (objects[i]->body == NULL)
			return;

		glm::vec3 lastFramePlanarAcceleration = objects[i]->body->lastFrameAcceleration - glm::dot(objects[i]->body->lastFrameAcceleration, normal) * normal;
		glm::vec3 localVelocityWorld = glm::cross(objects[i]->body->angVelocity, relativeContactPositions[i]) + objects[i]->body->linVelocity + lastFramePlanarAcceleration;
		
		closingVelocityWorld += (i == 0 ? -1.0f : 1.0f) * localVelocityWorld;
	}
}
