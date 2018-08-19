#include "object.h"

void PhysicsObject::update(float deltaTime)
{
	if (body)
		body->integrate(deltaTime);
	collider->updateInternals();
}

glm::mat4 PhysicsObject::getTransformMatrix() const
{
	if (body == NULL)
		return glm::translate(glm::mat4(1), ((PlaneCollider*)collider)->normal * ((PlaneCollider*)collider)->offset);

	return body->transform;
}

glm::mat4 PhysicsObject::getColTransformMatrix()
{
	return collider->getTransformMatrix();
}

std::string PhysicsObject::toString()
{
	return std::string("") + "PhysicsObject {" + "\n\t"
		+ "name = " + name + "\n\t"
		+ "body = " + (body != NULL ? body->toString() : "NULL" ) + "\n\t"
		+ "collider = " + collider->toString() + "\n}\n";
}
