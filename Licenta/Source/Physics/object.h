#pragma once

#include <string>
#include <Core/GPU/Mesh.h>
#include "rigidbody.h"
#include "broadphase.h"
#include "shapes.h"

class PhysicsObject
{
public:
	std::string name;
	RigidBody *body;
	
	Mesh *mesh;
	glm::vec3 color;

	Shape *shape;

	Collider *collider;

	void update(float deltaTime);
	glm::mat4 getTransformMatrix(); /* returns the local to world transform matrix of the object */
	glm::mat4 getColTransformMatrix(); /* returns the local to world transform matrix of the object colldier */

	std::string toString();
};

