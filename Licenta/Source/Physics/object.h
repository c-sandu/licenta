#pragma once

#include <string>
#include <Core/GPU/Mesh.h>
#include "rigidbody.h"
#include "broadphase.h"

class PhysicsObject
{
public:
	std::string name;
	RigidBody *body;
	
	Mesh *mesh;
	glm::vec3 color;

	Collider *collider;

	void update(float deltaTime);
	glm::mat4 getTransformMatrix();
	glm::mat4 getColTransformMatrix();

	std::string toString();
};

