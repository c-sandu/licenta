#pragma once

#include <string>
#include <Core/GPU/Mesh.h>
#include <Physics/body/rigidbody.h>
#include <Physics/collision/detection/broadphase.h>
#include <Physics/body/shapes.h>

class PhysicsObject
{
public:
	std::string name; /* public name of the object */
	RigidBody *body; /* rigid body component attached to the object */
	
	/* rendering properties */
	Mesh *mesh; /* mesh used only for drawing */
	glm::vec3 color; /* object's color */

	Shape *shape; /* object's shape, used for support point calculation */
	Collider *collider; /* object's collider used in broad phase collision detection */


	void update(float deltaTime); /* performs one frame's worth of updating the object state */
	glm::mat4 getTransformMatrix() const; /* returns the local to world transform matrix of the object */
	glm::mat4 getColTransformMatrix(); /* returns the local to world transform matrix of the object's colldier */

	std::string toString();
};

