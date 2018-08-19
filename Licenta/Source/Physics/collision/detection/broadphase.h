#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

#include <Core/GPU/Mesh.h>

#include <Physics/body/rigidbody.h>
#include <list>

/* Forward declaration for pointer to the object the collider is associated to. */
class PhysicsObject;
class Contact;

/* Abstract class for a generic collider */
class Collider
{
public:
	PhysicsObject *phyObject; /* the object this collider is attached to */
	RigidBody *body; /* the rigid body component of the object this collider is attached to */
	Mesh *mesh; /* mesh used for drawing the collider */

	virtual void updateInternals() = 0;
	void setRigidBody(RigidBody *body);

	virtual glm::mat4 getTransformMatrix() = 0;
	virtual std::string toString() = 0;
};

class PlaneCollider : public Collider
{
public:
	glm::vec3 normal; /* plane's normal */
	float offset; /* offset from the origin of the world coordinate system */

	PlaneCollider(glm::vec3 normal, float offset, PhysicsObject *phyObject, Mesh * mesh);
	void updateInternals();

	glm::mat4 getTransformMatrix();
	std::string toString();
};


/* Oriented bounding box collider used for broad phase collision detection. */
class OBBCollider : public Collider
{
public:
	glm::vec3 halfSizes; /* halfsizes (width, height, depth) */
	
private:
	glm::vec3 position; /* position of the box's center in world coordinates */
	glm::quat orientation; /* orientation of the box */

public:
	OBBCollider(const glm::vec3 & halfSizes, PhysicsObject *phyObject, Mesh * mesh);

	/* updates position and orientation of the collider */
	void updateInternals();

	/* tests intersection with another OBB */
	bool testIntersectionOBB(OBBCollider & other);
	bool testIntersectionPlane(PlaneCollider & plane);

	/* returns collider's trasnform matrix for rendering purposes */
	glm::mat4 getTransformMatrix();
	std::string toString();
};


/* Struct that holds a pair of objects to further be tested for intersection */
struct PotentialCollision
{
	PhysicsObject *one;
	PhysicsObject *two;

	PotentialCollision(PhysicsObject *one, PhysicsObject *two) : one(one), two(two) {};
};

/* O(n^2) broad phase collision detector */
class PotentialCollisionDetector
{
private:
	std::vector<PlaneCollider*> planeVector; /* plane colliders assigned to this detector */
	std::vector<OBBCollider*> obbVector; /* OBB colliders assigned to this detector */

public:
	std::vector<PotentialCollision*> potentialCollisions;
	void addCollider(PlaneCollider *plane);
	void addCollider(OBBCollider *obb);

	void clearPotentialCollisions();
	void fillPotentialCollisions();
};