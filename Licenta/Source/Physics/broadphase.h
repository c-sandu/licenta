#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

#include "rigidbody.h"
#include <list>

/* Forward declaration for pointer to the object the collider is associated to. */
class PhysicsObject;
class Contact;

class Collider
{
public:
	PhysicsObject *phyObject;
	RigidBody *body;
	std::string meshName;

	std::list<Contact*> *contactManifold;

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

	PlaneCollider(glm::vec3 normal, float offset, PhysicsObject *phyObject);
	void updateInternals();

	glm::mat4 getTransformMatrix();
	std::string toString();
};


/* Oriented bounding box collider used for broad phase collision detection. */
class OBBCollider : public Collider
{
public:
	glm::vec3 halfSizes; /* (width, height, depth) */
	glm::vec3 position; /* position of the box's center in world coordinates */
	glm::quat orientation; /* orientation of the box */

	OBBCollider(const glm::vec3 & halfSizes, PhysicsObject *phyObject);

	void updateInternals();

	bool testIntersectionOBB(OBBCollider & other);
	bool testIntersectionPlane(PlaneCollider & plane);

	glm::mat4 getTransformMatrix();
	std::string toString();
};


class PotentialCollision
{
public:
	PhysicsObject *one;
	PhysicsObject *two;

	PotentialCollision(PhysicsObject *one, PhysicsObject *two);
};

class PotentialCollisionDetector
{
private:
	std::vector<PlaneCollider*> planeVector;
	std::vector<OBBCollider*> obbVector;

public:
	std::vector<PotentialCollision*> potentialCollisions;
	void addCollider(PlaneCollider *plane);
	void addCollider(OBBCollider *obb);

	void clearPotentialCollisions();
	void fillPotentialCollisions();
};