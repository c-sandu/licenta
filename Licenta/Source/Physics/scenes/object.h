#pragma once

#include <string>
#include <Core/GPU/Mesh.h>
#include <Physics/body/rigidbody.h>
#include <Physics/collision/detection/broadphase.h>
#include <Physics/body/shapes.h>
#include <unordered_map>

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

	~PhysicsObject();
};

class ObjectSpawner
{
public:
	std::list<PhysicsObject*> objects;
	std::unordered_map<std::string, Mesh*> *meshes;
	PotentialCollisionDetector *pcd;

	glm::vec3 spawnPosition;
	glm::vec3 spawnDirection;
	float offsetInFront;
	bool applyStartingImpulse;

	PhysicsObject *selectedObject;

	enum ObjectType { box=0, longBox=1, sphere=2, cylinder=3, capsule=4 };
	ObjectType nextObject;
	bool randomizeNextObject;
	bool randomizeProperties;

	bool mustSpawnObject;

	ObjectSpawner(std::unordered_map<std::string, Mesh*> *meshes, PotentialCollisionDetector *pcd);

	void updateFromCamera(glm::vec3 cameraPosition, glm::vec3 forward);

	void updateObjects(float deltaTime);

	void spawnNewObject();


	void spawnBoxDynamic();
	void spawnBoxStatic(glm::vec3 position, glm::vec3 scale = glm::vec3(1), float mass = PhysicsSettings::get().rigidBodies.defaultMass, glm::quat orientation = glm::quat(1, 0, 0, 0));
	void spawnLongBoxDynamic();
	void spawnLongBoxStatic(glm::vec3 position, glm::vec3 scale = glm::vec3(1), float mass = PhysicsSettings::get().rigidBodies.defaultMass, glm::quat orientation = glm::quat(1, 0, 0, 0));
	void spawnWallStatic(glm::vec3 position, glm::vec3 scale = glm::vec3(1), glm::quat orientation = glm::quat(1, 0, 0, 0));
	void spawnSphereDynamic();
	void spawnSphereStatic(glm::vec3 position, glm::vec3 scale = glm::vec3(1), float mass = PhysicsSettings::get().rigidBodies.defaultMass, glm::quat orientation = glm::quat(1, 0, 0, 0));
	void spawnCylinderDynamic();
	void spawnCylinderStatic(glm::vec3 position, glm::vec3 scale = glm::vec3(1), float mass = PhysicsSettings::get().rigidBodies.defaultMass, glm::quat orientation = glm::quat(1, 0, 0, 0));
	void spawnCapsuleDynamic();
	void spawnCapsuleStatic(glm::vec3 position, glm::vec3 scale = glm::vec3(1), float mass = PhysicsSettings::get().rigidBodies.defaultMass, glm::quat orientation = glm::quat(1, 0, 0, 0));

	std::string toString();
};