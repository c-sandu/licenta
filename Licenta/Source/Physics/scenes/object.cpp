#include "object.h"

#include <cstdlib>
#include <glm/gtc/random.hpp>
#include <glm/gtx/string_cast.hpp>

void PhysicsObject::update(float deltaTime)
{
	if (body)
		body->integrate(deltaTime);
	collider->updateInternals();
}

glm::mat4 PhysicsObject::getTransformMatrix() const
{
	if (body == nullptr)
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
		+ "body = " + (body != nullptr ? body->toString() : "NULL" ) + "\n\t"
		+ "collider = " + collider->toString() + "\n}\n";
}

PhysicsObject::~PhysicsObject()
{
	collider->markedForDeletion = true;
	delete body;
	delete shape;
	body = nullptr;
	shape = nullptr;
}

ObjectSpawner::ObjectSpawner(std::unordered_map<std::string, Mesh*> *meshes, PotentialCollisionDetector *pcd)
	: meshes(meshes), pcd(pcd)
{
	spawnPosition = glm::vec3(0, 6, 0);
	spawnDirection = glm::vec3(0, 0, 0);
	offsetInFront = 1.0f;
	applyStartingImpulse = false;

	randomizeNextObject = true;
	randomizeProperties = false;

	mustSpawnObject = false;
	nextObject = box;
}

void ObjectSpawner::updateFromCamera(glm::vec3 cameraPosition, glm::vec3 forward)
{
	spawnDirection = glm::normalize(forward);
	spawnPosition = cameraPosition + spawnDirection * offsetInFront;
}

void ObjectSpawner::updateObjects(float deltaTime)
{
	
	for (auto objIt = objects.begin(); objIt != objects.end();) {
		float length = glm::length((*objIt)->body->position);
		if (isnan(length) || length > 100) {
			bool isSelectedObject = false;
			if (selectedObject == *objIt)
				isSelectedObject = true;
			delete *objIt;
			objIt = objects.erase(objIt);
			if (isSelectedObject)
				selectedObject = objects.front();
			continue;
		}

		(*objIt)->update(deltaTime);
		objIt++;
	}

	if (mustSpawnObject) {
		spawnNewObject();
		mustSpawnObject = false;
	}
}

void ObjectSpawner::spawnNewObject()
{
	switch (nextObject) {
	case box:
		spawnBoxDynamic();
		break;
	case longBox:
		spawnLongBoxDynamic();
		break;
	case sphere:
		spawnSphereDynamic();
		break;
	case cylinder:
		spawnCylinderDynamic();
		break;
	case capsule:
		spawnCapsuleDynamic();
		break;
	}

	if (randomizeNextObject)
		nextObject = ObjectType(rand() % 5);
}

void ObjectSpawner::spawnBoxDynamic()
{
	glm::vec3 scale(1);
	float mass = PhysicsSettings::get().rigidBodies.defaultMass;
	glm::quat orientation = glm::quat(1, 0, 0, 0);
	glm::vec3 color = glm::vec3(1, 1, 0);

	static unsigned int dynamicBoxID = 0;

	if (randomizeProperties) {
		float randFactor;
		randFactor = glm::linearRand(0.25f, 4.0f);
		scale *= randFactor;

		randFactor = glm::linearRand(0.25f, 4.0f);
		mass *= randFactor;

		glm::vec3 randDirection = glm::sphericalRand(1.0f);
		orientation = glm::normalize(glm::quat(1.0f, randDirection));

		randFactor = glm::linearRand(0.0f, 1.0f);
		color.x = randFactor;
		randFactor = glm::linearRand(0.0f, 1.0f);
		color.y = randFactor;
		randFactor = glm::linearRand(0.0f, 1.0f);
		color.z = randFactor;
	}
	RigidBody *cube = new RigidBody(spawnPosition, scale);
	cube->setMass(mass);
	cube->setInertiaTensor(RigidBody::inertiaTensorCube(PhysicsSettings::get().shapes.box.halfSizes * cube->scale));
	cube->orientation = orientation;
	cube->updateTransformMatrix();

	if (applyStartingImpulse)
		cube->applyLinearImpulse(spawnDirection * 10.0f);

	if (randomizeProperties)
		cube->applyTorqueAtLocalPoint(spawnDirection * 10.0f, glm::sphericalRand(2.0f));
	PhysicsObject *obj = new PhysicsObject();
	obj->body = cube;
	obj->mesh = (*meshes)["box"];
	obj->color = color;
	obj->collider = new OBBCollider(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale, obj, (*meshes)["box"]);
	obj->collider->setRigidBody(obj->body);
	obj->collider->updateInternals();
	obj->name.assign("dynamicBox" + std::to_string(dynamicBoxID++));
	obj->shape = new Box(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);
}

void ObjectSpawner::spawnBoxStatic(glm::vec3 position, glm::vec3 scale, float mass, glm::quat orientation) {
	glm::vec3 color = glm::vec3(1, 1, 0);

	static unsigned int staticBoxID = 0;

	float randFactor;
	randFactor = glm::linearRand(0.0f, 1.0f);
	color.x = randFactor;
	randFactor = glm::linearRand(0.0f, 1.0f);
	color.y = randFactor;
	randFactor = glm::linearRand(0.0f, 1.0f);
	color.z = randFactor;

	RigidBody *box = new RigidBody(position, scale);
	box->setMass(mass);
	box->setInertiaTensor(RigidBody::inertiaTensorCube(PhysicsSettings::get().shapes.box.halfSizes * box->scale));
	box->orientation = orientation;
	box->updateTransformMatrix();
	box->isAwake = false;

	PhysicsObject *obj = new PhysicsObject();
	obj->body = box;
	obj->mesh = (*meshes)["box"];
	obj->color = color;
	obj->collider = new OBBCollider(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale, obj, (*meshes)["box"]);
	obj->collider->setRigidBody(obj->body);
	obj->collider->updateInternals();
	obj->name.assign("staticBox" + std::to_string(staticBoxID++));
	obj->shape = new Box(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);
}

void ObjectSpawner::spawnLongBoxDynamic()
{
	glm::vec3 scale(1, 2, 1);
	float mass = PhysicsSettings::get().rigidBodies.defaultMass;
	glm::quat orientation = glm::quat(1, 0, 0, 0);
	glm::vec3 color = glm::vec3(1, 1, 0);

	static unsigned int dynamicLongBoxID = 0;

	if (randomizeProperties) {
		float randFactor;
		randFactor = glm::linearRand(0.25f, 4.0f);
		scale *= randFactor;

		randFactor = glm::linearRand(0.25f, 4.0f);
		mass *= randFactor;

		glm::vec3 randDirection = glm::sphericalRand(1.0f);
		orientation = glm::normalize(glm::quat(1.0f, randDirection));

		randFactor = glm::linearRand(0.0f, 1.0f);
		color.x = randFactor;
		randFactor = glm::linearRand(0.0f, 1.0f);
		color.y = randFactor;
		randFactor = glm::linearRand(0.0f, 1.0f);
		color.z = randFactor;
	}
	RigidBody *longBox = new RigidBody(spawnPosition, scale);
	longBox->setMass(mass);
	longBox->setInertiaTensor(RigidBody::inertiaTensorCube(PhysicsSettings::get().shapes.box.halfSizes * longBox->scale));
	longBox->orientation = orientation;
	longBox->updateTransformMatrix();

	if (applyStartingImpulse)
		longBox->applyLinearImpulse(spawnDirection * 10.0f);

	if (randomizeProperties)
		longBox->applyTorqueAtLocalPoint(spawnDirection * 10.0f, glm::sphericalRand(2.0f));
	PhysicsObject *obj = new PhysicsObject();
	obj->body = longBox;
	obj->mesh = (*meshes)["box"];
	obj->color = color;
	obj->collider = new OBBCollider(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale, obj, (*meshes)["box"]);
	obj->collider->setRigidBody(obj->body);
	obj->collider->updateInternals();
	obj->name.assign("dynamicLongBox" + std::to_string(dynamicLongBoxID++));
	obj->shape = new Box(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);
}

void ObjectSpawner::spawnLongBoxStatic(glm::vec3 position, glm::vec3 scale, float mass, glm::quat orientation)
{
	scale = scale * glm::vec3(1, 2, 1);
	glm::vec3 color = glm::vec3(1, 1, 0);

	static unsigned int staticLongBoxID = 0;

	float randFactor;
	randFactor = glm::linearRand(0.0f, 1.0f);
	color.x = randFactor;
	randFactor = glm::linearRand(0.0f, 1.0f);
	color.y = randFactor;
	randFactor = glm::linearRand(0.0f, 1.0f);
	color.z = randFactor;

	RigidBody *longBox = new RigidBody(position, scale);
	longBox->setMass(mass);
	longBox->setInertiaTensor(RigidBody::inertiaTensorCube(PhysicsSettings::get().shapes.box.halfSizes * longBox->scale));
	longBox->orientation = orientation;
	longBox->updateTransformMatrix();
	longBox->isAwake = false;

	PhysicsObject *obj = new PhysicsObject();
	obj->body = longBox;
	obj->mesh = (*meshes)["box"];
	obj->color = color;
	obj->collider = new OBBCollider(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale, obj, (*meshes)["box"]);
	obj->collider->setRigidBody(obj->body);
	obj->collider->updateInternals();
	obj->name.assign("staticLongBox" + std::to_string(staticLongBoxID++));
	obj->shape = new Box(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);
}

void ObjectSpawner::spawnWallStatic(glm::vec3 position, glm::vec3 scale, glm::quat orientation)
{
	scale = scale * glm::vec3(8, 8, 1);
	glm::vec3 color = glm::vec3(1, 1, 0);

	static unsigned int wallID = 0;

	float randFactor;
	randFactor = glm::linearRand(0.0f, 1.0f);
	color.x = randFactor;
	randFactor = glm::linearRand(0.0f, 1.0f);
	color.y = randFactor;
	randFactor = glm::linearRand(0.0f, 1.0f);
	color.z = randFactor;

	RigidBody *wall = new RigidBody(position, scale);
	wall->setMass(0.0f, true);
	wall->setInertiaTensor(RigidBody::inertiaTensorCube(PhysicsSettings::get().shapes.box.halfSizes * wall->scale));
	wall->orientation = orientation;
	wall->updateTransformMatrix();
	wall->isAwake = false;

	PhysicsObject *obj = new PhysicsObject();
	obj->body = wall;
	obj->mesh = (*meshes)["box"];
	obj->color = color;
	obj->collider = new OBBCollider(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale, obj, (*meshes)["box"]);
	obj->collider->setRigidBody(obj->body);
	obj->collider->updateInternals();
	obj->name.assign("wall" + std::to_string(wallID++));
	obj->shape = new Box(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);
}

void ObjectSpawner::spawnSphereDynamic()
{
	glm::vec3 scale(1);
	float mass = PhysicsSettings::get().rigidBodies.defaultMass;
	glm::quat orientation = glm::quat(1, 0, 0, 0);
	glm::vec3 color = glm::vec3(1, 1, 0);

	static unsigned int dynamicSphereID = 0;

	if (randomizeProperties) {
		float randFactor;
		randFactor = glm::linearRand(0.25f, 4.0f);
		scale *= randFactor;

		randFactor = glm::linearRand(0.25f, 4.0f);
		mass *= randFactor;

		glm::vec3 randDirection = glm::sphericalRand(1.0f);
		orientation = glm::normalize(glm::quat(1.0f, randDirection));

		randFactor = glm::linearRand(0.0f, 1.0f);
		color.x = randFactor;
		randFactor = glm::linearRand(0.0f, 1.0f);
		color.y = randFactor;
		randFactor = glm::linearRand(0.0f, 1.0f);
		color.z = randFactor;
	}
	RigidBody *sphere = new RigidBody(spawnPosition, scale);
	sphere->setMass(mass);
	sphere->setInertiaTensor(RigidBody::inertiaTensorSphere(PhysicsSettings::get().shapes.sphere.radius * sphere->scale.x));
	sphere->orientation = orientation;
	sphere->updateTransformMatrix();

	if (applyStartingImpulse)
		sphere->applyLinearImpulse(spawnDirection * 10.0f);

	if (randomizeProperties)
		sphere->applyTorqueAtLocalPoint(spawnDirection * 10.0f, glm::sphericalRand(2.0f));
	PhysicsObject *obj = new PhysicsObject();
	obj->body = sphere;
	obj->mesh = (*meshes)["sphere"];
	obj->color = color;
	obj->collider = new OBBCollider(PhysicsSettings::get().shapes.sphere.obbHalfSizes * obj->body->scale, obj, (*meshes)["box"]);
	obj->collider->setRigidBody(obj->body);
	obj->collider->updateInternals();
	obj->name.assign("dynamicSphere" + std::to_string(dynamicSphereID++));
	obj->shape = new Sphere(PhysicsSettings::get().shapes.sphere.radius * sphere->scale.x);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);
}

void ObjectSpawner::spawnSphereStatic(glm::vec3 position, glm::vec3 scale, float mass, glm::quat orientation)
{
	glm::vec3 color = glm::vec3(1, 1, 0);

	static unsigned int staticSphereID = 0;

	float randFactor;
	randFactor = glm::linearRand(0.0f, 1.0f);
	color.x = randFactor;
	randFactor = glm::linearRand(0.0f, 1.0f);
	color.y = randFactor;
	randFactor = glm::linearRand(0.0f, 1.0f);
	color.z = randFactor;

	RigidBody *sphere = new RigidBody(position, scale);
	sphere->setMass(mass);
	sphere->setInertiaTensor(RigidBody::inertiaTensorSphere(PhysicsSettings::get().shapes.sphere.radius * sphere->scale.x));
	sphere->orientation = orientation;
	sphere->updateTransformMatrix();
	sphere->isAwake = false;

	PhysicsObject *obj = new PhysicsObject();
	obj->body = sphere;
	obj->mesh = (*meshes)["sphere"];
	obj->color = color;
	obj->collider = new OBBCollider(PhysicsSettings::get().shapes.sphere.obbHalfSizes * obj->body->scale, obj, (*meshes)["box"]);
	obj->collider->setRigidBody(obj->body);
	obj->collider->updateInternals();
	obj->name.assign("staticSphere" + std::to_string(staticSphereID++));
	obj->shape = new Sphere(PhysicsSettings::get().shapes.sphere.radius * sphere->scale.x);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);
}

void ObjectSpawner::spawnCylinderDynamic()
{
	glm::vec3 scale(1);
	float mass = PhysicsSettings::get().rigidBodies.defaultMass;
	glm::quat orientation = glm::quat(1, 0, 0, 0);
	glm::vec3 color = glm::vec3(1, 1, 0);

	static unsigned int dynamicCylinderID = 0;

	if (randomizeProperties) {
		float randFactor;
		randFactor = glm::linearRand(0.25f, 4.0f);
		scale *= randFactor;

		randFactor = glm::linearRand(0.25f, 4.0f);
		mass *= randFactor;

		glm::vec3 randDirection = glm::sphericalRand(1.0f);
		orientation = glm::normalize(glm::quat(1.0f, randDirection));

		randFactor = glm::linearRand(0.0f, 1.0f);
		color.x = randFactor;
		randFactor = glm::linearRand(0.0f, 1.0f);
		color.y = randFactor;
		randFactor = glm::linearRand(0.0f, 1.0f);
		color.z = randFactor;
	}
	RigidBody *cylinder = new RigidBody(spawnPosition, scale);
	cylinder->setMass(mass);
	cylinder->setInertiaTensor(RigidBody::inertiaTensorCylinder(PhysicsSettings::get().shapes.cylinder.height * cylinder->scale.y, PhysicsSettings::get().shapes.cylinder.radius * cylinder->scale.x));
	cylinder->orientation = orientation;
	cylinder->updateTransformMatrix();

	if (applyStartingImpulse)
		cylinder->applyLinearImpulse(spawnDirection * 10.0f);

	if (randomizeProperties)
		cylinder->applyTorqueAtLocalPoint(spawnDirection * 10.0f, glm::sphericalRand(2.0f));
	PhysicsObject *obj = new PhysicsObject();
	obj->body = cylinder;
	obj->mesh = (*meshes)["cylinder"];
	obj->color = color;
	obj->collider = new OBBCollider(PhysicsSettings::get().shapes.cylinder.obbHalfSizes * obj->body->scale, obj, (*meshes)["box"]);
	obj->collider->setRigidBody(obj->body);
	obj->collider->updateInternals();
	obj->name.assign("dynamicCylinder" + std::to_string(dynamicCylinderID++));
	obj->shape = new Cylinder(PhysicsSettings::get().shapes.cylinder.height * cylinder->scale.y, PhysicsSettings::get().shapes.cylinder.radius * cylinder->scale.x);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);
}

void ObjectSpawner::spawnCylinderStatic(glm::vec3 position, glm::vec3 scale, float mass, glm::quat orientation)
{
	glm::vec3 color = glm::vec3(1, 1, 0);

	static unsigned int staticCylinderID = 0;

	float randFactor;
	randFactor = glm::linearRand(0.0f, 1.0f);
	color.x = randFactor;
	randFactor = glm::linearRand(0.0f, 1.0f);
	color.y = randFactor;
	randFactor = glm::linearRand(0.0f, 1.0f);
	color.z = randFactor;

	RigidBody *cylinder = new RigidBody(position, scale);
	cylinder->setMass(mass);
	cylinder->setInertiaTensor(RigidBody::inertiaTensorCylinder(PhysicsSettings::get().shapes.cylinder.height * cylinder->scale.y, PhysicsSettings::get().shapes.cylinder.radius * cylinder->scale.x));
	cylinder->orientation = orientation;
	cylinder->updateTransformMatrix();
	cylinder->isAwake = false;

	PhysicsObject *obj = new PhysicsObject();
	obj->body = cylinder;
	obj->mesh = (*meshes)["sphere"];
	obj->color = color;
	obj->collider = new OBBCollider(PhysicsSettings::get().shapes.cylinder.obbHalfSizes * obj->body->scale, obj, (*meshes)["box"]);
	obj->collider->setRigidBody(obj->body);
	obj->collider->updateInternals();
	obj->name.assign("staticCylinder" + std::to_string(staticCylinderID++));
	obj->shape = new Cylinder(PhysicsSettings::get().shapes.cylinder.height * cylinder->scale.y, PhysicsSettings::get().shapes.cylinder.radius * cylinder->scale.x);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);
}

void ObjectSpawner::spawnCapsuleDynamic()
{
	glm::vec3 scale(1);
	float mass = PhysicsSettings::get().rigidBodies.defaultMass;
	glm::quat orientation = glm::quat(1, 0, 0, 0);
	glm::vec3 color = glm::vec3(1, 1, 0);

	static unsigned int dynamicCapsuleID = 0;

	if (randomizeProperties) {
		float randFactor;
		randFactor = glm::linearRand(0.25f, 4.0f);
		scale *= randFactor;

		randFactor = glm::linearRand(0.25f, 4.0f);
		mass *= randFactor;

		glm::vec3 randDirection = glm::sphericalRand(1.0f);
		orientation = glm::normalize(glm::quat(1.0f, randDirection));

		randFactor = glm::linearRand(0.0f, 1.0f);
		color.x = randFactor;
		randFactor = glm::linearRand(0.0f, 1.0f);
		color.y = randFactor;
		randFactor = glm::linearRand(0.0f, 1.0f);
		color.z = randFactor;
	}
	RigidBody *capsule = new RigidBody(spawnPosition, scale);
	capsule->setMass(mass);
	capsule->setInertiaTensor(RigidBody::inertiaTensorCapsule(PhysicsSettings::get().shapes.capsule.height * capsule->scale.y, PhysicsSettings::get().shapes.capsule.radius * capsule->scale.x));
	capsule->orientation = orientation;
	capsule->updateTransformMatrix();

	if (applyStartingImpulse)
		capsule->applyLinearImpulse(spawnDirection * 10.0f);

	if (randomizeProperties)
		capsule->applyTorqueAtLocalPoint(spawnDirection * 10.0f, glm::sphericalRand(2.0f));
	PhysicsObject *obj = new PhysicsObject();
	obj->body = capsule;
	obj->mesh = (*meshes)["capsule"];
	obj->color = color;
	obj->collider = new OBBCollider(PhysicsSettings::get().shapes.capsule.obbHalfSizes * obj->body->scale, obj, (*meshes)["box"]);
	obj->collider->setRigidBody(obj->body);
	obj->collider->updateInternals();
	obj->name.assign("dynamicCapsule" + std::to_string(dynamicCapsuleID++));
	obj->shape = new Capsule(PhysicsSettings::get().shapes.capsule.height * capsule->scale.y, PhysicsSettings::get().shapes.capsule.radius * capsule->scale.x);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);
}

void ObjectSpawner::spawnCapsuleStatic(glm::vec3 position, glm::vec3 scale, float mass, glm::quat orientation)
{
	glm::vec3 color = glm::vec3(1, 1, 0);

	static unsigned int staticCapsuleID = 0;

	float randFactor;
	randFactor = glm::linearRand(0.0f, 1.0f);
	color.x = randFactor;
	randFactor = glm::linearRand(0.0f, 1.0f);
	color.y = randFactor;
	randFactor = glm::linearRand(0.0f, 1.0f);
	color.z = randFactor;

	RigidBody *capsule = new RigidBody(position, scale);
	capsule->setMass(mass);
	capsule->setInertiaTensor(RigidBody::inertiaTensorCapsule(PhysicsSettings::get().shapes.capsule.height * capsule->scale.y, PhysicsSettings::get().shapes.capsule.radius * capsule->scale.x));
	capsule->orientation = orientation;
	capsule->updateTransformMatrix();
	capsule->isAwake = false;

	PhysicsObject *obj = new PhysicsObject();
	obj->body = capsule;
	obj->mesh = (*meshes)["capsule"];
	obj->color = color;
	obj->collider = new OBBCollider(PhysicsSettings::get().shapes.capsule.obbHalfSizes * obj->body->scale, obj, (*meshes)["box"]);
	obj->collider->setRigidBody(obj->body);
	obj->collider->updateInternals();
	obj->name.assign("staticCapsule" + std::to_string(staticCapsuleID++));
	obj->shape = new Capsule(PhysicsSettings::get().shapes.capsule.height * capsule->scale.y, PhysicsSettings::get().shapes.capsule.radius * capsule->scale.x);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);
}

std::string ObjectSpawner::toString()
{
	std::string nextObject;

	switch (this->nextObject) {
	case box:
		nextObject.assign("box");
		break;
	case longBox:
		nextObject.assign("longbox");
		break;
	case sphere:
		nextObject.assign("sphere");
		break;
	case cylinder:
		nextObject.assign("cylinder");
		break;
	case capsule:
		nextObject.assign("capsule");
		break;
	}
	return std::string("") + "ObjectSpawner {" + "\n\t"
		+ "numObjects = " + std::to_string(objects.size()) + "\n\t"
		+ "selectedObject = " + selectedObject->name + "\n\t"
		+ "nextSpawnedObject = " + nextObject + "\n\t"
		+ "randomizeNextObject = " + std::to_string(randomizeNextObject) + "\n\t"
		+ "randomizeProperties = " + std::to_string(randomizeProperties) + "\n\t"
		+ "applyStartingImpulse = " + std::to_string(applyStartingImpulse) + "\n\t"
		+ "spawnPosition = " + glm::to_string(spawnPosition) + "\n\t"
		+ "spawnDirection = " + glm::to_string(spawnDirection) + "\n}\n";
}
