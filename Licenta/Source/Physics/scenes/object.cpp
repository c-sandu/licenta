#include "object.h"

#include <cstdlib>
#include <glm/gtc/random.hpp>
#include <glm/gtx/string_cast.hpp>

#include <Core/Engine.h>
#include <picojson.h>


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

void ObjectSpawner::loadFromFile(const std::string & filename)
{
	for (auto objIt = objects.begin(); objIt != objects.end();) {
		delete *objIt;
		objIt = objects.erase(objIt);
	}

	selectedObject = nullptr;

	std::ifstream fin(filename);

	picojson::value v;

	fin >> v;

	if (std::cin.fail()) {
		std::cerr << picojson::get_last_error() << std::endl;
	}

	if (v.is<picojson::null>()) {
		std::cout << "input is null" << std::endl;
	}

	std::string err = picojson::get_last_error();
	if (!err.empty()) {
		std::cerr << err << std::endl;
	}
	else {
		picojson::value settings = v.get("settings");
		if (!settings.is<picojson::null>()) {
			picojson::value damping = settings.get("damping");
			if (!damping.is<picojson::null>()) {
				picojson::value linear = damping.get("linear");
				if (!linear.is<picojson::null>())
					PhysicsSettings::get().damping.linear = linear.get<double>();
				picojson::value angular = damping.get("angular");
				if (!angular.is<picojson::null>())
					PhysicsSettings::get().damping.angular = angular.get<double>();
			}

			picojson::value forceMultipliers = settings.get("forceMultipliers");
			if (!forceMultipliers.is<picojson::null>()) {
				picojson::value torque = forceMultipliers.get("torque");
				if (!torque.is<picojson::null>())
					PhysicsSettings::get().forceMultipliers.torque = torque.get<double>();
				picojson::value force = forceMultipliers.get("force");
				if (!force.is<picojson::null>())
					PhysicsSettings::get().forceMultipliers.force = force.get<double>();
				picojson::value impulse = forceMultipliers.get("impulse");
				if (!impulse.is<picojson::null>())
					PhysicsSettings::get().forceMultipliers.impulse = impulse.get<double>();
			}

			picojson::value gjkepa = settings.get("gjkepa");
			if (!gjkepa.is<picojson::null>()) {
				picojson::value gjkMaxIters = gjkepa.get("gjkMaxIters");
				if (!gjkMaxIters.is<picojson::null>())
					PhysicsSettings::get().gjkepa.gjkMaxIters = gjkMaxIters.get<double>();
				picojson::value epaMaxIters = gjkepa.get("epaMaxIters");
				if (!epaMaxIters.is<picojson::null>())
					PhysicsSettings::get().gjkepa.epaMaxIters = epaMaxIters.get<double>();
			}

			picojson::value collisionResolution = settings.get("collisionResolution");
			if (!collisionResolution.is<picojson::null>()) {
				picojson::value penMaxIterations = collisionResolution.get("penMaxIterations");
				if (!penMaxIterations.is<picojson::null>())
					PhysicsSettings::get().collisionResolution.penMaxIterations = penMaxIterations.get<double>();
				picojson::value velMaxIterations = collisionResolution.get("velMaxIterations");
				if (!velMaxIterations.is<picojson::null>())
					PhysicsSettings::get().collisionResolution.velMaxIterations = velMaxIterations.get<double>();
				picojson::value minVelocityForRestitution = collisionResolution.get("minVelocityForRestitution");
				if (!minVelocityForRestitution.is<picojson::null>())
					PhysicsSettings::get().collisionResolution.minVelocityForRestitution = minVelocityForRestitution.get<double>();
				picojson::value angularMovementLimitFactor = collisionResolution.get("angularMovementLimitFactor");
				if (!angularMovementLimitFactor.is<picojson::null>())
					PhysicsSettings::get().collisionResolution.angularMovementLimitFactor = angularMovementLimitFactor.get<double>();
				picojson::value persistentContactDistanceThreshold = collisionResolution.get("persistentContactDistanceThreshold");
				if (!persistentContactDistanceThreshold.is<picojson::null>())
					PhysicsSettings::get().collisionResolution.persistentContactDistanceThreshold = persistentContactDistanceThreshold.get<double>();
				picojson::value coefInterpAlpha = collisionResolution.get("coefInterpAlpha");
				if (!coefInterpAlpha.is<picojson::null>())
					PhysicsSettings::get().collisionResolution.coefInterpAlpha = coefInterpAlpha.get<double>();
			}

			picojson::value rigidBodies = settings.get("rigidBodies");
			if (!rigidBodies.is<picojson::null>()) {
				picojson::value defaultMass = rigidBodies.get("defaultMass");
				if (!defaultMass.is<picojson::null>())
					PhysicsSettings::get().rigidBodies.defaultMass = defaultMass.get<double>();
				picojson::value defaultRestitutionCoef = rigidBodies.get("defaultRestitutionCoef");
				if (!defaultRestitutionCoef.is<picojson::null>())
					PhysicsSettings::get().rigidBodies.defaultRestitutionCoef = defaultRestitutionCoef.get<double>();
				picojson::value defaultFrictionCoef = rigidBodies.get("defaultFrictionCoef");
				if (!defaultFrictionCoef.is<picojson::null>())
					PhysicsSettings::get().rigidBodies.defaultFrictionCoef = defaultFrictionCoef.get<double>();
				picojson::value sleepMotionThreshold = rigidBodies.get("sleepMotionThreshold");
				if (!sleepMotionThreshold.is<picojson::null>())
					PhysicsSettings::get().rigidBodies.sleepMotionThreshold = sleepMotionThreshold.get<double>();
			}

			picojson::value gravity = settings.get("gravity");
			if (!gravity.is<picojson::null>()) {
				picojson::value x = gravity.get("x");
				if (!x.is<picojson::null>())
					PhysicsSettings::get().gravity.x = x.get<double>();
				picojson::value y = gravity.get("y");
				if (!y.is<picojson::null>())
					PhysicsSettings::get().gravity.y = y.get<double>();
				picojson::value z = gravity.get("z");
				if (!z.is<picojson::null>())
					PhysicsSettings::get().gravity.z = z.get<double>();
			}

			picojson::value timeScale = settings.get("timeScale");
			if (!timeScale.is<picojson::null>()) {
				PhysicsSettings::get().timeScale = timeScale.get<double>();
			}

			picojson::value rendering = settings.get("rendering");
			if (!rendering.is<picojson::null>()) {
				picojson::value renderColliders = rendering.get("renderColliders");
				if (!renderColliders.is<picojson::null>())
					PhysicsSettings::get().rendering.renderColliders = renderColliders.get<bool>();
				picojson::value renderContacts = rendering.get("renderContacts");
				if (!renderContacts.is<picojson::null>())
					PhysicsSettings::get().rendering.renderContacts = renderContacts.get<bool>();
				picojson::value renderContactNormals = rendering.get("renderContactNormals");
				if (!renderContactNormals.is<picojson::null>())
					PhysicsSettings::get().rendering.renderContactNormals = renderContactNormals.get<bool>();
				picojson::value renderSpawner = rendering.get("renderSpawner");
				if (!renderSpawner.is<picojson::null>())
					PhysicsSettings::get().rendering.renderSpawner = renderSpawner.get<bool>();
				picojson::value renderSelection = rendering.get("renderSelection");
				if (!renderSelection.is<picojson::null>())
					PhysicsSettings::get().rendering.renderSelection = renderSelection.get<bool>();
			}
		}
		picojson::value objects = v.get("objects");
		if (!objects.is<picojson::null>()) {
			for (const auto o : objects.get<picojson::array>()) {
				picojson::value type = o.get("type");
				std::string objType = "";
				if (!type.is<picojson::null>())
					objType = type.get<std::string>();
				picojson::value name = o.get("name");
				std::string objName = "";
				if (!name.is<picojson::null>())
					objName = name.get<std::string>();
				picojson::value position = o.get("position");
				glm::vec3 objPosition = glm::vec3(0);
				if (!position.is<picojson::null>()) {
					picojson::value x = position.get("x");
					if (!x.is<picojson::null>())
						objPosition.x = x.get<double>();
					picojson::value y = position.get("y");
					if (!y.is<picojson::null>())
						objPosition.y = y.get<double>();
					picojson::value z = position.get("z");
					if (!z.is<picojson::null>())
						objPosition.z = z.get<double>();
				}
				picojson::value scale = o.get("scale");
				glm::vec3 objScale = glm::vec3(1);
				if (!scale.is<picojson::null>()) {
					picojson::value x = scale.get("x");
					if (!x.is<picojson::null>())
						objScale.x = x.get<double>();
					picojson::value y = scale.get("y");
					if (!y.is<picojson::null>())
						objScale.y = y.get<double>();
					picojson::value z = scale.get("z");
					if (!z.is<picojson::null>())
						objScale.z = z.get<double>();
				}
				picojson::value orientation = o.get("orientation");
				glm::vec3 objOrientationEuler = glm::vec3(1);
				if (!scale.is<picojson::null>()) {
					picojson::value x = orientation.get("pitch");
					if (!x.is<picojson::null>())
						objOrientationEuler.x = x.get<double>();
					picojson::value y = orientation.get("yaw");
					if (!y.is<picojson::null>())
						objOrientationEuler.y = y.get<double>();
					picojson::value z = orientation.get("roll");
					if (!z.is<picojson::null>())
						objOrientationEuler.z = z.get<double>();
				}
				picojson::value frictionCoef = o.get("frictionCoef");
				float objFrictionCoef = PhysicsSettings::get().rigidBodies.defaultFrictionCoef;
				if (!frictionCoef.is<picojson::null>())
					objFrictionCoef = frictionCoef.get<double>();
				picojson::value restitutionCoef = o.get("restitutionCoef");
				float objRestitutionCoef = PhysicsSettings::get().rigidBodies.defaultRestitutionCoef;
				if (!restitutionCoef.is<picojson::null>())
					objRestitutionCoef = restitutionCoef.get<double>();
				picojson::value mass = o.get("mass");
				float objMass = PhysicsSettings::get().rigidBodies.defaultMass;
				if (!mass.is<picojson::null>())
					objMass = mass.get<double>();

				PhysicsObject *newObject = nullptr;
				if (!objType.compare("wall")) {
					newObject = spawnWallStatic(objPosition, objScale, glm::quat(RADIANS(objOrientationEuler)));
				}
				else if (!objType.compare("box")) {
					newObject = spawnBoxStatic(objPosition, objScale, objMass, glm::quat(RADIANS(objOrientationEuler)));
				}
				else if (!objType.compare("longBox")) {
					newObject = spawnLongBoxStatic(objPosition, objScale, objMass, glm::quat(RADIANS(objOrientationEuler)));
				}
				else if (!objType.compare("sphere")) {
					newObject = spawnSphereStatic(objPosition, objScale, objMass, glm::quat(RADIANS(objOrientationEuler)));
				}
				else if (!objType.compare("cylinder")) {
					newObject = spawnCylinderStatic(objPosition, objScale, objMass, glm::quat(RADIANS(objOrientationEuler)));
				}
				else if (!objType.compare("capsule")) {
					newObject = spawnCapsuleStatic(objPosition, objScale, objMass, glm::quat(RADIANS(objOrientationEuler)));
				}
				else {
					continue;
				}

				newObject->body->frictionCoef = objFrictionCoef;
				newObject->body->restitutionCoef = objRestitutionCoef;
				newObject->body->setMass(objMass);
				newObject->type.assign(objType);
				if (objName.compare(""))
					newObject->name.assign(objName);
			}
		}
	}

	fin.close();
}

void ObjectSpawner::saveToFile(const std::string & filename)
{
	std::ofstream fout(filename);

	picojson::object v;

	/* write settings */
	{
		picojson::object settings;

		picojson::object damping;
		damping["linear"] = picojson::value((double) PhysicsSettings::get().damping.linear);
		damping["angular"] = picojson::value((double) PhysicsSettings::get().damping.angular);
		settings["damping"] = picojson::value(damping);

		picojson::object forceMultipliers;
		forceMultipliers["torque"] = picojson::value((double) PhysicsSettings::get().forceMultipliers.torque);
		forceMultipliers["force"] = picojson::value((double) PhysicsSettings::get().forceMultipliers.force);
		forceMultipliers["impulse"] = picojson::value((double) PhysicsSettings::get().forceMultipliers.impulse);
		settings["forceMultipliers"] = picojson::value(forceMultipliers);

		picojson::object gjkepa;
		damping["gjkMaxIters"] = picojson::value((double) PhysicsSettings::get().gjkepa.gjkMaxIters);
		damping["epaMaxIters"] = picojson::value((double) PhysicsSettings::get().gjkepa.epaMaxIters);
		settings["gjkepa"] = picojson::value(gjkepa);

		picojson::object collisionResolution;
		collisionResolution["penMaxIterations"] = picojson::value((double)PhysicsSettings::get().collisionResolution.penMaxIterations);
		collisionResolution["velMaxIterations"] = picojson::value((double)PhysicsSettings::get().collisionResolution.velMaxIterations);
		collisionResolution["minVelocityForRestitution"] = picojson::value((double)PhysicsSettings::get().collisionResolution.minVelocityForRestitution);
		collisionResolution["angularMovementLimitFactor"] = picojson::value((double)PhysicsSettings::get().collisionResolution.angularMovementLimitFactor);
		collisionResolution["persistentContactDistanceThreshold"] = picojson::value((double)PhysicsSettings::get().collisionResolution.persistentContactDistanceThreshold);
		collisionResolution["coefInterpAlpha"] = picojson::value((double)PhysicsSettings::get().collisionResolution.coefInterpAlpha);
		settings["collisionResolution"] = picojson::value(collisionResolution);

		picojson::object rigidBodies;
		rigidBodies["defaultMass"] = picojson::value((double)PhysicsSettings::get().rigidBodies.defaultMass);
		rigidBodies["defaultRestitutionCoef"] = picojson::value((double)PhysicsSettings::get().rigidBodies.defaultRestitutionCoef);
		rigidBodies["defaultFrictionCoef"] = picojson::value((double)PhysicsSettings::get().rigidBodies.defaultFrictionCoef);
		rigidBodies["sleepMotionThreshold"] = picojson::value((double)PhysicsSettings::get().rigidBodies.sleepMotionThreshold);
		settings["rigidBodies"] = picojson::value(rigidBodies);

		picojson::object gravity;
		gravity["x"] = picojson::value((double)PhysicsSettings::get().gravity.x);
		gravity["y"] = picojson::value((double)PhysicsSettings::get().gravity.y);
		gravity["z"] = picojson::value((double)PhysicsSettings::get().gravity.z);
		settings["gravity"] = picojson::value(gravity);

		settings["timeScale"] = picojson::value((double)PhysicsSettings::get().timeScale);

		picojson::object rendering;
		rendering["renderColliders"] = picojson::value(PhysicsSettings::get().rendering.renderColliders);
		rendering["renderContacts"] = picojson::value(PhysicsSettings::get().rendering.renderContacts);
		rendering["renderContactNormals"] = picojson::value(PhysicsSettings::get().rendering.renderContactNormals);
		rendering["renderSpawner"] = picojson::value(PhysicsSettings::get().rendering.renderSpawner);
		rendering["renderSelection"] = picojson::value(PhysicsSettings::get().rendering.renderSelection);
		settings["rendering"] = picojson::value(rendering);

		v["settings"] = picojson::value(settings);
	}

	/* write objects */
	{
		picojson::array objects;

		for (auto o : this->objects) {
			picojson::object object;

			object["type"] = picojson::value(o->type);
			object["name"] = picojson::value(o->name);

			picojson::object position;
			position["x"] = picojson::value((double) o->body->position.x);
			position["y"] = picojson::value((double) o->body->position.y);
			position["z"] = picojson::value((double) o->body->position.z);
			object["position"] = picojson::value(position);

			picojson::object scale;
			glm::vec3 objScale = o->body->scale;
			if (!o->type.compare("longBox"))
				objScale /= glm::vec3(1, 2, 1);
			if (!o->type.compare("wall"))
				objScale /= glm::vec3(8, 8, 1);
			scale["x"] = picojson::value((double) objScale.x);
			scale["y"] = picojson::value((double) objScale.y);
			scale["z"] = picojson::value((double) objScale.z);
			object["scale"] = picojson::value(scale);

			picojson::object orientation;
			glm::vec3 euler = DEGREES(glm::eulerAngles(glm::normalize(o->body->orientation)));
			orientation["pitch"] = picojson::value((double) euler.x);
			orientation["yaw"] = picojson::value((double) euler.y);
			orientation["roll"] = picojson::value((double) euler.z);
			object["orientation"] = picojson::value(orientation);

			object["frictionCoef"] = picojson::value((double) o->body->frictionCoef);
			object["restitutionCoef"] = picojson::value((double)o->body->restitutionCoef);
			object["mass"] = picojson::value((double)o->body->mass);

			objects.push_back(picojson::value(object));
		}

		v["objects"] = picojson::value(objects);
	}

	fout << picojson::value(v);
	fout.close();
}

PhysicsObject* ObjectSpawner::spawnBoxDynamic()
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
	obj->type.assign("box");
	obj->shape = new Box(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);

	selectedObject = obj;

	return obj;
}

PhysicsObject* ObjectSpawner::spawnBoxStatic(glm::vec3 position, glm::vec3 scale, float mass, glm::quat orientation) {
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
	obj->type.assign("box");
	obj->shape = new Box(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);

	selectedObject = obj;

	return obj;
}

PhysicsObject* ObjectSpawner::spawnLongBoxDynamic()
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
	obj->type.assign("longBox");
	obj->shape = new Box(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);

	selectedObject = obj;

	return obj;
}

PhysicsObject* ObjectSpawner::spawnLongBoxStatic(glm::vec3 position, glm::vec3 scale, float mass, glm::quat orientation)
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
	obj->type.assign("longBox");
	obj->shape = new Box(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);

	selectedObject = obj;

	return obj;
}

PhysicsObject* ObjectSpawner::spawnWallStatic(glm::vec3 position, glm::vec3 scale, glm::quat orientation)
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
	obj->type.assign("wall");
	obj->shape = new Box(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);

	selectedObject = obj;

	return obj;
}

PhysicsObject* ObjectSpawner::spawnSphereDynamic()
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
	obj->type.assign("sphere");
	obj->shape = new Sphere(PhysicsSettings::get().shapes.sphere.radius * sphere->scale.x);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);

	selectedObject = obj;

	return obj;
}

PhysicsObject* ObjectSpawner::spawnSphereStatic(glm::vec3 position, glm::vec3 scale, float mass, glm::quat orientation)
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
	obj->type.assign("sphere");
	obj->shape = new Sphere(PhysicsSettings::get().shapes.sphere.radius * sphere->scale.x);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);

	selectedObject = obj;

	return obj;
}

PhysicsObject* ObjectSpawner::spawnCylinderDynamic()
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
	obj->type.assign("cylinder");
	obj->shape = new Cylinder(PhysicsSettings::get().shapes.cylinder.height * cylinder->scale.y, PhysicsSettings::get().shapes.cylinder.radius * cylinder->scale.x);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);

	selectedObject = obj;

	return obj;
}

PhysicsObject* ObjectSpawner::spawnCylinderStatic(glm::vec3 position, glm::vec3 scale, float mass, glm::quat orientation)
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
	obj->type.assign("cylinder");
	obj->shape = new Cylinder(PhysicsSettings::get().shapes.cylinder.height * cylinder->scale.y, PhysicsSettings::get().shapes.cylinder.radius * cylinder->scale.x);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);

	selectedObject = obj;

	return obj;
}

PhysicsObject* ObjectSpawner::spawnCapsuleDynamic()
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
	obj->type.assign("capsule");
	obj->shape = new Capsule(PhysicsSettings::get().shapes.capsule.height * capsule->scale.y, PhysicsSettings::get().shapes.capsule.radius * capsule->scale.x);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);

	selectedObject = obj;

	return obj;
}

PhysicsObject* ObjectSpawner::spawnCapsuleStatic(glm::vec3 position, glm::vec3 scale, float mass, glm::quat orientation)
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
	obj->type.assign("capsule");
	obj->shape = new Capsule(PhysicsSettings::get().shapes.capsule.height * capsule->scale.y, PhysicsSettings::get().shapes.capsule.radius * capsule->scale.x);
	objects.push_back(obj);
	pcd->addCollider((OBBCollider*)obj->collider);

	selectedObject = obj;

	return obj;
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
