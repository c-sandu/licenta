#pragma once

#include <string>
#include <glm/glm.hpp>
#include <Physics/settings.h>

class Shape {
public:
	std::string type;

	virtual glm::vec3 getSupportPtInLocalSpace(glm::vec3 dir) = 0;
};

/*!!!!!!!!!!!!Not supported anymore */
class Plane : public Shape {
public:
	float rx, rz;

	Plane(float rx, float rz) : rx(rx), rz(rz) { this->type.assign("plane"); }

	glm::vec3 getSupportPtInLocalSpace(glm::vec3 dir);
};

class Box : public Shape {
public:
	glm::vec3 halfSizes;

	Box(glm::vec3 halfSizes = PhysicsSettings::get().shapes.box.halfSizes) : halfSizes(halfSizes) { this->type.assign("box"); }

	glm::vec3 getSupportPtInLocalSpace(glm::vec3 dir);
};

class Sphere : public Shape {
public:
	float radius;

	Sphere(float radius = PhysicsSettings::get().shapes.sphere.radius) : radius(radius) { this->type.assign("sphere"); }

	glm::vec3 getSupportPtInLocalSpace(glm::vec3 dir);
};

class Cylinder : public Shape {
public:
	float height;
	float radius;

	Cylinder(float height = PhysicsSettings::get().shapes.cylinder.height, float radius = PhysicsSettings::get().shapes.cylinder.radius) : height(height), radius(radius) { this->type.assign("cylinder"); }

	glm::vec3 getSupportPtInLocalSpace(glm::vec3 dir);
};

/* not supported */
class Cone : public Shape {
public:
	float height;
	float radius;

	Cone(float height = 1.0f, float radius = 0.5f) : height(height), radius(radius) { this->type.assign("cone"); }

	glm::vec3 getSupportPtInLocalSpace(glm::vec3 dir);
};

class Capsule : public Shape {
public:
	float height;
	float radius;

	Capsule(float height = PhysicsSettings::get().shapes.capsule.height, float radius = PhysicsSettings::get().shapes.capsule.radius) : height(height), radius(radius) { this->type.assign("capsule"); }

	glm::vec3 getSupportPtInLocalSpace(glm::vec3 dir);
};