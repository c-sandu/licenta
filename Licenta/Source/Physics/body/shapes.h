#pragma once

#include <string>
#include <glm/glm.hpp>

class Shape {
public:
	std::string type;

	virtual glm::vec3 getSupportPtInLocalSpace(glm::vec3 dir) = 0;
};

class Plane : public Shape {
public:
	float rx, rz;

	Plane(float rx, float rz) : rx(rx), rz(rz) { this->type.assign("plane"); }

	glm::vec3 getSupportPtInLocalSpace(glm::vec3 dir);
};

class Box : public Shape {
public:
	float rx, ry, rz;

	Box(float rx = 0.5f, float ry = 0.5f, float rz = 0.5f) : rx(rx), ry(ry), rz(rz) { this->type.assign("box"); }

	glm::vec3 getSupportPtInLocalSpace(glm::vec3 dir);
};

class Sphere : public Shape {
public:
	float radius;

	Sphere(float radius = 0.5f) : radius(radius) { this->type.assign("sphere"); }

	glm::vec3 getSupportPtInLocalSpace(glm::vec3 dir);
};

class Cylinder : public Shape {
public:
	float height;
	float radius;

	Cylinder(float height = 2.0f, float radius = 1.0f) : height(height), radius(radius) { this->type.assign("cylinder"); }

	glm::vec3 getSupportPtInLocalSpace(glm::vec3 dir);
};

class Cone : public Shape {
public:
	float height;
	float radius;

	Cone(float height, float radius) : height(height), radius(radius) { this->type.assign("cone"); }

	glm::vec3 getSupportPtInLocalSpace(glm::vec3 dir);
};