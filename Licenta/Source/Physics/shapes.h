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

	Plane(float rx, float rz) : rx(rx), rz(rz) { this->type.assign("plane"); };

	glm::vec3 getSupportPtInLocalSpace(glm::vec3 dir);
};

class Box : public Shape {
public:
	float rx, ry, rz;

	Box(float rx, float ry, float rz) : rx(rx), ry(ry), rz(rz) { this->type.assign("box"); };

	glm::vec3 getSupportPtInLocalSpace(glm::vec3 dir);
};