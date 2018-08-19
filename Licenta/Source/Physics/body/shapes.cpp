#include "shapes.h"

glm::vec3 Plane::getSupportPtInLocalSpace(glm::vec3 dir)
{
	return glm::vec3(rx * (dir.x >= 0.0f ? 1.0f : -1.0f), 0, rz * (dir.z >= 0.0f ? 1.0f : -1.0f));
}

glm::vec3 Box::getSupportPtInLocalSpace(glm::vec3 dir)
{
	//return glm::vec3(rx * (dir.x >= 0.0f ? 1.0f : -1.0f), ry * (dir.y >= 0.0f ? 1.0f : -1.0f), rz * (dir.z >= 0.0f ? 1.0f : -1.0f));
	return glm::vec3(rx * glm::sign(dir.x), ry * glm::sign(dir.y), rz * glm::sign(dir.z));
}

glm::vec3 Sphere::getSupportPtInLocalSpace(glm::vec3 dir)
{
	return radius * dir;
}

glm::vec3 Cylinder::getSupportPtInLocalSpace(glm::vec3 dir)
{
	glm::vec3 supportDisc = glm::length(glm::vec3(dir.x, 0, dir.z)) > 0 ? radius * glm::vec3(dir.x, 0, dir.z) / glm::length(glm::vec3(dir.x, 0, dir.z)) : glm::vec3(0);
	glm::vec3 supportHeight = glm::vec3(0, 0.5f * height * glm::sign(dir.y), 0);
	return supportHeight + supportDisc;
}

glm::vec3 Cone::getSupportPtInLocalSpace(glm::vec3 dir)
{
	// TODO
	glm::vec3 supportDisc = radius * glm::vec3(dir.x, dir.y, 0) / glm::length(glm::vec3(dir.x, dir.y, 0));
	glm::vec3 supportHeight = glm::vec3(0, 0, height * glm::sign(dir.z));

	if (glm::dot(dir, supportDisc) > glm::dot(dir, supportHeight))
		return supportDisc;
	return supportHeight;
}
