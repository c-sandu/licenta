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
