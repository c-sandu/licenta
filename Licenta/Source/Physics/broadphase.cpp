#include "broadphase.h"

#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/transform.hpp>
#include <iostream>


void Collider::setRigidBody(RigidBody * body)
{
	this->body = body;
}


OBBCollider::OBBCollider(const glm::vec3 & halfSizes, PhysicsObject * phyObject)
	: halfSizes(halfSizes)
{
	meshName.assign("box");
	this->phyObject = phyObject;
}

void OBBCollider::updateInternals()
{
	position = body->position;
	orientation = body->orientation;
}

bool OBBCollider::testIntersectionOBB(OBBCollider & other)
{
	unsigned int i;

	/* compute local axis */
	glm::vec3 thisAxis[3] = { glm::normalize(glm::vec3(1, 0, 0) * orientation),
							  glm::normalize(glm::vec3(0, 1, 0) * orientation),
							  glm::normalize(glm::vec3(0, 0, 1) * orientation) };
	glm::vec3 otherAxis[3] = { glm::normalize(glm::vec3(1, 0, 0) * other.orientation),
							   glm::normalize(glm::vec3(0, 1, 0) * other.orientation),
							   glm::normalize(glm::vec3(0, 0, 1) * other.orientation) };

	glm::vec3 diff = other.position - position;
	float c[3][3]; /* c[i][j] = dot(thisAxis[i], otherAxis[j]) = cosinus of angle between them */
	float absC[3][3]; /* abs(c[i][j]) */
	float epsilon = 0.000001f;
	float d[3]; /* d[i] = dot(diff, thisAxis[i]) */
	float r; /* distance between centers */
	float r0, r1; /* projection radius for this OBB and other OBB, respectively */

	bool existParallelPair = false;

	/* axis C0 + t * A0 */
	{
		for (i = 0; i < 3; i++) {
			c[0][i] = glm::dot(thisAxis[0], otherAxis[i]);
			absC[0][i] = glm::abs(c[0][i]);
			if (1.0f - absC[0][i] < epsilon)
				existParallelPair = true;
		}
		d[0] = glm::dot(diff, thisAxis[0]);
		r = glm::abs(d[0]);
		r0 = this->halfSizes.x;
		r1 = other.halfSizes.x * absC[0][0] + other.halfSizes.y * absC[0][1]
			+ other.halfSizes.z * absC[0][2];
		if (r > r0 + r1)
			return false;
	}
	/* axis C0 + t * A1 */
	{
		for (i = 0; i < 3; i++) {
			c[1][i] = glm::dot(thisAxis[1], otherAxis[i]);
			absC[1][i] = glm::abs(c[1][i]);
			if (1.0f - absC[1][i] < epsilon)
				existParallelPair = true;
		}
		d[1] = glm::dot(diff, thisAxis[1]);
		r = glm::abs(d[1]);
		r0 = this->halfSizes.y;
		r1 = other.halfSizes.x * absC[1][0] + other.halfSizes.y * absC[1][1]
			+ other.halfSizes.z * absC[1][2];
		if (r > r0 + r1)
			return false;
	}
	/* axis C0 + t * A2 */
	{
		for (i = 0; i < 3; i++) {
			c[2][i] = glm::dot(thisAxis[2], otherAxis[i]);
			absC[2][i] = glm::abs(c[2][i]);
			if (1.0f - absC[2][i] < epsilon)
				existParallelPair = true;
		}
		d[2] = glm::dot(diff, thisAxis[2]);
		r = glm::abs(d[2]);
		r0 = this->halfSizes.z;
		r1 = other.halfSizes.x * absC[2][0] + other.halfSizes.y * absC[2][1]
			+ other.halfSizes.z * absC[2][2];
		if (r > r0 + r1)
			return false;
	}
	/* axis C0 + t * B0 */
	{
		r = glm::dot(diff, otherAxis[0]);
		r0 = this->halfSizes.x * absC[0][0] + this->halfSizes.y * absC[1][0]
			+ this->halfSizes.z * absC[2][0];
		r1 = other.halfSizes.x;
		if (r > r0 + r1)
			return false;
	}
	/* axis C0 + t * B1 */
	{
		r = glm::dot(diff, otherAxis[1]);
		r0 = this->halfSizes.x * absC[0][1] + this->halfSizes.y * absC[1][1]
			+ this->halfSizes.z * absC[2][1];
		r1 = other.halfSizes.y;
		if (r > r0 + r1)
			return false;
	}
	/* axis C0 + t * B2 */
	{
		r = glm::dot(diff, otherAxis[2]);
		r0 = this->halfSizes.x * absC[0][2] + this->halfSizes.y * absC[1][2]
			+ this->halfSizes.z * absC[2][2];
		r1 = other.halfSizes.z;
		if (r > r0 + r1)
			return false;
	}

	if (existParallelPair)
		return true;

	/* axis C0 + t * A0 x B0 */
	{
		r = glm::abs(d[2] * c[1][0] - d[1] * c[2][0]);
		r0 = this->halfSizes.y * absC[2][0] + this->halfSizes.z * absC[1][0];
		r1 = other.halfSizes.y * absC[0][2] + other.halfSizes.z * absC[0][1];
		if (r > r0 + r1)
			return false;
	}
	/* axis C0 + t * A0 x B1 */
	{
		r = glm::abs(d[2] * c[1][1] - d[1] * c[2][1]);
		r0 = this->halfSizes.y * absC[2][1] + this->halfSizes.z * absC[1][1];
		r1 = other.halfSizes.x * absC[0][2] + other.halfSizes.z * absC[0][0];
		if (r > r0 + r1)
			return false;
	}
	/* axis C0 + t * A0 x B2 */
	{
		r = glm::abs(d[2] * c[1][2] - d[1] * c[2][2]);
		r0 = this->halfSizes.y * absC[2][2] + this->halfSizes.z * absC[1][2];
		r1 = other.halfSizes.x * absC[0][1] + other.halfSizes.y * absC[0][0];
		if (r > r0 + r1)
			return false;
	}
	/* axis C0 + t * A1 x B0 */
	{
		r = glm::abs(d[0] * c[2][0] - d[2] * c[0][0]);
		r0 = this->halfSizes.x * absC[2][0] + this->halfSizes.z * absC[0][0];
		r1 = other.halfSizes.y * absC[1][2] + other.halfSizes.z * absC[1][1];
		if (r > r0 + r1)
			return false;
	}
	/* axis C0 + t * A1 x B1 */
	{
		r = glm::abs(d[0] * c[2][1] - d[2] * c[0][1]);
		r0 = this->halfSizes.x * absC[2][1] + this->halfSizes.z * absC[0][1];
		r1 = other.halfSizes.x * absC[1][2] + other.halfSizes.z * absC[1][0];
		if (r > r0 + r1)
			return false;
	}
	/* axis C0 + t * A1 x B2 */
	{
		r = glm::abs(d[0] * c[2][2] - d[2] * c[0][2]);
		r0 = this->halfSizes.x * absC[2][2] + this->halfSizes.z * absC[0][2];
		r1 = other.halfSizes.x * absC[1][1] + other.halfSizes.y * absC[1][0];
		if (r > r0 + r1)
			return false;
	}
	/* axis C0 + t * A2 x B0 */
	{
		r = glm::abs(d[1] * c[0][0] - d[0] * c[1][0]);
		r0 = this->halfSizes.x * absC[1][0] + this->halfSizes.y * absC[0][0];
		r1 = other.halfSizes.y * absC[2][2] + other.halfSizes.z * absC[2][1];
		if (r > r0 + r1)
			return false;
	}
	/* axis C0 + t * A2 x B1 */
	{
		r = glm::abs(d[1] * c[0][1] - d[0] * c[1][1]);
		r0 = this->halfSizes.x * absC[1][1] + this->halfSizes.y * absC[0][1];
		r1 = other.halfSizes.x * absC[2][2] + other.halfSizes.z * absC[2][0];
		if (r > r0 + r1)
			return false;
	}
	/* axis C0 + t * A2 x B2 */
	{
		r = glm::abs(d[1] * c[0][2] - d[0] * c[1][2]);
		r0 = this->halfSizes.x * absC[1][2] + this->halfSizes.y * absC[0][2];
		r1 = other.halfSizes.x * absC[2][1] + other.halfSizes.t * absC[2][0];
		if (r > r0 + r1)
			return false;
	}
	return true;
}

bool OBBCollider::testIntersectionPlane(PlaneCollider & plane)
{
	/* compute local axis */
	glm::vec3 thisAxis[3] = { glm::normalize(glm::vec3(1, 0, 0) * orientation),
							  glm::normalize(glm::vec3(0, 1, 0) * orientation),
							  glm::normalize(glm::vec3(0, 0, 1) * orientation) };

	float projectedRadius = halfSizes.x * glm::abs(glm::dot(plane.normal, thisAxis[0])) +
		halfSizes.y * glm::abs(glm::dot(plane.normal, thisAxis[1])) +
		halfSizes.z * glm::abs(glm::dot(plane.normal, thisAxis[2]));

	float distanceFromOrigin = glm::dot(plane.normal, position) - projectedRadius;

	return distanceFromOrigin <= plane.offset;
}

void PotentialCollisionDetector::addCollider(PlaneCollider * plane)
{
	planeVector.push_back(plane);
}

void PotentialCollisionDetector::addCollider(OBBCollider * obb)
{
	obbVector.push_back(obb);
}

void PotentialCollisionDetector::clearPotentialCollisions()
{
	for (PotentialCollision *p : potentialCollisions)
		delete p;

	potentialCollisions.clear();
}

void PotentialCollisionDetector::fillPotentialCollisions()
{
	clearPotentialCollisions();

	for (auto & i : obbVector) {
		/* look for OBB-OBB collisions */
		for (auto & j : obbVector)
			if (i != j && i->testIntersectionOBB(*j)) {
				potentialCollisions.push_back(new PotentialCollision(i->phyObject, j->phyObject));
			}
		/* look for OBB-Plane collisions */
		for (auto & p : planeVector)
			if (i->testIntersectionPlane(*p)) {
				potentialCollisions.push_back(new PotentialCollision(i->phyObject, p->phyObject));
			}
	}
}

PlaneCollider::PlaneCollider(glm::vec3 normal, float offset, PhysicsObject * phyObject)
	: normal(normal), offset(offset)
{
	meshName.assign("plane");
	this->phyObject = phyObject;
}

void PlaneCollider::updateInternals()
{
}

glm::mat4 PlaneCollider::getTransformMatrix()
{
	return glm::translate(glm::mat4(1), normal * offset);
}

std::string PlaneCollider::toString()
{
	return std::string("") + "\tPlaneCollider {" + "\n\t\t"
		+ "normal = " + to_string(normal) + "\n\t\t"
		+ "offsetFromOrigin = " + std::to_string(offset) + "\n\t}\n";
}

glm::mat4 OBBCollider::getTransformMatrix()
{
	glm::mat4 transform = body->transform;
	transform = glm::scale(transform, 1.0f / body->scale);
	transform = glm::scale(transform, halfSizes / 0.5f);
	return transform;
}

std::string OBBCollider::toString()
{
	return std::string("") + "\tOBBCollider {" + "\n\t\t"
		+ "halfSizes = " + to_string(halfSizes) + "\n\t\t"
		+ "position = " + to_string(position) + "\n\t\t"
		+ "orientation = " + to_string(orientation) + "\n\t}\n";
}

PotentialCollision::PotentialCollision(PhysicsObject * one, PhysicsObject * two)
	: one(one), two(two)
{
}

