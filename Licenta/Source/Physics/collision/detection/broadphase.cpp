#include "broadphase.h"

#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <Physics/debug.h>


void Collider::setRigidBody(RigidBody * body)
{
	this->body = body;
}


OBBCollider::OBBCollider(const glm::vec3 & halfSizes, PhysicsObject * phyObject, Mesh * mesh)
	: halfSizes(halfSizes)
{
	this->mesh = mesh;
	this->phyObject = phyObject;
}

void OBBCollider::updateInternals()
{
	glm::vec3 _scale, _skew;
	glm::vec4 _perspective;
	glm::decompose(body->transform, _scale, orientation, position, _skew, _perspective);
}

//bool OBBCollider::testIntersectionOBB(OBBCollider & other)
//{
//	unsigned int i;
//
//	/* compute local axis */
//	glm::vec3 thisAxis[3] = { glm::normalize(glm::vec3(1, 0, 0) * orientation),
//							  glm::normalize(glm::vec3(0, 1, 0) * orientation),
//							  glm::normalize(glm::vec3(0, 0, 1) * orientation) };
//	glm::vec3 otherAxis[3] = { glm::normalize(glm::vec3(1, 0, 0) * other.orientation),
//							   glm::normalize(glm::vec3(0, 1, 0) * other.orientation),
//							   glm::normalize(glm::vec3(0, 0, 1) * other.orientation) };
//
//	glm::vec3 diff = other.position - position;
//	float c[3][3]; /* c[i][j] = dot(thisAxis[i], otherAxis[j]) = cosinus of angle between them */
//	float absC[3][3]; /* abs(c[i][j]) */
//	float epsilon = 0.0001f;
//	float d[3]; /* d[i] = dot(diff, thisAxis[i]) */
//	float r; /* distance between centers */
//	float r0, r1; /* projection radius for this OBB and other OBB, respectively */
//
//	bool existParallelPair = false;
//
//	/* axis C0 + t * A0 */
//	{
//		for (i = 0; i < 3; i++) {
//			c[0][i] = glm::dot(thisAxis[0], otherAxis[i]);
//			absC[0][i] = glm::abs(c[0][i]);
//			if (1.0f - absC[0][i] < epsilon)
//				existParallelPair = true;
//		}
//		d[0] = glm::dot(diff, thisAxis[0]);
//		r = glm::abs(d[0]);
//		r0 = this->halfSizes.x;
//		r1 = other.halfSizes.x * absC[0][0] + other.halfSizes.y * absC[0][1]
//			+ other.halfSizes.z * absC[0][2];
//		if (r > r0 + r1)
//			return false;
//	}
//	/* axis C0 + t * A1 */
//	{
//		for (i = 0; i < 3; i++) {
//			c[1][i] = glm::dot(thisAxis[1], otherAxis[i]);
//			absC[1][i] = glm::abs(c[1][i]);
//			if (1.0f - absC[1][i] < epsilon)
//				existParallelPair = true;
//		}
//		d[1] = glm::dot(diff, thisAxis[1]);
//		r = glm::abs(d[1]);
//		r0 = this->halfSizes.y;
//		r1 = other.halfSizes.x * absC[1][0] + other.halfSizes.y * absC[1][1]
//			+ other.halfSizes.z * absC[1][2];
//		if (r > r0 + r1)
//			return false;
//	}
//	/* axis C0 + t * A2 */
//	{
//		for (i = 0; i < 3; i++) {
//			c[2][i] = glm::dot(thisAxis[2], otherAxis[i]);
//			absC[2][i] = glm::abs(c[2][i]);
//			if (1.0f - absC[2][i] < epsilon)
//				existParallelPair = true;
//		}
//		d[2] = glm::dot(diff, thisAxis[2]);
//		r = glm::abs(d[2]);
//		r0 = this->halfSizes.z;
//		r1 = other.halfSizes.x * absC[2][0] + other.halfSizes.y * absC[2][1]
//			+ other.halfSizes.z * absC[2][2];
//		if (r > r0 + r1)
//			return false;
//	}
//	/* axis C0 + t * B0 */
//	{
//		r = glm::dot(diff, otherAxis[0]);
//		r0 = this->halfSizes.x * absC[0][0] + this->halfSizes.y * absC[1][0]
//			+ this->halfSizes.z * absC[2][0];
//		r1 = other.halfSizes.x;
//		if (r > r0 + r1)
//			return false;
//	}
//	/* axis C0 + t * B1 */
//	{
//		r = glm::dot(diff, otherAxis[1]);
//		r0 = this->halfSizes.x * absC[0][1] + this->halfSizes.y * absC[1][1]
//			+ this->halfSizes.z * absC[2][1];
//		r1 = other.halfSizes.y;
//		if (r > r0 + r1)
//			return false;
//	}
//	/* axis C0 + t * B2 */
//	{
//		r = glm::dot(diff, otherAxis[2]);
//		r0 = this->halfSizes.x * absC[0][2] + this->halfSizes.y * absC[1][2]
//			+ this->halfSizes.z * absC[2][2];
//		r1 = other.halfSizes.z;
//		if (r > r0 + r1)
//			return false;
//	}
//
//	if (existParallelPair)
//		return true;
//
//	/* axis C0 + t * A0 x B0 */
//	{
//		r = glm::abs(d[2] * c[1][0] - d[1] * c[2][0]);
//		r0 = this->halfSizes.y * absC[2][0] + this->halfSizes.z * absC[1][0];
//		r1 = other.halfSizes.y * absC[0][2] + other.halfSizes.z * absC[0][1];
//		if (r > r0 + r1)
//			return false;
//	}
//	/* axis C0 + t * A0 x B1 */
//	{
//		r = glm::abs(d[2] * c[1][1] - d[1] * c[2][1]);
//		r0 = this->halfSizes.y * absC[2][1] + this->halfSizes.z * absC[1][1];
//		r1 = other.halfSizes.x * absC[0][2] + other.halfSizes.z * absC[0][0];
//		if (r > r0 + r1)
//			return false;
//	}
//	/* axis C0 + t * A0 x B2 */
//	{
//		r = glm::abs(d[2] * c[1][2] - d[1] * c[2][2]);
//		r0 = this->halfSizes.y * absC[2][2] + this->halfSizes.z * absC[1][2];
//		r1 = other.halfSizes.x * absC[0][1] + other.halfSizes.y * absC[0][0];
//		if (r > r0 + r1)
//			return false;
//	}
//	/* axis C0 + t * A1 x B0 */
//	{
//		r = glm::abs(d[0] * c[2][0] - d[2] * c[0][0]);
//		r0 = this->halfSizes.x * absC[2][0] + this->halfSizes.z * absC[0][0];
//		r1 = other.halfSizes.y * absC[1][2] + other.halfSizes.z * absC[1][1];
//		if (r > r0 + r1)
//			return false;
//	}
//	/* axis C0 + t * A1 x B1 */
//	{
//		r = glm::abs(d[0] * c[2][1] - d[2] * c[0][1]);
//		r0 = this->halfSizes.x * absC[2][1] + this->halfSizes.z * absC[0][1];
//		r1 = other.halfSizes.x * absC[1][2] + other.halfSizes.z * absC[1][0];
//		if (r > r0 + r1)
//			return false;
//	}
//	/* axis C0 + t * A1 x B2 */
//	{
//		r = glm::abs(d[0] * c[2][2] - d[2] * c[0][2]);
//		r0 = this->halfSizes.x * absC[2][2] + this->halfSizes.z * absC[0][2];
//		r1 = other.halfSizes.x * absC[1][1] + other.halfSizes.y * absC[1][0];
//		if (r > r0 + r1)
//			return false;
//	}
//	/* axis C0 + t * A2 x B0 */
//	{
//		r = glm::abs(d[1] * c[0][0] - d[0] * c[1][0]);
//		r0 = this->halfSizes.x * absC[1][0] + this->halfSizes.y * absC[0][0];
//		r1 = other.halfSizes.y * absC[2][2] + other.halfSizes.z * absC[2][1];
//		if (r > r0 + r1)
//			return false;
//	}
//	/* axis C0 + t * A2 x B1 */
//	{
//		r = glm::abs(d[1] * c[0][1] - d[0] * c[1][1]);
//		r0 = this->halfSizes.x * absC[1][1] + this->halfSizes.y * absC[0][1];
//		r1 = other.halfSizes.x * absC[2][2] + other.halfSizes.z * absC[2][0];
//		if (r > r0 + r1)
//			return false;
//	}
//	/* axis C0 + t * A2 x B2 */
//	{
//		r = glm::abs(d[1] * c[0][2] - d[0] * c[1][2]);
//		r0 = this->halfSizes.x * absC[1][2] + this->halfSizes.y * absC[0][2];
//		r1 = other.halfSizes.x * absC[2][1] + other.halfSizes.t * absC[2][0];
//		if (r > r0 + r1)
//			return false;
//	}
//	return true;
//}

bool OBBCollider::testIntersectionOBB(OBBCollider & other)
{
	/* Christer Ericson - Real Time Collision Detection - 4.4 Oriented Bounding Boxes(OBBs) */
	const float EPSILON = 0.0001f;
	
	float ra, rb;
	glm::mat3 R, AbsR;


	glm::mat3 rotationMatA = glm::inverse(glm::mat3_cast(this->orientation));
	glm::mat3 rotationMatB = glm::inverse(glm::mat3_cast(other.orientation));

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R[i][j] = glm::dot(glm::column(rotationMatA, i), glm::column(rotationMatB, j));

	glm::vec3 t = other.position - this->position;
	t = glm::vec3(glm::dot(t, glm::column(rotationMatA, 0)), glm::dot(t, glm::column(rotationMatA, 1)), glm::dot(t, glm::column(rotationMatA, 2)));

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			AbsR[i][j] = glm::abs(R[i][j]) + EPSILON;

	// Test axes L = A0, L = A1, L = A2
	for (int i = 0; i < 3; i++) {
		ra = this->halfSizes[i];
		rb = other.halfSizes[0] * AbsR[i][0] + other.halfSizes[1] * AbsR[i][1] + other.halfSizes[2] * AbsR[i][2];
		if (glm::abs(t[i]) > ra + rb)
			return 0;
	}
	// Test axes L = B0, L = B1, L = B2
	for (int i = 0; i < 3; i++) {
		ra = this->halfSizes[0] * AbsR[0][i] + this->halfSizes[1] * AbsR[1][i] + this->halfSizes[2] * AbsR[2][i];
		rb = other.halfSizes[i];
		if (glm::abs(t[0] * R[0][i] + t[1] * R[1][i] + t[2] * R[2][i]) > ra + rb)
			return 0;
	}

	// Test axis L = A0 x B0
	ra = this->halfSizes[1] * AbsR[2][0] + this->halfSizes[2] * AbsR[1][0];
	rb = other.halfSizes[1] * AbsR[0][2] + other.halfSizes[2] * AbsR[0][1];
	if (glm::abs(t[2] * R[1][0] - t[1] * R[2][0]) > ra + rb) 
		return 0;
	// Test axis L = A0 x B1
	ra = this->halfSizes[1] * AbsR[2][1] + this->halfSizes[2] * AbsR[1][1];
	rb = other.halfSizes[0] * AbsR[0][2] + other.halfSizes[2] * AbsR[0][0];
	if (glm::abs(t[2] * R[1][1] - t[1] * R[2][1]) > ra + rb)
		return 0;
	// Test axis L = A0 x B2
	ra = this->halfSizes[1] * AbsR[2][2] + this->halfSizes[2] * AbsR[1][2];
	rb = other.halfSizes[0] * AbsR[0][1] + other.halfSizes[1] * AbsR[0][0];
	if (glm::abs(t[2] * R[1][2] - t[1] * R[2][2]) > ra + rb) 
		return 0;
	// Test axis L = A1 x B0
	ra = this->halfSizes[0] * AbsR[2][0] + this->halfSizes[2] * AbsR[0][0];
	rb = other.halfSizes[1] * AbsR[1][2] + other.halfSizes[2] * AbsR[1][1];
	if (glm::abs(t[0] * R[2][0] - t[2] * R[0][0]) > ra + rb) 
		return 0;
	// Test axis L = A1 x B1
	ra = this->halfSizes[0] * AbsR[2][1] + this->halfSizes[2] * AbsR[0][1];
	rb = other.halfSizes[0] * AbsR[1][2] + other.halfSizes[2] * AbsR[1][0];
	if (glm::abs(t[0] * R[2][1] - t[2] * R[0][1]) > ra + rb)
		return 0;
	// Test axis L = A1 x B2
	ra = this->halfSizes[0] * AbsR[2][2] + this->halfSizes[2] * AbsR[0][2];
	rb = other.halfSizes[0] * AbsR[1][1] + other.halfSizes[1] * AbsR[1][0];
	if (glm::abs(t[0] * R[2][2] - t[2] * R[0][2]) > ra + rb) 
		return 0;
	// Test axis L = A2 x B0
	ra = this->halfSizes[0] * AbsR[1][0] + this->halfSizes[1] * AbsR[0][0];
	rb = other.halfSizes[1] * AbsR[2][2] + other.halfSizes[2] * AbsR[2][1];
	if (glm::abs(t[1] * R[0][0] - t[0] * R[1][0]) > ra + rb) 
		return 0;
	// Test axis L = A2 x B1
	ra = this->halfSizes[0] * AbsR[1][1] + this->halfSizes[1] * AbsR[0][1];
	rb = other.halfSizes[0] * AbsR[2][2] + other.halfSizes[2] * AbsR[2][0];
	if (glm::abs(t[1] * R[0][1] - t[0] * R[1][1]) > ra + rb) 
		return 0;
	// Test axis L = A2 x B2
	ra = this->halfSizes[0] * AbsR[1][2] + this->halfSizes[1] * AbsR[0][2];
	rb = other.halfSizes[0] * AbsR[2][1] + other.halfSizes[1] * AbsR[2][0];
	if (glm::abs(t[1] * R[0][2] - t[0] * R[1][2]) > ra + rb) 
		return 0;

	// Since no separating axis is found, the OBBs must be intersecting
	return 1;
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

	for (unsigned int i = 0; i < obbVector.size() - 1; i++) {
		/* look for OBB-OBB collisions */
		for (unsigned int j = i + 1; j < obbVector.size(); j++)
			if (obbVector[i]->testIntersectionOBB(*obbVector[j])) {
				potentialCollisions.push_back(new PotentialCollision(obbVector[i]->phyObject, obbVector[j]->phyObject));
			}
		/* look for OBB-Plane collisions */
		/*for (auto & p : planeVector)
			if (i->testIntersectionPlane(*p)) {
				potentialCollisions.push_back(new PotentialCollision(i->phyObject, p->phyObject));
			}*/
	}
}

PlaneCollider::PlaneCollider(glm::vec3 normal, float offset, PhysicsObject * phyObject, Mesh * mesh)
	: normal(normal), offset(offset)
{
	this->mesh = mesh;
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
