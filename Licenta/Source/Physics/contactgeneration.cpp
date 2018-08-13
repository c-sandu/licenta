#include "contactgeneration.h"

#include <list>
#include <limits>
#include <glm/gtx/matrix_decompose.hpp>
#include "debug.h"

GJK::GJKContactGenerator::GJKContactGenerator(PhysicsObject *objA, PhysicsObject *objB)
	: objA(objA), objB(objB)
{
	transformA = objA->getTransformMatrix();
	transformB = objB->getTransformMatrix();

	meshA = objA->mesh;
	meshB = objB->mesh;

	/* TODO: see if removing duplicates helps a great deal */
	for (glm::vec3 &v : meshA->positions)
		verticesA.push_back(glm::vec3(transformA * glm::vec4(v, 1)));

	for (glm::vec3 &v : meshB->positions)
		verticesB.push_back(glm::vec3(transformB * glm::vec4(v, 1)));
}

GJK::SupportPoint GJK::GJKContactGenerator::support(const glm::vec3 & dir, bool initialSupport)
{
	/* TODO: http://uu.diva-portal.org/smash/get/diva2:343820/FULLTEXT01 slide 16 for accurate support mappings */
	if (initialSupport)
		return SupportPoint(verticesA[0] - verticesB[0], verticesA[0], verticesB[0]);

	glm::vec3 dirNormalized = glm::normalize(dir);

	glm::vec3 scaleA = objA->body->scale;
	glm::vec3 translationA = objA->body->position;
	glm::quat orientationA = objA->body->orientation;
	glm::vec3 scaleB = objB->body->scale;
	glm::vec3 translationB = objB->body->position;
	glm::quat orientationB = objB->body->orientation;

	glm::vec3 maxALocal, maxBLocal;
	glm::vec3 maxAWorld, maxBWorld;

	glm::vec3 dirALocal = /*glm::mat3(glm::scale(glm::mat4(1), 1.0f / scaleA)) **/ glm::inverse(glm::mat3_cast(orientationA)) * dirNormalized;
	glm::vec3 dirBLocal = /*glm::mat3(glm::scale(glm::mat4(1), 1.0f / scaleB)) **/ glm::inverse(glm::mat3_cast(orientationB)) * -dirNormalized;

	maxALocal = glm::mat3(glm::scale(glm::mat4(1), 1.0f / scaleA)) * objA->shape->getSupportPtInLocalSpace(glm::normalize(dirALocal));
	maxBLocal = glm::mat3(glm::scale(glm::mat4(1), 1.0f / scaleB)) * objB->shape->getSupportPtInLocalSpace(glm::normalize(dirBLocal));

	/*maxAWorld = glm::vec3(transformA * glm::vec4(maxALocal, 1));
	maxBWorld = glm::vec3(transformB * glm::vec4(maxBLocal, 1));*/

	//maxAWorld = glm::translate(glm::mat4(1), translationA) * glm::mat4_cast(orientationA) * glm::scale(glm::mat4(1), scaleA) * glm::vec4(maxALocal, 1);
	//maxBWorld = glm::translate(glm::mat4(1), translationB) * glm::mat4_cast(orientationB) * glm::scale(glm::mat4(1), scaleB) * glm::vec4(maxBLocal, 1);
	maxAWorld = objA->body->transform * glm::vec4(maxALocal, 1);
	maxBWorld = objB->body->transform * glm::vec4(maxBLocal, 1);

	return SupportPoint(maxAWorld - maxBWorld, maxAWorld, maxBWorld);
	//return SupportPoint(maxA - maxB, maxA, maxB);
}

void GJK::GJKContactGenerator::doSimplex2()
{
	/* simplex is an edge \vec{AB}, A was just added */
	glm::vec3 vecAO = -simplex.a.v;
	glm::vec3 vecAB = simplex.b.v - simplex.a.v;
	/* origin is in the direction of \vec{AB}
	search direction is perpendicular to \vec{AB} and coplanar with \vec{AO} */
	if (glm::dot(vecAB, vecAO) > 0) {
		searchDir = glm::normalize(glm::cross(glm::cross(vecAB, vecAO), vecAB));
	}
	else {
		simplex.set(simplex.a);
		searchDir = glm::normalize(vecAO);
	}
}

void GJK::GJKContactGenerator::doSimplex3()
{
	/* simplex is a triangle {\triangle}ABC, A was just added */
	glm::vec3 vecAO = -simplex.a.v;
	glm::vec3 vecAB = simplex.b.v - simplex.a.v;
	glm::vec3 vecAC = simplex.c.v - simplex.a.v;
	glm::vec3 vecABC = glm::cross(vecAB, vecAC);

	if (glm::dot(glm::cross(vecABC, vecAC), vecAO) > 0) {
		if (glm::dot(vecAC, vecAO) > 0) {
			simplex.set(simplex.a, simplex.c);
			searchDir = glm::cross(glm::cross(vecAC, vecAO), vecAC);
		}
		else {
			if (glm::dot(vecAB, vecAO) > 0) {
				simplex.set(simplex.a, simplex.b);
				searchDir = glm::cross(glm::cross(vecAB, vecAO), vecAB);
			}
			else {
				simplex.set(simplex.a);
				searchDir = vecAO;
			}
		}
	}
	else {
		if (glm::dot(glm::cross(vecAB, vecABC), vecAO) > 0) {
			if (glm::dot(vecAB, vecAO) > 0) {
				simplex.set(simplex.a, simplex.b);
				searchDir = glm::cross(glm::cross(vecAB, vecAO), vecAB);
			}
			else {
				simplex.set(simplex.a);
				searchDir = vecAO;
			}
		}
		else {
			if (glm::dot(vecABC, vecAO) > 0) {
				searchDir = vecABC;
			}
			else {
				simplex.set(simplex.a, simplex.c, simplex.b);
				searchDir = -vecABC;
			}
		}
	}
}

bool GJK::GJKContactGenerator::doSimplex4()
{
	glm::vec3 vecAO = -simplex.a.v;
	glm::vec3 vecAB = simplex.b.v - simplex.a.v;
	glm::vec3 vecAC = simplex.c.v - simplex.a.v;
	glm::vec3 vecAD = simplex.d.v - simplex.a.v;
	glm::vec3 vecABC = glm::cross(vecAB, vecAC);
	glm::vec3 vecACD = glm::cross(vecAC, vecAD);
	glm::vec3 vecADB = glm::cross(vecAD, vecAB);

	uint8_t directionCode = 0;

	if (glm::dot(vecABC, vecAO) > 0)
		directionCode |= 0x1;
	if (glm::dot(vecACD, vecAO) > 0)
		directionCode |= 0x2;
	if (glm::dot(vecADB, vecAO) > 0)
		directionCode |= 0x4;

	switch (directionCode)
	{
	case 0:
		return true;
	case 1: {
		simplex.set(simplex.a, simplex.b, simplex.c);
		doSimplex3();
		break;
	}
	case 2: {
		simplex.set(simplex.a, simplex.c, simplex.d);
		doSimplex3();
		break;
	}
	case 3: {
		if (glm::dot(glm::cross(vecABC, vecAC), vecAO) > 0) {
			if (glm::dot(glm::cross(vecAC, vecACD), vecAO) > 0) {
				if (glm::dot(vecAC, vecAO) > 0) {
					simplex.set(simplex.a, simplex.c);
					searchDir = glm::cross(glm::cross(vecAC, vecAO), vecAC);
				}
				else {
					simplex.set(simplex.a);
					searchDir = -simplex.a.v;
				}
			}
			else {
				if (glm::dot(glm::cross(vecACD, vecAD), vecAO) > 0) {
					simplex.set(simplex.a, simplex.d);
					searchDir = glm::cross(glm::cross(vecAD, vecAO), vecAD);
				}
				else {
					simplex.set(simplex.a, simplex.c, simplex.d);
					searchDir = vecACD;
				}
			}
		}
		else {
			if (glm::dot(glm::cross(vecAB, vecABC), vecAO) > 0) {
				if (glm::dot(vecAB, vecAO) > 0) {
					simplex.set(simplex.a, simplex.b);
					searchDir = glm::cross(glm::cross(vecAB, vecAO), vecAB);
				}
				else {
					simplex.set(simplex.a);
					searchDir = -simplex.a.v;
				}
			}
			else {
				simplex.set(simplex.a, simplex.b, simplex.c);
				searchDir = vecABC;
			}
		}
		break;
	}
	case 4: {
		simplex.set(simplex.a, simplex.d, simplex.b);
		doSimplex3();
		break;
	}
	case 5: {
		if (glm::dot(glm::cross(vecADB, vecAB), vecAO) > 0) {
			if (glm::dot(glm::cross(vecAB, vecABC), vecAO) > 0) {
				if (glm::dot(vecAB, vecAO) > 0)
				{
					simplex.set(simplex.a, simplex.b);
					searchDir = glm::cross(glm::cross(vecAB, vecAO), vecAB);
				}
				else {
					simplex.set(simplex.a);
					searchDir = -simplex.a.v;
				}
			}
			else {
				if (glm::dot(glm::cross(vecABC, vecAC), vecAO) > 0) {
					simplex.set(simplex.a, simplex.c);
					searchDir = glm::cross(glm::cross(vecAC, vecAO), vecAC);
				}
				else {
					simplex.set(simplex.a, simplex.b, simplex.c);
					searchDir = vecABC;
				}
			}
		}
		else {
			if (glm::dot(glm::cross(vecAD, vecADB), vecAO) > 0) {
				if (glm::dot(vecAD, vecAO) > 0) {
					simplex.set(simplex.a, simplex.d);
					searchDir = glm::cross(glm::cross(vecAD, vecAO), vecAD);
				}
				else {
					simplex.set(simplex.a);
					searchDir = -simplex.a.v;
				}
			}
			else {
				simplex.set(simplex.a, simplex.d, simplex.b);
				searchDir = vecADB;
			}
		}
		break;
	}
	case 6: {
		if (glm::dot(glm::cross(vecACD, vecAD), vecAO) > 0) {
			if (glm::dot(glm::cross(vecAD, vecADB), vecAO) > 0) {
				if (glm::dot(vecAD, vecAO) > 0) {
					simplex.set(simplex.a, simplex.d);
					searchDir = glm::cross(glm::cross(vecAD, vecAO), vecAD);
				}
				else {
					simplex.set(simplex.a);
					searchDir = -simplex.a.v;
				}
			}
			else {
				if (glm::dot(glm::cross(vecADB, vecAB), vecAO) > 0) {
					simplex.set(simplex.a, simplex.b);
					searchDir = glm::cross(glm::cross(vecAB, vecAO), vecAB);
				}
				else {
					simplex.set(simplex.a, simplex.d, simplex.b);
					searchDir = vecADB;
				}
			}
		}
		else {
			if (glm::dot(glm::cross(vecAC, vecACD), vecAO) > 0) {
				if (glm::dot(vecAC, vecAO) > 0) {
					simplex.set(simplex.a, simplex.c);
					searchDir = glm::cross(glm::cross(vecAC, vecAO), vecAC);
				}
				else {
					simplex.set(simplex.a);
					searchDir = -simplex.a.v;
				}
			}
			else {
				simplex.set(simplex.a, simplex.c, simplex.d);
				searchDir = vecACD;
			}
		}
		break;
	}
	case 7: {
		if (glm::dot(vecAB, vecAO) > 0) {
			simplex.set(simplex.a, simplex.b);
			searchDir = glm::cross(glm::cross(vecAB, vecAO), vecAB);
		}
		else {
			if (glm::dot(vecAC, vecAO) > 0) {
				simplex.set(simplex.a, simplex.c);
				searchDir = glm::cross(glm::cross(vecAC, vecAO), vecAC);
			}
			else {
				if (glm::dot(vecAD, vecAO) > 0) {
					simplex.set(simplex.a, simplex.d);
					searchDir = glm::cross(glm::cross(vecAD, vecAO), vecAD);
				}
				else {
					simplex.set(simplex.a);
					searchDir = -simplex.a.v;
				}
			}
		}
		break;
	}
	}
	return false;
}

bool GJK::GJKContactGenerator::doSimplex()
{
	switch (simplex.numVertices) {
	case 2: {
		doSimplex2();
		break;
	}
	case 3: {
		doSimplex3();
		break;
	}
	case 4: {
		if (doSimplex4())
			return true;
		break;
	}
	}
	return false;
}

bool GJK::GJKContactGenerator::testIntersection()
{
	const unsigned int GJK_MAX_ITERATIONS = 100;
	const float GJK_EPSILON = 0.0001f;
	unsigned int iterations = 0;

	simplex.clear();

	searchDir = glm::vec3(0, 1, 0);
	SupportPoint initialPoint = support(searchDir);

	if (glm::abs(glm::dot(searchDir, initialPoint.v)) >= initialPoint.v.length() * 0.8f) {
		searchDir = glm::vec3(1, 0, 0);
		initialPoint = support(searchDir);
	}

	simplex.set(initialPoint);
	searchDir = -initialPoint.v;

	while (true) {
		if (iterations++ > GJK_MAX_ITERATIONS) {
			DEBUG_PRINT("max iterations reached\n");
			DEBUG_PRINT("simplex = {\n\t");
			if (simplex.numVertices > 0)
				DEBUG_PRINT("A = " << simplex.a.v << "\n\t");
			if (simplex.numVertices > 1)
				DEBUG_PRINT("B = " << simplex.b.v << "\n\t");
			if (simplex.numVertices > 2)
				DEBUG_PRINT("C = " << simplex.c.v << "\n\t");
			if (simplex.numVertices > 3)
				DEBUG_PRINT("D = " << simplex.c.v << "\n}\n");
			return false;
		}

		if (searchDir.length() <= GJK_EPSILON)
			return false;

		SupportPoint supPt = support(searchDir);
		//DEBUG_PRINT("supPt = {\n\tsupPt.v = " << supPt.v << "\n\tsupPt.supA = " << supPt.supA << "\n\tsupPt.supB = " << supPt.supB << "\n}\n");
		/* new point is not past the origin */
		if (glm::dot(supPt.v, searchDir) < -GJK_EPSILON) {
			DEBUG_PRINT("new point is not past the origin \n");
			return false;
		}

		simplex.pushVertex(supPt);

		if (doSimplex()) {
			DEBUG_PRINT("origin inside tetrahedron \n");
			return true;
		}
	}

	DEBUG_PRINT("GJK intersection test returned false \n");
	return false;
}

static void addRemoveEdge(std::list<GJK::Edge> &edges, const GJK::SupportPoint &a, const GJK::SupportPoint &b)
{
	for (auto it = edges.begin(); it != edges.end(); it++) {
		if (it->supA.v == b.v && it->supB.v == a.v) {
			/* found reversed edge, just remove it */
			it = edges.erase(it);
			return;
		}
	}
	if (edges.size() < 50)
		edges.push_back(GJK::Edge(a, b));
}

/* Compute barycentric coordinates barCoords(u, v, w) for point p
with respect to triangle (a, b, c) */
static void computeBarycentricCoords(const glm::vec3 &a, const glm::vec3 &b, const glm::vec3 &c,
	const glm::vec3 &p, glm::vec3 &barCoords)
{
	/* source: Christer Ericson - Real-Time Collision Detection, chap. 3.4 Barycentric Coordinates */
	glm::vec3 v0 = b - a, v1 = c - a, v2 = p - a;
	float d00 = glm::dot(v0, v0);
	float d01 = glm::dot(v0, v1);
	float d11 = glm::dot(v1, v1);
	float d20 = glm::dot(v2, v0);
	float d21 = glm::dot(v2, v1);
	float denom = d00 * d11 - d01 * d01;
	float inverseDenom = denom < 0.0f ? 0.0f : 1.0f / denom;
	barCoords.y = (d11 * d20 - d01 * d21) * inverseDenom;
	barCoords.z = (d00 * d21 - d01 * d20) * inverseDenom;
	barCoords.x = 1.0f - barCoords.y - barCoords.z;
}
bool GJK::GJKContactGenerator::createContact(ContactInfo *contact)
{
	const float EPA_GROWTH_THRESHOLD = 0.001f;
	const unsigned int EPA_MAX_ITERATIONS = 50;
	const float EPA_EPSILON = 0.001f;
	unsigned int iterations = 0;

	std::list<Triangle> triangles;
	std::list<Edge> edges;

	float test1, test2, test3, test4;
	triangles.push_back(Triangle(simplex.a, simplex.b, simplex.c));
	test1 = glm::dot(triangles.back().supA.v, triangles.back().vecABC);
	triangles.push_back(Triangle(simplex.a, simplex.d, simplex.b));
	test2 = glm::dot(triangles.back().supA.v, triangles.back().vecABC);
	triangles.push_back(Triangle(simplex.a, simplex.c, simplex.d));
	test3 = glm::dot(triangles.back().supA.v, triangles.back().vecABC);
	triangles.push_back(Triangle(simplex.b, simplex.d, simplex.c));
	test4 = glm::dot(triangles.back().supA.v, triangles.back().vecABC);

	std::vector<float> dotProducts;
	for (Triangle & t : triangles) {
		dotProducts.push_back(glm::dot(-t.supA.v, t.vecABC));
		if (dotProducts[dotProducts.size() - 1] > 0)
			DEBUG_PRINT(" NOT OKAY");
	}
	DEBUG_PRINT("Simplex = {\n\t");
	DEBUG_PRINT("A = " << simplex.a.v << "\n\t");
	DEBUG_PRINT("B = " << simplex.b.v << "\n\t");
	DEBUG_PRINT("C = " << simplex.c.v << "\n\t");
	DEBUG_PRINT("D = " << simplex.d.v << "\n}\n\n");


	while (iterations < EPA_MAX_ITERATIONS) {
		auto closestTriangleIt = triangles.begin();
		float minDistance = FLT_MAX;
		for (auto t = triangles.begin(); t != triangles.end(); t++) {
			/* distance to origin */
			//float distance = glm::length(t.supA.v - glm::dot(t.vecABC, t.supA.v) * t.vecABC);
			float distance = glm::dot(t->vecABC, t->supA.v);
			if (distance < minDistance) {
				/*glm::vec3 vecAB = t.supB.v - t.supA.v;
				glm::vec3 vecBC = t.supC.v - t.supB.v;
				glm::vec3 vecCA = t.supA.v - t.supC.v;

				if (glm::dot(glm::cross(vecAB, t.vecABC), -t.supA.v) > 0 ||
					glm::dot(glm::cross(vecBC, t.vecABC), -t.supB.v) > 0 ||
					glm::dot(glm::cross(vecCA, t.vecABC), -t.supC.v) > 0)
					continue;*/
				minDistance = distance;
				closestTriangleIt = t;
			}
		}

		/* get furthest point in the triangle's normal direction, opposite to the origin */
		SupportPoint nextSup = support(closestTriangleIt->vecABC);
		/* distance from origin to the next point to be added to the expanding polytope */
		float nextSupDist = glm::dot(nextSup.v, closestTriangleIt->vecABC);

		/* if the new point is not further from the origin than the face in whose direction we
		are expanding the polytope, we can't expand anymore */
		if (nextSupDist - minDistance < EPA_GROWTH_THRESHOLD) {
			/* TODO: fill contact info */
			glm::vec3 barCoords;

			//minDistance = glm::dot(closestTriangle->vecABC, closestTriangle->supA.v);

			/* compute barycentric coordinates of origin's projection on the triangle with respect to this
			closest triangle */
			computeBarycentricCoords(closestTriangleIt->supA.v,
				closestTriangleIt->supB.v,
				closestTriangleIt->supC.v,
				closestTriangleIt->vecABC * minDistance,
				barCoords);

			if (isnan(barCoords.x) || isnan(barCoords.y) || isnan(barCoords.z))
				return false;

			if (glm::abs(barCoords.x) > 1.0f || glm::abs(barCoords.y) > 1.0f || glm::abs(barCoords.z) > 1.0f)
				return false;
			contact->points[0] = glm::vec3(
				barCoords.x * closestTriangleIt->supA.supA +
				barCoords.y * closestTriangleIt->supB.supA +
				barCoords.z * closestTriangleIt->supC.supA);
			contact->points[1] = glm::vec3(
				barCoords.x * closestTriangleIt->supA.supB +
				barCoords.y * closestTriangleIt->supB.supB +
				barCoords.z * closestTriangleIt->supC.supB);

			contact->normal = -closestTriangleIt->vecABC;

			contact->penetration = minDistance;

			contact->objects[0] = objA;
			contact->objects[1] = objB;

			DEBUG_PRINT("EPA found contact point in barycentric coordinates$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
			return true;
		}

		if (glm::dot(closestTriangleIt->vecABC, nextSup.v - closestTriangleIt->supA.v) < 0) {
			DEBUG_PRINT("NOT OK");
			return false;
		}

		for (auto it = triangles.begin(); it != triangles.end();) {
			if (glm::dot(it->vecABC, nextSup.v - it->supA.v) > -EPA_EPSILON) {
				/* update the edge list in order to remove the triangles facing this point */
				addRemoveEdge(edges, it->supA, it->supC);
				addRemoveEdge(edges, it->supC, it->supB);
				addRemoveEdge(edges, it->supB, it->supA);
				it = triangles.erase(it);
				continue;
			}
			it++;
		}
		/* re-create the triangles from the remaining edges */
		for (Edge e : edges) {
			glm::vec3 normal = glm::normalize(glm::cross(e.supA.v - nextSup.v, e.supB.v - nextSup.v));
			if (glm::isnan(glm::dot(nextSup.v, normal)) || triangles.size() > 64)
				continue;
			if (glm::dot(normal, nextSup.v) > 0) {
				triangles.push_back(Triangle(nextSup, e.supA, e.supB));
				float test = glm::dot(nextSup.v, triangles.back().vecABC);
				int g = 0;
			}
			else {
				triangles.push_back(Triangle(nextSup, e.supB, e.supA));
				float test = glm::dot(nextSup.v, triangles.back().vecABC);
				int g = 0;
			}
		}

		/*for (Triangle t : triangles) {
		if (glm::dot(t.vecABC, t.supA.v) < 0)
		DEBUG_PRINT("not ok");
		}*/
		edges.clear();
		iterations++;
	}

	DEBUG_PRINT("EPA max iterations reached\n");
	return false;
}

void ContactGenerator::fillContacts()
{
	for (auto & col : (*potentialCollisions)) {
		GJK::GJKContactGenerator cg = GJK::GJKContactGenerator(col->one, col->two);

		if (cg.testIntersection()) {
			ContactInfo contactInfo;
			bool ok = cg.createContact(&contactInfo);
			if (ok) {
				collisions.push_back(new ContactInfo(contactInfo));
			}
		}
	}
}

void ContactGenerator::clearContacts()
{
	auto collisionIt = collisions.begin();
	while (collisionIt != collisions.end()) {
		delete *collisionIt;
		collisionIt++;
	}
	collisions.clear();
}
