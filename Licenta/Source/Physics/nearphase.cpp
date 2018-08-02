#include "nearphase.h"

#include <list>
#include <limits>
#include <stdio.h>
#include <glm/gtx/matrix_decompose.hpp>

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

	glm::vec3 maxA, maxB;
	float maxProjA, maxProjB;
	glm::vec3 dirNormalized = glm::normalize(dir);

	maxProjA = glm::dot(dirNormalized, verticesA[0]);
	maxA = verticesA[0];
	for (glm::vec3 &v : verticesA) {
		float temp = glm::dot(dirNormalized, v);
		if (temp > maxProjA) {
			maxProjA = temp;
			maxA = v;
		}
	}

	maxProjB = glm::dot(-dirNormalized, verticesB[0]);
	maxB = verticesB[0];
	for (glm::vec3 &v : verticesB) {
		float temp = glm::dot(-dirNormalized, v);
		if (temp > maxProjB) {
			maxProjB = temp;
			maxB = v;
		}
	}
	glm::vec3 scaleA, translationA, skewA;
	glm::quat orientationA;
	glm::vec4 perspectiveA;
	glm::vec3 scaleB, translationB, skewB;
	glm::quat orientationB;
	glm::vec4 perspectiveB;

	glm::decompose(transformA, scaleA, orientationA, translationA, skewA, perspectiveA);
	glm::decompose(transformB, scaleB, orientationB, translationB, skewB, perspectiveB);

	glm::vec3 maxALocal, maxBLocal;
	glm::vec3 maxAWorld, maxBWorld;


	glm::vec3 dirALocal = glm::inverse(glm::mat3_cast(orientationA)) * glm::normalize(dir);
	glm::vec3 dirBLocal = glm::inverse(glm::mat3_cast(orientationB)) * glm::normalize(-dir);

	maxALocal = objA->shape->getSupportPtInLocalSpace(glm::normalize(dirALocal));
	maxBLocal = objB->shape->getSupportPtInLocalSpace(glm::normalize(dirBLocal));

	/*maxAWorld = glm::vec3(transformA * glm::vec4(maxALocal, 1));
	maxBWorld = glm::vec3(transformB * glm::vec4(maxBLocal, 1));*/

	maxAWorld = glm::translate(glm::mat4(1), translationA) * glm::mat4_cast(orientationA) * glm::scale(glm::mat4(1), scaleA) * glm::vec4(maxALocal, 1);
	maxBWorld = glm::translate(glm::mat4(1), translationB) * glm::mat4_cast(orientationB) * glm::scale(glm::mat4(1), scaleB) * glm::vec4(maxBLocal, 1);

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
			if (glm::dot(vecAB, vecABC) > 0) {
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
	unsigned int iterations = 0;

	simplex.clear();

	searchDir = glm::vec3(1, 0, 0);
	SupportPoint initialPoint = support(searchDir);

	if (glm::abs(glm::dot(searchDir, initialPoint.v)) >= initialPoint.v.length() * 0.8f) {
		searchDir = glm::vec3(0, 1, 0);
		initialPoint = support(searchDir);
	}

	simplex.set(initialPoint);
	searchDir = -initialPoint.v;

	while (true) {
		if (iterations++ > GJK_MAX_ITERATIONS) {
			std::cout << "max iterations reached\n";
			std::cout << "simplex = {\n\t";
			if (simplex.numVertices > 0)
				std::cout << "A = " << simplex.a.v << "\n\t";
			if (simplex.numVertices > 1)
				std::cout << "B = " << simplex.b.v << "\n\t";
			if (simplex.numVertices > 2)
				std::cout << "C = " << simplex.c.v << "\n\t";
			if (simplex.numVertices > 3)
				std::cout << "D = " << simplex.c.v << "\n}\n";
			return false;
		}

		SupportPoint supPt = support(searchDir);
		//std::cout << "supPt = {\n\tsupPt.v = " << supPt.v << "\n\tsupPt.supA = " << supPt.supA << "\n\tsupPt.supB = " << supPt.supB << "\n}\n";
		/* new point is not past the origin */
		if (glm::dot(supPt.v, searchDir) < 0) {
			std::cout << "new point is not past the origin \n";
			return false;
		}

		simplex.pushVertex(supPt);

		if (doSimplex()) {
			std::cout << "origin inside tetrahedron \n";
			return true;
		}
	}

	std::cout << "GJK intersection test returned false \n";
	return false;
}

static void addRemoveEdge(std::list<GJK::Edge> &edges, const GJK::SupportPoint &a, const GJK::SupportPoint &b)
{
	std::list<GJK::Edge>::iterator it = edges.begin();
	while (it != edges.end()) {
		if (it->supA.v == b.v && it->supB.v == a.v) {
			/* found reversed edge, just remove it */
			it = edges.erase(it);
			return;
		}
		it++;
	}
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
	const float EPA_GROWTH_THRESHOLD = 0.0001f;
	const unsigned int EPA_MAX_ITERATIONS = 50;
	unsigned int iterations = 0;

	std::list<Triangle> triangles;
	std::list<Edge> edges;

	triangles.push_back(Triangle(simplex.a, simplex.b, simplex.c));
	triangles.push_back(Triangle(simplex.a, simplex.d, simplex.b));
	triangles.push_back(Triangle(simplex.a, simplex.c, simplex.d));
	triangles.push_back(Triangle(simplex.b, simplex.d, simplex.c));

	std::vector<float> dotProducts;
	for (Triangle & t : triangles) {
		dotProducts.push_back(glm::dot(-t.supA.v, t.vecABC));
		if (dotProducts[dotProducts.size() - 1] < 0)
			std::cout << " NOT OKAY";
	}
	std::cout << "Simplex = {\n\t";
	std::cout << "A = " << simplex.a.v << "\n\t";
	std::cout << "B = " << simplex.b.v << "\n\t";
	std::cout << "C = " << simplex.c.v << "\n\t";
	std::cout << "D = " << simplex.d.v << "\n}\n\n";


	while (iterations < EPA_MAX_ITERATIONS) {
		Triangle *closestTriangle = &triangles.front();
		float minDistance = FLT_MAX;
		for (Triangle t : triangles) {
			/* distance to origin */
			float distance = glm::length(t.supA.v - glm::dot(t.vecABC, t.supA.v) * t.vecABC);
			if (distance < minDistance) {
				minDistance = distance;
				closestTriangle = &t;
			}
		}

		/* get furthest point in the triangle's normal direction, opposite to the origin */
		SupportPoint nextSup = support(closestTriangle->vecABC);
		/* distance from origin to the next point to be added to the expanding polytope */
		float nextSupDist = glm::dot(nextSup.v, closestTriangle->vecABC);

		/* if the new point is not further from the origin than the face in whose direction we
		are expanding the polytope, we can't expand anymore */
		if (nextSupDist - minDistance < EPA_GROWTH_THRESHOLD) {
			/* TODO: fill contact info */
			glm::vec3 barCoords;

			/* compute barycentric coordinates of origin's projection on the triangle with respect to this
			closest triangle */
			computeBarycentricCoords(closestTriangle->supA.v,
				closestTriangle->supB.v,
				closestTriangle->supC.v,
				closestTriangle->vecABC * minDistance,
				barCoords);

			if (isnan(barCoords.x) || isnan(barCoords.y) || isnan(barCoords.z))
				return false;

			if (glm::abs(barCoords.x) > 1.0f || glm::abs(barCoords.y) > 1.0f || glm::abs(barCoords.z) > 1.0f)
				return false;
			contact->point = glm::vec3(
				barCoords.x * closestTriangle->supA.supA +
				barCoords.y * closestTriangle->supB.supA +
				barCoords.z * closestTriangle->supC.supA);

			contact->normal = -closestTriangle->vecABC;

			contact->penetration = minDistance;

			contact->objA = objA;
			contact->objB = objB;

			std::cout << "EPA found contact point in barycentric coordinates$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n";
			return true;
		}

		if (glm::dot(closestTriangle->vecABC, nextSup.v - closestTriangle->supA.v) < 0)
			std::cout << "NOT OK";

		std::list<Triangle>::iterator it = triangles.begin();
		while (it != triangles.end()) {
			if (glm::dot(it->vecABC, nextSup.v - it->supA.v) > 0) {
				/* update the edge list in order to remove the triangles facing this point */
				addRemoveEdge(edges, it->supA, it->supC);
				addRemoveEdge(edges, it->supC, it->supB);
				addRemoveEdge(edges, it->supB, it->supA);
				it = triangles.erase(it);
			}
			else {
				it++;
			}
		}
		/* re-create the triangles from the remaining edges */
		for (Edge e : edges) {
			glm::vec3 normal = glm::normalize(glm::cross(e.supA.v - nextSup.v, e.supB.v - nextSup.v));
			if (glm::isnan(glm::dot(nextSup.v, normal)))
				continue;
			if (glm::dot(normal, nextSup.v) < 0)
				triangles.push_back(Triangle(nextSup, e.supA, e.supB));
			else
				triangles.push_back(Triangle(nextSup, e.supB, e.supA));
		}

		/*for (Triangle t : triangles) {
		if (glm::dot(t.vecABC, t.supA.v) < 0)
		std::cout << "not ok";
		}*/
		edges.clear();
		iterations++;
	}

	std::cout << "EPA max iterations reached\n";
	return false;
}
