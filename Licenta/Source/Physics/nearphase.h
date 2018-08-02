#pragma once

#include <Core/GPU/Mesh.h>
#include "rigidbody.h"
#include "object.h"

class ContactInfo
{
public:
	glm::vec3 point; /* point of contact in world coordinates */
	glm::vec3 normal; /* contact normal */
	float penetration; /* penetration depth */

	PhysicsObject *objA;
	PhysicsObject *objB;
};

namespace GJK
{
	class SupportPoint {
	public:
		glm::vec3 v; /* Minkowski difference point = supA - supB */
		glm::vec3 supA;
		glm::vec3 supB;

		SupportPoint(glm::vec3 v, glm::vec3 supA, glm::vec3 supB) : v(v), supA(supA), supB(supB) {}
		SupportPoint() {}
	};

	class Simplex
	{
	public:
		unsigned int numVertices;
		SupportPoint a;
		SupportPoint b;
		SupportPoint c;
		SupportPoint d;

		Simplex() { numVertices = 0; }
		
		void clear() { numVertices = 0; }
		void set(SupportPoint a, SupportPoint b, SupportPoint c, SupportPoint d) { numVertices = 4; this->a = a; this->b = b; this->c = c; this->d = d; }
		void set(SupportPoint a, SupportPoint b, SupportPoint c) { numVertices = 3; this->a = a; this->b = b; this->c = c; }
		void set(SupportPoint a, SupportPoint b) { numVertices = 2; this->a = a; this->b = b; }
		void set(SupportPoint a) { numVertices = 1; this->a = a; }

		void pushVertex(SupportPoint &p) {
			numVertices = glm::min(numVertices + 1, 4u);
			d = c;
			c = b;
			b = a;
			a = p;
		}

	};

	class Triangle {
	public:
		SupportPoint supA;
		SupportPoint supB;
		SupportPoint supC;
		glm::vec3 vecABC;

		Triangle(const SupportPoint &supA, const SupportPoint &supB, const SupportPoint &supC) {
			this->supA = supA;
			this->supB = supB;
			this->supC = supC;
			this->vecABC = glm::normalize(glm::cross(supC.v - supA.v, supB.v - supA.v));
		}
	};

	class Edge {
	public:
		SupportPoint supA;
		SupportPoint supB;

		Edge(const SupportPoint &supA, const SupportPoint &supB) : supA(supA), supB(supB) {}
	};

	class GJKContactGenerator
	{
		glm::mat4 transformA;
		glm::mat4 transformB;

		Mesh *meshA;
		Mesh *meshB;

		PhysicsObject *objA;
		PhysicsObject *objB;

		std::vector<glm::vec3> verticesA;
		std::vector<glm::vec3> verticesB;

		Simplex simplex;
		glm::vec3 searchDir;

	public:
		GJKContactGenerator(PhysicsObject *objA, PhysicsObject *objB);

		SupportPoint support(const glm::vec3 &dir, bool initialSupport = false);
		void doSimplex2();
		void doSimplex3();
		bool doSimplex4();
		bool doSimplex();

		bool testIntersection();
		bool createContact(ContactInfo *contact);
	};
}
