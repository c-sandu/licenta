#pragma once

#include <Core/GPU/Mesh.h>
#include <Physics/body/rigidbody.h>
#include <Physics/collision/contact.h>
#include <Physics/scenes/object.h>

#include "broadphase.h"

namespace GJKEPA {
/* class describing a Minkowski Difference point */
class MinkowskiDiffPt {
 public:
  glm::vec3 v;    /* Minkowski difference point = supA - supB */
  glm::vec3 supA; /* support point on objA's shape */
  glm::vec3 supB; /* support point on objB's shape */

  MinkowskiDiffPt(glm::vec3 v, glm::vec3 supA, glm::vec3 supB)
      : v(v), supA(supA), supB(supB) {}
  MinkowskiDiffPt() {}
};

/* class describing a simplex used in GJK and EPA */
class Simplex {
 public:
  unsigned int numVertices; /* number of vertices in the simplex */
  MinkowskiDiffPt a;        /* most recently added vertex */
  MinkowskiDiffPt b;        /* 2nd most recently added vertex */
  MinkowskiDiffPt c;        /* 3rd most recently added vertex */
  MinkowskiDiffPt d;        /* 4th most recently added vertex */

  Simplex() { numVertices = 0; }

  /* clears all vertices from the simplex */
  void clear() { numVertices = 0; }
  /* sets the simplex as a tetrahedron */
  void set(MinkowskiDiffPt a,
           MinkowskiDiffPt b,
           MinkowskiDiffPt c,
           MinkowskiDiffPt d) {
    numVertices = 4;
    this->a = a;
    this->b = b;
    this->c = c;
    this->d = d;
  }
  /* sets the simplex as a triangle */
  void set(MinkowskiDiffPt a, MinkowskiDiffPt b, MinkowskiDiffPt c) {
    numVertices = 3;
    this->a = a;
    this->b = b;
    this->c = c;
  }
  /* sets the simplex as a line */
  void set(MinkowskiDiffPt a, MinkowskiDiffPt b) {
    numVertices = 2;
    this->a = a;
    this->b = b;
  }
  /* sets the simplex as a point */
  void set(MinkowskiDiffPt a) {
    numVertices = 1;
    this->a = a;
  }

  /* adds one new point to the simplex */
  void pushVertex(MinkowskiDiffPt p) {
    numVertices = glm::min(numVertices + 1, 4u);
    d = c;
    c = b;
    b = a;
    a = p;
  }
};

/* class describing a CCW triangle for EPA */
class Triangle {
 public:
  MinkowskiDiffPt a; /* triangle's vertex A */
  MinkowskiDiffPt b; /* triangle's vertex B */
  MinkowskiDiffPt c; /* triangle's vertex C */
  glm::vec3 vecABC;  /* triangle's normal, pointing away from origin */

  Triangle(const MinkowskiDiffPt& a,
           const MinkowskiDiffPt& b,
           const MinkowskiDiffPt& c) {
    this->a = a;
    this->b = b;
    this->c = c;
    this->vecABC = glm::normalize(glm::cross(b.v - a.v, c.v - a.v));
  }
};

/* class describing an edge for EPA*/
class Edge {
 public:
  MinkowskiDiffPt a; /* edge's vertex A */
  MinkowskiDiffPt b; /* edge's vertex B */

  Edge(const MinkowskiDiffPt& a, const MinkowskiDiffPt& b) : a(a), b(b) {}
};

/* an individual collision point generator utilizing GJK and EPA */
class GJKEPACollisionPointGenerator {
 private:
  PhysicsObject* objA; /* pointer to the first object in the collision */
  PhysicsObject* objB; /* pointer to the second object in the collision */

  Simplex simplex;     /* the simplex onject used by this generator */
  glm::vec3 searchDir; /* the current search direction for the GJK algorithm */

 public:
  /* produces an individual collision point generator for 2 physics objects */
  GJKEPACollisionPointGenerator(PhysicsObject* objA, PhysicsObject* objB);

 private:
  /* computes a support point on the Minkowski difference of the 2 objects*/
  MinkowskiDiffPt support(const glm::vec3& dir);
  /* handles the simplex = line case */
  void doSimplex2();
  /* handles the simplex = triangle case */
  void doSimplex3();
  /* handles the simplex = tetrahedron case */
  bool doSimplex4();
  /* simplex update routine of the GJK algorithm*/
  bool doSimplex();

 public:
  /* uses GJK for accurate test of intersection */
  bool testIntersection();

  /* uses EPA to generate a contact point */
  bool createCollisionPoint(CollisionPoint* contact);
};
}  /* end of namespace GJKEPA */

/* describes a collision point generator that computes all collision points
   between the pairs of potential collisions */
class CollisionsGenerator {
 public:
  std::
      vector<PotentialCollision*>*
          potentialCollisions; /* pointer to the potential collision vector
                                  created in the broad phase */
  std::vector<CollisionPoint*>
      collisionsPoints; /* vector of all collision points in the scene */

  CollisionsGenerator(){};
  CollisionsGenerator(std::vector<PotentialCollision*>* potentialCollisions)
      : potentialCollisions(potentialCollisions){};

  void fillCollisions();
  void clearCollisions();
};