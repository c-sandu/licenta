#include "contactgeneration.h"

#include <Physics/debug.h>
#include <glm/gtx/matrix_decompose.hpp>
#include <limits>
#include <list>

GJKEPA::GJKEPACollisionPointGenerator::GJKEPACollisionPointGenerator(
    PhysicsObject* objA,
    PhysicsObject* objB)
    : objA(objA), objB(objB) {}

GJKEPA::MinkowskiDiffPt GJKEPA::GJKEPACollisionPointGenerator::support(
    const glm::vec3& dir) {
  glm::vec3 dirNormalized = glm::normalize(dir);

  glm::vec3 scaleA = objA->body->scale;
  glm::vec3 translationA = objA->body->position;
  glm::quat orientationA = objA->body->orientation;
  glm::vec3 scaleB = objB->body->scale;
  glm::vec3 translationB = objB->body->position;
  glm::quat orientationB = objB->body->orientation;

  glm::vec3 maxALocal, maxBLocal;
  glm::vec3 maxAWorld, maxBWorld;

  glm::vec3 dirALocal =
      glm::inverse(glm::mat3_cast(orientationA)) * dirNormalized;
  glm::vec3 dirBLocal =
      glm::inverse(glm::mat3_cast(orientationB)) * -dirNormalized;

  maxALocal = glm::mat3(glm::scale(glm::mat4(1), 1.0f / scaleA)) *
              objA->shape->getSupportPtInLocalSpace(glm::normalize(dirALocal));
  maxBLocal = glm::mat3(glm::scale(glm::mat4(1), 1.0f / scaleB)) *
              objB->shape->getSupportPtInLocalSpace(glm::normalize(dirBLocal));

  maxAWorld = objA->body->transform * glm::vec4(maxALocal, 1);
  maxBWorld = objB->body->transform * glm::vec4(maxBLocal, 1);

  return MinkowskiDiffPt(maxAWorld - maxBWorld, maxAWorld, maxBWorld);
}

void GJKEPA::GJKEPACollisionPointGenerator::doSimplex2() {
  /* simplex is an edge \vec{AB}, A was just added */
  glm::vec3 vecAO = -simplex.a.v;
  glm::vec3 vecAB = simplex.b.v - simplex.a.v;
  /* origin is in the direction of \vec{AB}
  search direction is perpendicular to \vec{AB} and coplanar with \vec{AO} */
  if (glm::dot(vecAB, vecAO) > 0) {
    searchDir = glm::normalize(glm::cross(glm::cross(vecAB, vecAO), vecAB));
  } else {
    simplex.set(simplex.a);
    searchDir = glm::normalize(vecAO);
  }
}

void GJKEPA::GJKEPACollisionPointGenerator::doSimplex3() {
  /* simplex is a triangle {\triangle}ABC, A was just added */
  glm::vec3 vecAO = -simplex.a.v;
  glm::vec3 vecAB = simplex.b.v - simplex.a.v;
  glm::vec3 vecAC = simplex.c.v - simplex.a.v;
  glm::vec3 vecABC = glm::cross(vecAB, vecAC);

  if (glm::dot(glm::cross(vecABC, vecAC), vecAO) > 0) {
    if (glm::dot(vecAC, vecAO) > 0) {
      simplex.set(simplex.a, simplex.c);
      searchDir = glm::cross(glm::cross(vecAC, vecAO), vecAC);
    } else {
      if (glm::dot(vecAB, vecAO) > 0) {
        simplex.set(simplex.a, simplex.b);
        searchDir = glm::cross(glm::cross(vecAB, vecAO), vecAB);
      } else {
        simplex.set(simplex.a);
        searchDir = vecAO;
      }
    }
  } else {
    if (glm::dot(glm::cross(vecAB, vecABC), vecAO) > 0) {
      if (glm::dot(vecAB, vecAO) > 0) {
        simplex.set(simplex.a, simplex.b);
        searchDir = glm::cross(glm::cross(vecAB, vecAO), vecAB);
      } else {
        simplex.set(simplex.a);
        searchDir = vecAO;
      }
    } else {
      if (glm::dot(vecABC, vecAO) > 0) {
        searchDir = vecABC;
      } else {
        simplex.set(simplex.a, simplex.c, simplex.b);
        searchDir = -vecABC;
      }
    }
  }
}

bool GJKEPA::GJKEPACollisionPointGenerator::doSimplex4() {
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

  switch (directionCode) {
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
          } else {
            simplex.set(simplex.a);
            searchDir = -simplex.a.v;
          }
        } else {
          if (glm::dot(glm::cross(vecACD, vecAD), vecAO) > 0) {
            simplex.set(simplex.a, simplex.d);
            searchDir = glm::cross(glm::cross(vecAD, vecAO), vecAD);
          } else {
            simplex.set(simplex.a, simplex.c, simplex.d);
            searchDir = vecACD;
          }
        }
      } else {
        if (glm::dot(glm::cross(vecAB, vecABC), vecAO) > 0) {
          if (glm::dot(vecAB, vecAO) > 0) {
            simplex.set(simplex.a, simplex.b);
            searchDir = glm::cross(glm::cross(vecAB, vecAO), vecAB);
          } else {
            simplex.set(simplex.a);
            searchDir = -simplex.a.v;
          }
        } else {
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
          if (glm::dot(vecAB, vecAO) > 0) {
            simplex.set(simplex.a, simplex.b);
            searchDir = glm::cross(glm::cross(vecAB, vecAO), vecAB);
          } else {
            simplex.set(simplex.a);
            searchDir = -simplex.a.v;
          }
        } else {
          if (glm::dot(glm::cross(vecABC, vecAC), vecAO) > 0) {
            simplex.set(simplex.a, simplex.c);
            searchDir = glm::cross(glm::cross(vecAC, vecAO), vecAC);
          } else {
            simplex.set(simplex.a, simplex.b, simplex.c);
            searchDir = vecABC;
          }
        }
      } else {
        if (glm::dot(glm::cross(vecAD, vecADB), vecAO) > 0) {
          if (glm::dot(vecAD, vecAO) > 0) {
            simplex.set(simplex.a, simplex.d);
            searchDir = glm::cross(glm::cross(vecAD, vecAO), vecAD);
          } else {
            simplex.set(simplex.a);
            searchDir = -simplex.a.v;
          }
        } else {
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
          } else {
            simplex.set(simplex.a);
            searchDir = -simplex.a.v;
          }
        } else {
          if (glm::dot(glm::cross(vecADB, vecAB), vecAO) > 0) {
            simplex.set(simplex.a, simplex.b);
            searchDir = glm::cross(glm::cross(vecAB, vecAO), vecAB);
          } else {
            simplex.set(simplex.a, simplex.d, simplex.b);
            searchDir = vecADB;
          }
        }
      } else {
        if (glm::dot(glm::cross(vecAC, vecACD), vecAO) > 0) {
          if (glm::dot(vecAC, vecAO) > 0) {
            simplex.set(simplex.a, simplex.c);
            searchDir = glm::cross(glm::cross(vecAC, vecAO), vecAC);
          } else {
            simplex.set(simplex.a);
            searchDir = -simplex.a.v;
          }
        } else {
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
      } else {
        if (glm::dot(vecAC, vecAO) > 0) {
          simplex.set(simplex.a, simplex.c);
          searchDir = glm::cross(glm::cross(vecAC, vecAO), vecAC);
        } else {
          if (glm::dot(vecAD, vecAO) > 0) {
            simplex.set(simplex.a, simplex.d);
            searchDir = glm::cross(glm::cross(vecAD, vecAO), vecAD);
          } else {
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

bool GJKEPA::GJKEPACollisionPointGenerator::doSimplex() {
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

bool GJKEPA::GJKEPACollisionPointGenerator::testIntersection() {
  unsigned int iterations = 0;

  simplex.clear();

  searchDir = glm::vec3(0, 1, 0);
  MinkowskiDiffPt initialPoint = support(searchDir);

  if (glm::abs(glm::dot(searchDir, initialPoint.v)) >=
      initialPoint.v.length() * 0.8f) {
    searchDir = glm::vec3(1, 0, 0);
    initialPoint = support(searchDir);
  }

  simplex.set(initialPoint);
  searchDir = -initialPoint.v;

  while (true) {
    if (iterations++ > PhysicsSettings::get().gjkepa.gjkMaxIters) {
      return false;
    }

    if (searchDir.length() <= PhysicsSettings::get().epsilons.gjkEpsilon)
      return false;

    MinkowskiDiffPt newPt = support(searchDir);
    /* new point is not past the origin */
    if (glm::dot(newPt.v, searchDir) <
        -PhysicsSettings::get().epsilons.gjkEpsilon) {
      DEBUG_PRINT("new point is not past the origin \n");
      return false;
    }

    simplex.pushVertex(newPt);

    if (doSimplex()) {
      DEBUG_PRINT("origin inside tetrahedron \n");
      return true;
    }
  }

  DEBUG_PRINT("GJK intersection test returned false \n");
  return false;
}

/* adds an edge to the list of edges or removes it, if it is already there, but
 * reversed */
static void addRemoveEdge(std::list<GJKEPA::Edge>& edges,
                          const GJKEPA::MinkowskiDiffPt& a,
                          const GJKEPA::MinkowskiDiffPt& b) {
  for (auto it = edges.begin(); it != edges.end(); it++) {
    if (it->a.v == b.v && it->b.v == a.v) {
      /* found reversed edge, just remove it */
      it = edges.erase(it);
      return;
    }
  }
  if (edges.size() < PhysicsSettings::get().gjkepa.epaMaxEdges)
    edges.push_back(GJKEPA::Edge(a, b));
}

/* Compute barycentric coordinates barCoords(u, v, w) for point p
with respect to triangle (a, b, c)

source: Christer Ericson - Real-Time Collision Detection, chap. 3.4 Barycentric
Coordinates*/
static void computeBarycentricCoords(const glm::vec3& a,
                                     const glm::vec3& b,
                                     const glm::vec3& c,
                                     const glm::vec3& p,
                                     glm::vec3& barCoords) {
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
bool GJKEPA::GJKEPACollisionPointGenerator::createCollisionPoint(
    CollisionPoint* contact) {
  unsigned int iterations = 0;

  std::list<Triangle> triangles;
  std::list<Edge> edges;

  float test1, test2, test3, test4;
  triangles.push_back(Triangle(simplex.a, simplex.b, simplex.c));
  test1 = glm::dot(triangles.back().a.v, triangles.back().vecABC);
  triangles.push_back(Triangle(simplex.a, simplex.d, simplex.b));
  test2 = glm::dot(triangles.back().a.v, triangles.back().vecABC);
  triangles.push_back(Triangle(simplex.a, simplex.c, simplex.d));
  test3 = glm::dot(triangles.back().a.v, triangles.back().vecABC);
  triangles.push_back(Triangle(simplex.b, simplex.d, simplex.c));
  test4 = glm::dot(triangles.back().a.v, triangles.back().vecABC);

  DEBUG_PRINT("Simplex = {\n\t");
  DEBUG_PRINT("A = " << simplex.a.v << "\n\t");
  DEBUG_PRINT("B = " << simplex.b.v << "\n\t");
  DEBUG_PRINT("C = " << simplex.c.v << "\n\t");
  DEBUG_PRINT("D = " << simplex.d.v << "\n}\n\n");

  while (iterations < PhysicsSettings::get().gjkepa.epaMaxIters) {
    if (triangles.size() == 0)
      return false;

    auto closestTriangleIt = triangles.begin();
    float minDistance = FLT_MAX;
    for (auto t = triangles.begin(); t != triangles.end(); t++) {
      /* distance to origin */
      float distance = glm::dot(t->vecABC, t->a.v);
      if (distance < minDistance) {
        minDistance = distance;
        closestTriangleIt = t;
      }
    }

    /* get furthest point in the triangle's normal direction, opposite to the
     * origin */
    MinkowskiDiffPt nextSup = support(closestTriangleIt->vecABC);
    /* distance from origin to the next point to be added to the expanding
     * polytope */
    float nextSupDist = glm::dot(nextSup.v, closestTriangleIt->vecABC);

    /* if the new point is not further from the origin than the face in whose
    direction we are expanding the polytope, we can't expand anymore */
    if (nextSupDist - minDistance <
        PhysicsSettings::get().gjkepa.epaGrowthThreshold) {
      glm::vec3 barCoords;

      /* compute barycentric coordinates of origin's projection on the triangle
      with respect to this closest triangle */
      computeBarycentricCoords(closestTriangleIt->a.v, closestTriangleIt->b.v,
                               closestTriangleIt->c.v,
                               closestTriangleIt->vecABC * minDistance,
                               barCoords);

      if (isnan(barCoords.x) || isnan(barCoords.y) || isnan(barCoords.z))
        return false;

      if (glm::abs(barCoords.x) > 1.0f || glm::abs(barCoords.y) > 1.0f ||
          glm::abs(barCoords.z) > 1.0f)
        return false;
      contact->points[0] = glm::vec3(barCoords.x * closestTriangleIt->a.supA +
                                     barCoords.y * closestTriangleIt->b.supA +
                                     barCoords.z * closestTriangleIt->c.supA);
      contact->points[1] = glm::vec3(barCoords.x * closestTriangleIt->a.supB +
                                     barCoords.y * closestTriangleIt->b.supB +
                                     barCoords.z * closestTriangleIt->c.supB);

      contact->normal = -closestTriangleIt->vecABC;

      contact->penetration = minDistance;

      contact->objects[0] = objA;
      contact->objects[1] = objB;

      return true;
    }

    if (glm::dot(closestTriangleIt->vecABC,
                 nextSup.v - closestTriangleIt->a.v) < 0) {
      return false;
    }

    for (auto it = triangles.begin(); it != triangles.end();) {
      if (glm::dot(it->vecABC, nextSup.v - it->a.v) >
          -PhysicsSettings::get().epsilons.epaEpsilon) {
        /* update the edge list in order to remove the triangles facing this
         * point */
        addRemoveEdge(edges, it->a, it->c);
        addRemoveEdge(edges, it->c, it->b);
        addRemoveEdge(edges, it->b, it->a);
        it = triangles.erase(it);
        continue;
      }
      it++;
    }
    /* re-create the triangles from the remaining edges */
    for (Edge e : edges) {
      glm::vec3 normal =
          glm::normalize(glm::cross(e.a.v - nextSup.v, e.b.v - nextSup.v));
      if (glm::isnan(glm::dot(nextSup.v, normal)) ||
          triangles.size() > PhysicsSettings::get().gjkepa.epaMaxTriangles)
        continue;
      if (glm::dot(normal, nextSup.v) > 0) {
        triangles.push_back(Triangle(nextSup, e.a, e.b));
        float test = glm::dot(nextSup.v, triangles.back().vecABC);
        int g = 0;
      } else {
        triangles.push_back(Triangle(nextSup, e.b, e.a));
        float test = glm::dot(nextSup.v, triangles.back().vecABC);
        int g = 0;
      }
    }

    edges.clear();
    iterations++;
  }

  DEBUG_PRINT("EPA max iterations reached\n");
  return false;
}

void CollisionsGenerator::fillCollisions() {
  if (PhysicsSettings::get().clearCollisionsFlag) {
    PhysicsSettings::get().clearCollisionsFlag = false;

    return;
  }
  for (auto& col : (*potentialCollisions)) {
    GJKEPA::GJKEPACollisionPointGenerator cg =
        GJKEPA::GJKEPACollisionPointGenerator(col->one, col->two);

    if (cg.testIntersection()) {
      CollisionPoint contactInfo;
      bool ok = cg.createCollisionPoint(&contactInfo);
      if (ok) {
        collisionsPoints.push_back(new CollisionPoint(contactInfo));
      }
    }
  }
}

void CollisionsGenerator::clearCollisions() {
  auto collisionIt = collisionsPoints.begin();
  while (collisionIt != collisionsPoints.end()) {
    delete *collisionIt;
    collisionIt++;
  }
  collisionsPoints.clear();
}
