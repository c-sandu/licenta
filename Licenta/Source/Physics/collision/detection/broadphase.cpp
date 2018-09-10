#include "broadphase.h"

#include <Physics/debug.h>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/transform.hpp>

#include <Physics/body/rigidbody.h>

void Collider::setRigidBody(RigidBody* body) {
  this->body = body;
}

Collider::~Collider() {}

OBBCollider::OBBCollider(const glm::vec3& halfSizes,
                         PhysicsObject* phyObject,
                         Mesh* mesh)
    : halfSizes(halfSizes) {
  this->offset = glm::vec3(0);
  this->mesh = mesh;
  this->phyObject = phyObject;
  this->markedForDeletion = false;
}

OBBCollider::OBBCollider(const glm::vec3& halfSizes,
                         const glm::vec3& offset,
                         PhysicsObject* phyObject,
                         Mesh* mesh)
    : halfSizes(halfSizes), offset(offset) {
  this->mesh = mesh;
  this->phyObject = phyObject;
  this->markedForDeletion = false;
}

void OBBCollider::updateInternals() {
  glm::vec3 _scale, _skew;
  glm::vec4 _perspective;

  orientation = glm::normalize(body->orientation);
  position = glm::vec3(body->transform * glm::vec4(offset, 1));
}

/* source: Christer Ericson - Real Time Collision Detection - 4.4 Oriented
 * Bounding Boxes(OBBs) */
bool OBBCollider::testIntersectionOBB(OBBCollider& other) {
  float ra, rb;
  glm::mat3 R, AbsR;

  glm::mat3 rotationMatA = glm::mat3_cast(this->orientation);
  glm::mat3 rotationMatB = glm::mat3_cast(other.orientation);

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      R[i][j] =
          glm::dot(glm::column(rotationMatA, i), glm::column(rotationMatB, j));

  glm::vec3 t = other.position - this->position;
  t = glm::vec3(glm::dot(t, glm::column(rotationMatA, 0)),
                glm::dot(t, glm::column(rotationMatA, 1)),
                glm::dot(t, glm::column(rotationMatA, 2)));

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      AbsR[i][j] =
          glm::abs(R[i][j]) + PhysicsSettings::get().epsilons.globalEpsilon;

  /* Test axes L = A0, L = A1, L = A2 */
  for (int i = 0; i < 3; i++) {
    ra = this->halfSizes[i];
    rb = other.halfSizes[0] * AbsR[i][0] + other.halfSizes[1] * AbsR[i][1] +
         other.halfSizes[2] * AbsR[i][2];
    if (glm::abs(t[i]) > ra + rb)
      return 0;
  }
  /* Test axes L = B0, L = B1, L = B2 */
  for (int i = 0; i < 3; i++) {
    ra = this->halfSizes[0] * AbsR[0][i] + this->halfSizes[1] * AbsR[1][i] +
         this->halfSizes[2] * AbsR[2][i];
    rb = other.halfSizes[i];
    if (glm::abs(t[0] * R[0][i] + t[1] * R[1][i] + t[2] * R[2][i]) > ra + rb)
      return 0;
  }

  /* Test axis L = A0 x B0 */
  ra = this->halfSizes[1] * AbsR[2][0] + this->halfSizes[2] * AbsR[1][0];
  rb = other.halfSizes[1] * AbsR[0][2] + other.halfSizes[2] * AbsR[0][1];
  if (glm::abs(t[2] * R[1][0] - t[1] * R[2][0]) > ra + rb)
    return 0;
  /* Test axis L = A0 x B1 */
  ra = this->halfSizes[1] * AbsR[2][1] + this->halfSizes[2] * AbsR[1][1];
  rb = other.halfSizes[0] * AbsR[0][2] + other.halfSizes[2] * AbsR[0][0];
  if (glm::abs(t[2] * R[1][1] - t[1] * R[2][1]) > ra + rb)
    return 0;
  /* Test axis L = A0 x B2 */
  ra = this->halfSizes[1] * AbsR[2][2] + this->halfSizes[2] * AbsR[1][2];
  rb = other.halfSizes[0] * AbsR[0][1] + other.halfSizes[1] * AbsR[0][0];
  if (glm::abs(t[2] * R[1][2] - t[1] * R[2][2]) > ra + rb)
    return 0;
  /* Test axis L = A1 x B0 */
  ra = this->halfSizes[0] * AbsR[2][0] + this->halfSizes[2] * AbsR[0][0];
  rb = other.halfSizes[1] * AbsR[1][2] + other.halfSizes[2] * AbsR[1][1];
  if (glm::abs(t[0] * R[2][0] - t[2] * R[0][0]) > ra + rb)
    return 0;
  /* Test axis L = A1 x B1 */
  ra = this->halfSizes[0] * AbsR[2][1] + this->halfSizes[2] * AbsR[0][1];
  rb = other.halfSizes[0] * AbsR[1][2] + other.halfSizes[2] * AbsR[1][0];
  if (glm::abs(t[0] * R[2][1] - t[2] * R[0][1]) > ra + rb)
    return 0;
  /* Test axis L = A1 x B2 */
  ra = this->halfSizes[0] * AbsR[2][2] + this->halfSizes[2] * AbsR[0][2];
  rb = other.halfSizes[0] * AbsR[1][1] + other.halfSizes[1] * AbsR[1][0];
  if (glm::abs(t[0] * R[2][2] - t[2] * R[0][2]) > ra + rb)
    return 0;
  /* Test axis L = A2 x B0 */
  ra = this->halfSizes[0] * AbsR[1][0] + this->halfSizes[1] * AbsR[0][0];
  rb = other.halfSizes[1] * AbsR[2][2] + other.halfSizes[2] * AbsR[2][1];
  if (glm::abs(t[1] * R[0][0] - t[0] * R[1][0]) > ra + rb)
    return 0;
  /* Test axis L = A2 x B1 */
  ra = this->halfSizes[0] * AbsR[1][1] + this->halfSizes[1] * AbsR[0][1];
  rb = other.halfSizes[0] * AbsR[2][2] + other.halfSizes[2] * AbsR[2][0];
  if (glm::abs(t[1] * R[0][1] - t[0] * R[1][1]) > ra + rb)
    return 0;
  /* Test axis L = A2 x B2 */
  ra = this->halfSizes[0] * AbsR[1][2] + this->halfSizes[1] * AbsR[0][2];
  rb = other.halfSizes[0] * AbsR[2][1] + other.halfSizes[1] * AbsR[2][0];
  if (glm::abs(t[1] * R[0][2] - t[0] * R[1][2]) > ra + rb)
    return 0;

  /* Since no separating axis is found, the OBBs must be intersecting */
  return 1;
}

bool OBBCollider::TestIntersectionRay(glm::vec3 origin, glm::vec3 direction) {
  float tMin = 0.0f;
  float tMax = 10.0f;

  glm::mat3 localAxes = glm::mat3_cast(orientation);

  glm::vec3 farPoint = origin + tMax * glm::normalize(direction);
  origin = glm::inverse(localAxes) * origin;
  farPoint = glm::inverse(localAxes) * farPoint;

  glm::vec3 midPoint = (origin + farPoint) * 0.5f;
  glm::vec3 halfLengthV = farPoint - midPoint;
  midPoint = midPoint - position; /* translate box and segment to origin */

  float adx = glm::abs(halfLengthV.x);
  if (glm::abs(midPoint.x) > halfSizes.x + adx)
    return false;
  float ady = glm::abs(halfLengthV.y);
  if (glm::abs(midPoint.y) > halfSizes.y + ady)
    return false;
  float adz = glm::abs(halfLengthV.z);
  if (glm::abs(midPoint.z) > halfSizes.z + adz)
    return false;

  adx += PhysicsSettings::get().epsilons.globalEpsilon;
  ady += PhysicsSettings::get().epsilons.globalEpsilon;
  adz += PhysicsSettings::get().epsilons.globalEpsilon;

  if (glm::abs(midPoint.y * halfLengthV.z - midPoint.z * halfLengthV.y) >
      halfSizes.y * adz + halfSizes.z * ady)
    return false;
  if (glm::abs(midPoint.z * halfLengthV.x - midPoint.x * halfLengthV.z) >
      halfSizes.x * adz + halfSizes.z * adx)
    return false;
  if (glm::abs(midPoint.x * halfLengthV.y - midPoint.y * halfLengthV.x) >
      halfSizes.x * ady + halfSizes.y * adx)
    return false;

  return true;
}

/* source:
 * https://github.com/gszauer/GamePhysicsCookbook/blob/master/Code/Geometry3D.cpp
 */
bool OBBCollider::TestIntersectionRay(glm::vec3 origin,
                                      glm::vec3 direction,
                                      float& intDistance,
                                      glm::vec3& intPoint) {
  direction = glm::normalize(direction);

  glm::mat3 o = glm::mat3_cast(orientation);

  glm::vec3 p = position - origin;

  glm::vec3 X(o[0]);
  glm::vec3 Y(o[1]);
  glm::vec3 Z(o[2]);

  glm::vec3 f = o * direction;
  glm::vec3 e = o * p;

  float t[6] = {0, 0, 0, 0, 0, 0};
  for (int i = 0; i < 3; ++i) {
    if (glm::abs(f[i]) < PhysicsSettings::get().epsilons.globalEpsilon) {
      if (-e[i] - halfSizes[i] > 0 || -e[i] + halfSizes[i] < 0) {
        return false;
      }
      f[i] = 0.00001f; /* Avoid div by 0! */
    }

    t[i * 2 + 0] = (e[i] + halfSizes[i]) / f[i]; /* tmin[x, y, z] */
    t[i * 2 + 1] = (e[i] - halfSizes[i]) / f[i]; /* tmax[x, y, z] */
  }

  float tmin =
      fmaxf(fmaxf(fminf(t[0], t[1]), fminf(t[2], t[3])), fminf(t[4], t[5]));
  float tmax =
      fminf(fminf(fmaxf(t[0], t[1]), fmaxf(t[2], t[3])), fmaxf(t[4], t[5]));

  /* if tmax < 0, ray is intersecting AABB
  but entire AABB is behing it's origin */
  if (tmax < 0) {
    return false;
  }

  /* if tmin > tmax, ray doesn't intersect AABB */
  if (tmin > tmax) {
    return false;
  }

  /* If tmin is < 0, tmax is closer */
  float t_result = tmin;

  if (tmin < 0.0f) {
    t_result = tmax;
  }

  intDistance = t_result;
  intPoint = origin + direction * t_result;
  return true;
}

bool OBBCollider::testIntersectionPlane(PlaneCollider& plane) {
  /* compute local axis */
  glm::vec3 thisAxis[3] = {glm::normalize(glm::vec3(1, 0, 0) * orientation),
                           glm::normalize(glm::vec3(0, 1, 0) * orientation),
                           glm::normalize(glm::vec3(0, 0, 1) * orientation)};

  float projectedRadius =
      halfSizes.x * glm::abs(glm::dot(plane.normal, thisAxis[0])) +
      halfSizes.y * glm::abs(glm::dot(plane.normal, thisAxis[1])) +
      halfSizes.z * glm::abs(glm::dot(plane.normal, thisAxis[2]));

  float distanceFromOrigin = glm::dot(plane.normal, position) - projectedRadius;

  return distanceFromOrigin <= plane.offset;
}

void PotentialCollisionDetector::addCollider(PlaneCollider* plane) {
  planeVector.push_back(plane);
}

void PotentialCollisionDetector::addCollider(OBBCollider* obb) {
  obbVector.push_back(obb);
}

PhysicsObject* PotentialCollisionDetector::performRayIntersection(
    glm::vec3 origin,
    glm::vec3 direction) {
  OBBCollider* closestOBB = nullptr;
  float minDistance = FLT_MAX;
  float distance;
  glm::vec3 point;
  for (auto obb : obbVector) {
    if (obb->TestIntersectionRay(origin, direction, distance, point)) {
      if (closestOBB == nullptr || distance < minDistance) {
        closestOBB = obb;
        minDistance = distance;
      }
    }
  }

  return closestOBB == nullptr ? nullptr : closestOBB->phyObject;
}

void PotentialCollisionDetector::clearPotentialCollisions() {
  for (PotentialCollision* p : potentialCollisions)
    delete p;

  potentialCollisions.clear();
}

void PotentialCollisionDetector::fillPotentialCollisions() {
  clearPotentialCollisions();

  /* delete dangling colliders */
  for (unsigned int i = 0; i < obbVector.size();) {
    if (obbVector[i]->markedForDeletion) {
      delete obbVector[i];
      obbVector.erase(obbVector.begin() + i);
      continue;
    }
    i++;
  }

  for (unsigned int i = 0; i < obbVector.size() - 1;) {
    /* look for OBB-OBB collisions */
    for (unsigned int j = i + 1; j < obbVector.size();) {
      if (obbVector[i]->testIntersectionOBB(*obbVector[j])) {
        potentialCollisions.push_back(new PotentialCollision(
            obbVector[i]->phyObject, obbVector[j]->phyObject));
      }
      j++;
    }
    i++;
  }
}

PlaneCollider::PlaneCollider(glm::vec3 normal,
                             float offset,
                             PhysicsObject* phyObject,
                             Mesh* mesh)
    : normal(normal), offset(offset) {
  this->mesh = mesh;
  this->phyObject = phyObject;
}

void PlaneCollider::updateInternals() {}

glm::mat4 PlaneCollider::getTransformMatrix() {
  return glm::translate(glm::mat4(1), normal * offset);
}

std::string PlaneCollider::toString() {
  return std::string("") + "\tPlaneCollider {" + "\n\t\t" +
         "normal = " + to_string(normal) + "\n\t\t" +
         "offsetFromOrigin = " + std::to_string(offset) + "\n\t}\n";
}

glm::mat4 OBBCollider::getTransformMatrix() {
  glm::mat4 transform = glm::translate(body->transform, offset);
  transform = glm::scale(transform, 1.0f / body->scale);
  transform = glm::scale(transform, halfSizes / 0.5f);
  return transform;
}

std::string OBBCollider::toString() {
  return std::string("") + "\tOBBCollider {" + "\n\t\t" +
         "halfSizes = " + to_string(halfSizes) + "\n\t\t" +
         "position = " + to_string(position) + "\n\t\t" +
         "orientation = " + to_string(orientation) + "\n\t}\n";
}
