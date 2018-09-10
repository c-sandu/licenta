#pragma once

#include <Physics/body/rigidbody.h>
#include <Physics/collision/contact.h>
#include <list>
#include <vector>

/* describes the contact manifold of two objects in contact */
class ContactManifold {
  friend class SequentialImpulseContactResolver;

 public:
  ContactManifold(PhysicsObject* obj1, PhysicsObject* obj2)
      : obj1(obj1), obj2(obj2) {}

  PhysicsObject* obj1; /* first object */
  PhysicsObject* obj2; /* second object */
  std::vector<Contact*>
      contacts; /* pointers to the contacts in this manifold */

  Contact* getDeepestContact(); /* returns the deepest penetrating contact */
  Contact*
  getFastestContact(); /* returns the fastest contact (with the lowest desired
                          delta velocity) */

 private:
  unsigned int timestamp; /* internal time this manifold was las updated */
};

/* a sequential impulse contact resolver */
class SequentialImpulseContactResolver {
 private:
  unsigned int timestamp; /* internal timestamp */
 public:
  std::list<Contact> contacts; /* all contacts maintained by this resolver */
  std::list<ContactManifold>
      manifolds; /* all manifolds maintained by this resolver */

 private:
  /* adds a new contact to a manifold */
  void addContactToManifold(ContactManifold& manifold,
                            const CollisionPoint& contact);
  /* returns the index of the worst contact in the manifold(the one that should
   * be removed) */
  uint8_t findWorstContact(ContactManifold& manifold);
  /* solve all contacts in a manifold */
  void solveContactManifold(ContactManifold& manifold, float deltaTime);

  /* computes desired delta velocity in the direction of the contact's normal */
  static float computeDesiredDeltaVelocity(Contact* contact);
  /* computes impulse for contact resolution */
  static glm::vec3 computeImpulse(Contact* contact);
  /* updates the positions of the given contact and all the other contacts in
   * its manifold */
  static void applyPositionUpdate(ContactManifold& manifold,
                                  Contact* deepestContact);
  /* updates the velocities of the given contact and all the other contacts in
   * its manifold */
  static void applyVelocityUpdate(ContactManifold& manifold,
                                  Contact* fastestContact);

  /* update the contacts and manifolds maintained by this resolver from the
   * previously generated collisions */
  void updateContacts(const std::vector<CollisionPoint*>& collisions);

 public:
  /* creates a new empty contact resolver */
  SequentialImpulseContactResolver() { timestamp = 0; }

  /* solve all contact manifolds maintained by this resolver */
  void solve(const std::vector<CollisionPoint*>& collisions,
             float deltaTime = 1.0f);

  void clearData();
};