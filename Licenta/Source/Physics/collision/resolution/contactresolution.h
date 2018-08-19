#pragma once

#include <Physics/collision/contact.h>
#include <Physics/body/rigidbody.h>
#include <vector>
#include <list>

/* describes the contact manifold of two objects in contact */
class ContactManifold
{
friend class SequentialImpulseContactResolver;
public:
	ContactManifold(PhysicsObject *obj1, PhysicsObject *obj2)
		: obj1(obj1), obj2(obj2)
	{
	}

	PhysicsObject *obj1; /* first object */
	PhysicsObject *obj2; /* second object */
	std::vector<Contact*> contacts; /* pointers to the contacts in this manifold */

private:
	unsigned int timestamp; /* internal time this manifold was las updated */
};

/* a sequential impulse contact resolver */
class SequentialImpulseContactResolver
{
private:
	unsigned int timestamp; /* internal timestamp */
	static const uint8_t MAX_MANIFOLD_SIZE = 4;
public:

	std::list<Contact> contacts; /* all contacts maintained by this resolver */
	std::list<ContactManifold> manifolds; /* all manifolds maintained by this resolver */

private:
	/* adds a new contact to a manifold */
	void addContactToManifold(ContactManifold & manifold, const CollisionPoint & contact);
	/* returns the index of the worst contact in the manifold(the one that should be removed) */
	uint8_t findWorstContact(ContactManifold & manifold);
	/* solve all contacts in a manifold */
	void solveContactManifold(ContactManifold & manifold);

	/* computes desired delta velocity for contact resolution */
	static float computeDesiredDeltaVelocity(Contact *contact);
	/* computes impulse for contact resolution */
	static glm::vec3 computeImpulse(Contact *contact);
	
	/* update the contacts and manifolds maintained by this resolver from the previously generated collisions */
	void updateContacts(const std::vector<CollisionPoint*> & collisions);

public:
	/* creates a new empty contact resolver */
	SequentialImpulseContactResolver() { timestamp = 0; }


	/* solve all contact manifolds maintained by this resolver */
	void solve(const std::vector<CollisionPoint*> &collisions);
};