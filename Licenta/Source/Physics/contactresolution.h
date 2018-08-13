#pragma once

#include "contact.h"
#include <vector>
#include <list>


class ContactManifold
{
public:
	ContactManifold(PhysicsObject *obj1, PhysicsObject *obj2)
		: obj1(obj1), obj2(obj2)
	{
	}

	PhysicsObject *obj1;
	PhysicsObject *obj2;
	std::vector<Contact*> contacts;

	unsigned int timestamp;

};

class ImpulseContactResolver
{
public:
	unsigned int timestamp;

	std::list<Contact> contacts;
	std::list<ContactManifold> manifolds;

	ImpulseContactResolver() { timestamp = 0; }

	void solveContactManifold(ContactManifold & manifold);

	void solve(const std::vector<ContactInfo*> &collisions);

	void updateContacts(const std::vector<ContactInfo*> & collisions);
	static const uint8_t MAX_MANIFOLD_SIZE = 4;
	void addContactToManifold(ContactManifold & manifold, const ContactInfo & contact);
	uint8_t findWorstContact(ContactManifold & manifold);
};