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

	double timestamp;

};
const uint8_t MAX_MANIFOLD_SIZE = 4;
void addContactToManifold(ContactManifold & manifold, const Contact & contact);
uint8_t findWorstContact(ContactManifold & manifold);

class ImpulseContactResolver
{
public:
	std::vector<Contact*> contacts;
	std::list<std::list<Contact*>> contactManifolds;

	ImpulseContactResolver(const std::vector<Contact*> &contacts)
		: contacts(contacts) {}

	void buildContactManifolds();

	void solveContactManifold(const std::list<Contact*> &manifold);

	void solve();
};