#pragma once

#include "contact.h"
#include <vector>
#include <list>

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