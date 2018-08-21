#pragma once

#include<glm/glm.hpp>

class PhysicsSettings
{
private:

	PhysicsSettings() {};

public:
	static PhysicsSettings &get() {
		static PhysicsSettings settings;

		return settings;
	}

	struct {
		const float linear = 0.9f;
		const float angular = 0.9f;
	} damping;

	struct {
		float torque = 1.0f;
		float force = 1.0f;
		float impulse = 1.0f;
	} forceMultipliers;

	struct {
		struct {
			const glm::vec3 halfSizes = glm::vec3(0.5f, 0.5, 0.5f);
		} box;
		struct {
			const float radius = 0.5f;
			const glm::vec3 obbHalfSizes = glm::vec3(0.5f, 0.5f, 0.5f);
		} sphere;
		struct {
			const float height = 2.0f;
			const float radius = 0.5f;
			const glm::vec3 obbHalfSizes = glm::vec3(0.5f, 1.0f, 0.5f);
		} cylinder;
		struct {
			const float height = 1.0f;
			const float radius = 0.5f;
			const glm::vec3 obbHalfSizes = glm::vec3(0.5f, 1.0f, 0.5f);
		} capsule;
	} shapes;

	struct {
		const float globalEpsilon = 0.0001f;
		const float gjkEpsilon = 0.0001f;
		const float epaEpsilon = 0.001f;

		const float PEN_EPSILON = 0.01f;
		const float VEL_EPSILON = 0.01f;
	} epsilons;

	struct {
		const unsigned int gjkMaxIters = 100;
		const unsigned int epaMaxEdges = 50;
		const float epaGrowthThreshold = 0.001f;
		const unsigned int epaMaxIters = 50;
		const unsigned int epaMaxTriangles = 64;
	} gjkepa;

	struct {
		const unsigned int maxContactsPerManifold = 4;

		const float minVelocityForRestitution = 0.25f;

		const unsigned int PEN_MAX_ITERATIONS = 5;
		const unsigned int VEL_MAX_ITERATIONS = 5;

		const float angularMovementLimitFactor = 0.2f;

		const float oldContactDistanceThreshold = 0.001f;

		const float defaultRestitutionCoef = 0.1f;
		const float defaultFrictionCoef = 0.6f;

	} collisionResolution;

	struct {
		struct {
			glm::vec3 lightPosition = glm::vec3(0, 16, 0);
			glm::vec3 lightDirection = glm::vec3(0, -1, 0);
			const unsigned int materialShininess = 5;
			const float materialKd = 0.2f;
			const float materialKs = 0.2f;
		} lightning;

		const glm::vec3 defaultCameraPosition = glm::vec3(0, 2, 10);
	} sceneProperties;

	struct {
		const float defaultMass = 32.0f;
	} rigidBodies;

	struct {
		bool renderColliders = true;
		bool renderContacts = true;
		bool renderContactNormals = true;
		bool renderSpawner = true;
		bool renderSelection = true;
	} rendering;
	glm::vec3 gravity = glm::vec3(0, -20.0f, 0);
};