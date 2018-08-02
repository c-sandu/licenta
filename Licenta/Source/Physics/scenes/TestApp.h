#pragma once

#include <Component/SimpleScene.h>
#include <Component/Transform/Transform.h>
#include <Core/GPU/Mesh.h>

#include <vector>

#include "../rigidbody.h"
#include "../object.h"

class TestApp : public SimpleScene
{
public:
	TestApp();
	~TestApp();

	void Init() override;

private:
	void FrameStart() override;
	void Update(float deltaTimeSeconds) override;
	void FrameEnd() override;

	void RenderSimpleMesh(Mesh *mesh, Shader *shader, const glm::mat4 &modelMatrix, const glm::vec3 &color = glm::vec3(1));

	void OnInputUpdate(float deltaTime, int mods) override;
	void OnKeyPress(int key, int mods) override;
	void OnKeyRelease(int key, int mods) override;
	void OnMouseMove(int mouseX, int mouseY, int deltaX, int deltaY) override;
	void OnMouseBtnPress(int mouseX, int mouseY, int button, int mods) override;
	void OnMouseBtnRelease(int mouseX, int mouseY, int button, int mods) override;
	void OnMouseScroll(int mouseX, int mouseY, int offsetX, int offsetY) override;
	void OnWindowResize(int width, int height) override;

	glm::vec3 lightPosition;
	glm::vec3 lightDirection;
	unsigned int materialShininess;
	float materialKd;
	float materialKs;

	GLenum polygonMode;

	RigidBody *boxDefault, *boxSmall, *boxLarge, *boxHeavy, *boxSmallHeavy, *boxTall;
	std::vector <PhysicsObject*> objects;
	PotentialCollisionDetector pcd;

	unsigned int selectedObjIndex;
};