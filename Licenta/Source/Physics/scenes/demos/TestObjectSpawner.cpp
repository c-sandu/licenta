#include "TestObjectSpawner.h"

#include <Core/Engine.h>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtc/random.hpp>

#include <Physics/debug.h>
#include <Physics/collision/detection/contactgeneration.h>
#include <Physics/collision/resolution/contactresolution.h>

TestObjectSpawner::TestObjectSpawner()
{
}

TestObjectSpawner::~TestObjectSpawner()
{
}

void TestObjectSpawner::Init()
{

	/* load meshes */
	{
		{
			Mesh* mesh = new Mesh("box");
			mesh->LoadMesh(RESOURCE_PATH::MODELS + "Primitives", "box.obj");
			meshes[mesh->GetMeshID()] = mesh;
		}

		{
			Mesh* mesh = new Mesh("sphere");
			mesh->LoadMesh(RESOURCE_PATH::MODELS + "Primitives", "sphere.obj");
			meshes[mesh->GetMeshID()] = mesh;
		}

		{
			Mesh* mesh = new Mesh("plane");
			mesh->LoadMesh(RESOURCE_PATH::MODELS + "Primitives", "plane50.obj");
			meshes[mesh->GetMeshID()] = mesh;
		}

		{
			Mesh* mesh = new Mesh("cylinder");
			mesh->LoadMesh(RESOURCE_PATH::MODELS + "Primitives", "cylinder.obj");
			meshes[mesh->GetMeshID()] = mesh;
		}

		{
			Mesh* mesh = new Mesh("capsule");
			mesh->LoadMesh(RESOURCE_PATH::MODELS + "Primitives", "capsule.obj");
			meshes[mesh->GetMeshID()] = mesh;
		}

		{
			Mesh* mesh = new Mesh("cone");
			mesh->LoadMesh(RESOURCE_PATH::MODELS + "Primitives", "cone.obj");
			meshes[mesh->GetMeshID()] = mesh;
		}
	}

	// Initialize shaders
	{
		Shader *shader = new Shader("phong");
		shader->AddShader("Source/Physics/shaders/VertexShader.glsl", GL_VERTEX_SHADER);
		shader->AddShader("Source/Physics/shaders/FragmentShader.glsl", GL_FRAGMENT_SHADER);
		shader->CreateAndLink();
		shaders[shader->GetName()] = shader;
	}

	//Light & material properties
	{
		lightPosition = PhysicsSettings::get().sceneProperties.lightning.lightPosition;
		lightDirection = PhysicsSettings::get().sceneProperties.lightning.lightDirection;
		materialShininess = PhysicsSettings::get().sceneProperties.lightning.materialShininess;
		materialKd = PhysicsSettings::get().sceneProperties.lightning.materialKd;
		materialKs = PhysicsSettings::get().sceneProperties.lightning.materialKs;
	}

	GetSceneCamera()->SetPosition(PhysicsSettings::get().sceneProperties.defaultCameraPosition);
	GetSceneCamera()->transform->SetWorldRotation(glm::vec3(0, 0, 0));

	polygonMode = GL_FILL;

	/* init object spawner */
	objSpawner = new ObjectSpawner(&meshes, &pcd);

	/* spawn floor */
	objSpawner->spawnWallStatic(glm::vec3(0, -0.5, 0), glm::vec3(1), glm::rotate(glm::quat(1, 0, 0, 0), RADIANS(90), glm::vec3(1, 0, 0)));
	objSpawner->selectedObject = objSpawner->objects.front();
	
	/* spawn back walls */
	objSpawner->spawnWallStatic(glm::vec3(-4.5, 4, 0), glm::vec3(1), glm::rotate(glm::quat(1, 0, 0, 0), RADIANS(90), glm::vec3(0, 1, 0)));
	objSpawner->spawnWallStatic(glm::vec3(0, 4, -4.5), glm::vec3(1), glm::rotate(glm::quat(1, 0, 0, 0), RADIANS(0), glm::vec3(0, 1, 0)));


	cg.potentialCollisions = &pcd.potentialCollisions;
}

void TestObjectSpawner::FrameStart()
{
	// clears the color buffer (using the previously set color) and depth buffer
	glClearColor(0, 0, 0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glm::ivec2 resolution = window->GetResolution();
	// sets the screen area where to draw
	glViewport(0, 0, resolution.x, resolution.y);
}

void TestObjectSpawner::Update(float deltaTime)
{
	glLineWidth(3);
	glPointSize(5);
	glPolygonMode(GL_FRONT_AND_BACK, polygonMode);

	for (auto & obj : objSpawner->objects) {
		if (obj == objSpawner->selectedObject && PhysicsSettings::get().rendering.renderSelection) {
			materialKd = 0.9f;
		}
		RenderSimpleMesh(obj->mesh, shaders["phong"], obj->getTransformMatrix(), obj->color);
		materialKd = PhysicsSettings::get().sceneProperties.lightning.materialKd;
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		if (PhysicsSettings::get().rendering.renderColliders) {
			RenderSimpleMesh(obj->collider->mesh, shaders["phong"], obj->getColTransformMatrix(), glm::vec3(1) - obj->color);
			glPolygonMode(GL_FRONT_AND_BACK, polygonMode);
		}
	}

	if (PhysicsSettings::get().rendering.renderContacts) {
		for (auto manIt : icr.manifolds) {
			for (auto cIt : manIt.contacts) {
				glm::mat4 t = glm::scale(glm::translate(glm::mat4(1), cIt->points[0]), glm::vec3(0.25f));
				RenderSimpleMesh(meshes["sphere"], shaders["phong"], t, glm::vec3(1, 0, 0));

				if (PhysicsSettings::get().rendering.renderContactNormals) {
					glm::mat4 t = glm::scale(glm::translate(glm::mat4(1), cIt->points[0]) * glm::rotate(glm::mat4(cIt->matContactToWorld), RADIANS(-90), glm::vec3(0, 0, 1)) * glm::translate(glm::mat4(1), glm::vec3(0, -0.5, 0)), glm::vec3(0.1f, 0.25f, 0.1f));
					RenderSimpleMesh(meshes["cone"], shaders["phong"], t, glm::vec3(0, 1, 0));
				}
			}
		}
	}

	if (PhysicsSettings::get().rendering.renderSpawner) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glm::mat4 t = glm::scale(glm::translate(glm::mat4(1), objSpawner->spawnPosition), glm::vec3(0.01f));
		RenderSimpleMesh(meshes["sphere"], shaders["phong"], t, glm::vec3(1));
		glPolygonMode(GL_FRONT_AND_BACK, polygonMode);
	}
	// Render the point light in the scene
	{
		glm::mat4 modelMatrix = glm::mat4(1);
		modelMatrix = glm::translate(modelMatrix, lightPosition);
		modelMatrix = glm::scale(modelMatrix, glm::vec3(0.1f));
		RenderMesh(meshes["sphere"], shaders["phong"], modelMatrix);
	}
	
	/* update spawner properties */
	objSpawner->updateFromCamera(GetSceneCamera()->transform->GetWorldPosition(), glm::normalize(glm::vec3(-glm::row(GetSceneCamera()->View, 2))));
	
	/* update scene objects */
	objSpawner->updateObjects(deltaTime);

	/* broad phase collision checking */
	pcd.clearPotentialCollisions();
	pcd.fillPotentialCollisions();

	/* near phase collision checking */
	cg.clearCollisions();
	cg.fillCollisions();

	/* solve collisions */
	icr.solve(cg.collisionsPoints);
}

void TestObjectSpawner::FrameEnd()
{
	DrawCoordinatSystem();
}


void TestObjectSpawner::RenderSimpleMesh(Mesh *mesh, Shader *shader, const glm::mat4 & modelMatrix, const glm::vec3 &color)
{
	if (!mesh || !shader || !shader->GetProgramID())
		return;

	// render an object using the specified shader and the specified position
	glUseProgram(shader->program);

	// Set shader uniforms for light & material properties
	// Set light position uniform
	int light_position = glGetUniformLocation(shader->program, "light_position");
	glUniform3f(light_position, lightPosition.x, lightPosition.y, lightPosition.z);

	int light_direction = glGetUniformLocation(shader->program, "light_direction");
	glUniform3f(light_direction, lightDirection.x, lightDirection.y, lightDirection.z);

	// Set eye position (camera position) uniform
	glm::vec3 eyePosition = GetSceneCamera()->transform->GetWorldPosition();
	int eye_position = glGetUniformLocation(shader->program, "eye_position");
	glUniform3f(eye_position, eyePosition.x, eyePosition.y, eyePosition.z);

	// Set material property uniforms (shininess, kd, ks, object color) 
	int material_shininess = glGetUniformLocation(shader->program, "material_shininess");
	glUniform1i(material_shininess, materialShininess);

	int material_kd = glGetUniformLocation(shader->program, "material_kd");
	glUniform1f(material_kd, materialKd);

	int material_ks = glGetUniformLocation(shader->program, "material_ks");
	glUniform1f(material_ks, materialKs);

	int object_color = glGetUniformLocation(shader->program, "object_color");
	glUniform3f(object_color, color.r, color.g, color.b);

	// Bind model matrix
	GLint loc_model_matrix = glGetUniformLocation(shader->program, "Model");
	glUniformMatrix4fv(loc_model_matrix, 1, GL_FALSE, glm::value_ptr(modelMatrix));

	// Bind view matrix
	glm::mat4 viewMatrix = GetSceneCamera()->GetViewMatrix();
	int loc_view_matrix = glGetUniformLocation(shader->program, "View");
	glUniformMatrix4fv(loc_view_matrix, 1, GL_FALSE, glm::value_ptr(viewMatrix));

	// Bind projection matrix
	glm::mat4 projectionMatrix = GetSceneCamera()->GetProjectionMatrix();
	int loc_projection_matrix = glGetUniformLocation(shader->program, "Projection");
	glUniformMatrix4fv(loc_projection_matrix, 1, GL_FALSE, glm::value_ptr(projectionMatrix));

	// Draw the object
	glBindVertexArray(mesh->GetBuffers()->VAO);
	glDrawElements(mesh->GetDrawMode(), static_cast<int>(mesh->indices.size()), GL_UNSIGNED_SHORT, 0);
}

void TestObjectSpawner::OnInputUpdate(float deltaTime, int mods)
{
	if (!window->MouseHold(GLFW_MOUSE_BUTTON_2)) {
		if (!objSpawner->selectedObject->body->isStatic) {
			if (window->KeyHold(GLFW_KEY_W)) {
				objSpawner->selectedObject->body->position += glm::vec3(0, 0, -1) * deltaTime;
				objSpawner->selectedObject->body->isAwake = true;
				objSpawner->selectedObject->body->resetMovement();
			}
			if (window->KeyHold(GLFW_KEY_S)) {
				objSpawner->selectedObject->body->position += glm::vec3(0, 0, +1) * deltaTime;
				objSpawner->selectedObject->body->isAwake = true;
				objSpawner->selectedObject->body->resetMovement();
			}
			if (window->KeyHold(GLFW_KEY_A)) {
				objSpawner->selectedObject->body->position += glm::vec3(-1, 0, 0) * deltaTime;
				objSpawner->selectedObject->body->isAwake = true;
				objSpawner->selectedObject->body->resetMovement();
			}
			if (window->KeyHold(GLFW_KEY_D)) {
				objSpawner->selectedObject->body->position += glm::vec3(+1, 0, 0) * deltaTime;
				objSpawner->selectedObject->body->isAwake = true;
				objSpawner->selectedObject->body->resetMovement();
			}
			if (window->KeyHold(GLFW_KEY_Q)) {
				objSpawner->selectedObject->body->position += glm::vec3(0, -1, 0) * deltaTime;
				objSpawner->selectedObject->body->isAwake = true;
				objSpawner->selectedObject->body->resetMovement();
			}
			if (window->KeyHold(GLFW_KEY_E)) {
				objSpawner->selectedObject->body->position += glm::vec3(0, +1, 0) * deltaTime;
				objSpawner->selectedObject->body->isAwake = true;
				objSpawner->selectedObject->body->resetMovement();
			}
			if (window->KeyHold(GLFW_KEY_SEMICOLON)) {
				objSpawner->selectedObject->body->orientation = glm::normalize(glm::rotate(objSpawner->selectedObject->body->orientation, 1.0f * deltaTime, glm::vec3(0, 1, 0)));
				objSpawner->selectedObject->body->isAwake = true;
				objSpawner->selectedObject->body->resetMovement();
			}
			if (window->KeyHold(GLFW_KEY_APOSTROPHE)) {
				objSpawner->selectedObject->body->orientation = glm::normalize(glm::rotate(objSpawner->selectedObject->body->orientation, -1.0f * deltaTime, glm::vec3(0, 1, 0)));
				objSpawner->selectedObject->body->isAwake = true;
				objSpawner->selectedObject->body->resetMovement();
			}
			if (window->KeyHold(GLFW_KEY_LEFT_BRACKET)) {
				objSpawner->selectedObject->body->orientation = glm::normalize(glm::rotate(objSpawner->selectedObject->body->orientation, 1.0f * deltaTime, glm::vec3(0, 0, 1)));
				objSpawner->selectedObject->body->isAwake = true;
				objSpawner->selectedObject->body->resetMovement();
			}
			if (window->KeyHold(GLFW_KEY_RIGHT_BRACKET)) {
				objSpawner->selectedObject->body->orientation = glm::normalize(glm::rotate(objSpawner->selectedObject->body->orientation, -1.0f * deltaTime, glm::vec3(0, 0, 1)));
				objSpawner->selectedObject->body->isAwake = true;
				objSpawner->selectedObject->body->resetMovement();
			}
		}
	}
}

void TestObjectSpawner::OnKeyPress(int key, int mods)
{
	// add key press event
	if (key == GLFW_KEY_P)
	{
		switch (polygonMode)
		{
		case GL_POINT:
			polygonMode = GL_FILL;
			break;
		case GL_LINE:
			polygonMode = GL_POINT;
			break;
		default:
			polygonMode = GL_LINE;
			break;
		}
	}
	else if (key == GLFW_KEY_V)
	{
		PRINT_APP("selectedObject = " + objSpawner->selectedObject->toString());
	}
	else if (key == GLFW_KEY_TAB)
	{
		PRINT_APP(objSpawner->toString());
	}
	else if (key == GLFW_KEY_SPACE)
	{
		objSpawner->mustSpawnObject = true;
	}
	else if (key == GLFW_KEY_M)
	{
		objSpawner->applyStartingImpulse = !objSpawner->applyStartingImpulse;
	}
	else if (key == GLFW_KEY_R)
	{
		objSpawner->randomizeNextObject = !objSpawner->randomizeNextObject;
	}
	else if (key == GLFW_KEY_T)
	{
		objSpawner->randomizeProperties = !objSpawner->randomizeProperties;
	}
	else if (key == GLFW_KEY_LEFT_SHIFT)
	{
		PhysicsObject *newSelectedObj = pcd.performRayIntersection(GetSceneCamera()->transform->GetWorldPosition(), glm::normalize(-glm::vec3(glm::row(GetSceneCamera()->View, 2))));
		if (newSelectedObj != nullptr)
			objSpawner->selectedObject = newSelectedObj;
	}
}

void TestObjectSpawner::OnKeyRelease(int key, int mods)
{
}

void TestObjectSpawner::OnMouseMove(int mouseX, int mouseY, int deltaX, int deltaY)
{
}

void TestObjectSpawner::OnMouseBtnPress(int mouseX, int mouseY, int button, int mods)
{
}

void TestObjectSpawner::OnMouseBtnRelease(int mouseX, int mouseY, int button, int mods)
{
}

void TestObjectSpawner::OnMouseScroll(int mouseX, int mouseY, int offsetX, int offsetY)
{
}

void TestObjectSpawner::OnWindowResize(int width, int height)
{
}
