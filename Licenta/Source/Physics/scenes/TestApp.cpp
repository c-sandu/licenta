#include "TestApp.h"

#include <Core/Engine.h>
#include <iostream>



TestApp::TestApp()
{
}

TestApp::~TestApp()
{
}

void TestApp::Init()
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
	}

	// Create a shader program for drawing face polygon with the color of the normal
	{
		Shader *shader = new Shader("phong");
		shader->AddShader("Source/Physics/shaders/VertexShader.glsl", GL_VERTEX_SHADER);
		shader->AddShader("Source/Physics/shaders/FragmentShader.glsl", GL_FRAGMENT_SHADER);
		shader->CreateAndLink();
		shaders[shader->GetName()] = shader;
	}

	//Light & material properties
	{
		lightPosition = glm::vec3(0, 16, 0);
		lightDirection = glm::vec3(0, -1, 0);
		materialShininess = 30;
		materialKd = 0.5;
		materialKs = 0.5;
	}

	GetSceneCamera()->SetPosition(glm::vec3(0, 2, 10));

	polygonMode = GL_FILL;

	{
		boxDefault = new RigidBody();
		boxDefault->setMass(32.0f);
		boxDefault->updateTransformMatrix();
		boxDefault->setInertiaTensor(RigidBody::inertiaTensorCube(glm::vec3(0.5) * boxDefault->scale));
		boxDefault->applyForce(glm::vec3(0, 8, 0));
		boxDefault->applyForceAtLocalPoint(glm::vec3(8, 0, 4), glm::vec3(-0.5, 0.5, -0.5));
		PhysicsObject *obj = new PhysicsObject();
		obj->body = boxDefault;
		obj->mesh = meshes["box"];
		obj->color = glm::vec3(1, 1, 0);
		obj->collider = new OBBCollider(glm::vec3(0.5) * obj->body->scale, obj);
		obj->collider->setRigidBody(obj->body);
		obj->collider->updateInternals();
		obj->name.assign("boxDefault");
		objects.push_back(obj);
		pcd.addCollider((OBBCollider*)obj->collider);
	}
	{
		boxSmall = new RigidBody(glm::vec3(-8, 0, 0), glm::vec3(0.25f));
		boxSmall->setMass(32.0f);
		boxSmall->updateTransformMatrix();
		boxSmall->setInertiaTensor(RigidBody::inertiaTensorCube(glm::vec3(0.5) * boxSmall->scale));
		boxSmall->applyForce(glm::vec3(0, 8, 0));
		boxSmall->applyForceAtLocalPoint(glm::vec3(8, 0, 4), glm::vec3(-0.5, 0.5, -0.5));
		PhysicsObject *obj = new PhysicsObject();
		obj->body = boxSmall;
		obj->mesh = meshes["box"];
		obj->color = glm::vec3(1, 0, 0);
		obj->collider = new OBBCollider(glm::vec3(0.5) * obj->body->scale, obj);
		obj->collider->setRigidBody(obj->body);
		obj->collider->updateInternals();
		obj->name.assign("boxSmall");
		objects.push_back(obj);
		pcd.addCollider((OBBCollider*)obj->collider);
	}
	{
		boxLarge = new RigidBody(glm::vec3(-4, 0, 0), glm::vec3(2.0f));
		boxLarge->setMass(32.0f);
		boxLarge->updateTransformMatrix();
		boxLarge->setInertiaTensor(RigidBody::inertiaTensorCube(glm::vec3(0.5) * boxDefault->scale));
		boxLarge->applyForce(glm::vec3(0, 8, 0));
		boxLarge->applyForceAtLocalPoint(glm::vec3(8, 0, 4), glm::vec3(-0.5, 0.5, -0.5));
		PhysicsObject *obj = new PhysicsObject();
		obj->body = boxLarge;
		obj->mesh = meshes["box"];
		obj->color = glm::vec3(0, 1, 0);
		obj->collider = new OBBCollider(glm::vec3(0.5) * obj->body->scale, obj);
		obj->collider->setRigidBody(obj->body);
		obj->collider->updateInternals();
		obj->name.assign("boxLarge");
		objects.push_back(obj);
		pcd.addCollider((OBBCollider*)obj->collider);
	}
	{
		boxHeavy = new RigidBody(glm::vec3(4, 0, 0));
		boxHeavy->setMass(128.0f);
		boxHeavy->updateTransformMatrix();
		boxHeavy->setInertiaTensor(RigidBody::inertiaTensorCube());
		boxHeavy->applyForce(glm::vec3(0, 8, 0));
		boxHeavy->applyForceAtLocalPoint(glm::vec3(8, 0, 4), glm::vec3(-0.5, 0.5, -0.5));
		PhysicsObject *obj = new PhysicsObject();
		obj->body = boxHeavy;
		obj->mesh = meshes["box"];
		obj->color = glm::vec3(0, 0, 1);
		obj->collider = new OBBCollider(glm::vec3(0.5) * obj->body->scale, obj);
		obj->collider->setRigidBody(obj->body);
		obj->collider->updateInternals();
		obj->name.assign("boxHeavy");
		objects.push_back(obj);
		pcd.addCollider((OBBCollider*)obj->collider);
	}
	{
		boxSmallHeavy = new RigidBody(glm::vec3(8, 0, 0), glm::vec3(0.25f));
		boxSmallHeavy->setMass(128.0f);
		boxSmallHeavy->updateTransformMatrix();
		boxSmallHeavy->setInertiaTensor(RigidBody::inertiaTensorCube(glm::vec3(0.5) * boxDefault->scale));
		boxSmallHeavy->applyForce(glm::vec3(0, 8, 0));
		boxSmallHeavy->applyForceAtLocalPoint(glm::vec3(8, 0, 4), glm::vec3(-0.5, 0.5, -0.5));
		PhysicsObject *obj = new PhysicsObject();
		obj->body = boxSmallHeavy;
		obj->mesh = meshes["box"];
		obj->color = glm::vec3(0, 1, 1);
		obj->collider = new OBBCollider(glm::vec3(0.5) * obj->body->scale, obj);
		obj->collider->setRigidBody(obj->body);
		obj->collider->updateInternals();
		obj->name.assign("boxSmallHeavy");
		objects.push_back(obj);
		pcd.addCollider((OBBCollider*)obj->collider);
	}

	{
		boxTall = new RigidBody(glm::vec3(0, 0, -3), glm::vec3(1, 2, 1));
		boxTall->setMass(32.0f);
		boxTall->updateTransformMatrix();
		boxTall->setInertiaTensor(RigidBody::inertiaTensorCube(glm::vec3(0.5) * boxTall->scale));
		boxTall->applyForce(glm::vec3(0, 8, 0));
		boxTall->applyForceAtLocalPoint(glm::vec3(8, 0, 4), glm::vec3(-0.5, 0.5, -0.5));
		PhysicsObject *obj = new PhysicsObject();
		obj->body = boxTall;
		obj->mesh = meshes["box"];
		obj->color = glm::vec3(1, 0, 1);
		obj->collider = new OBBCollider(glm::vec3(0.5) * obj->body->scale, obj);
		obj->collider->setRigidBody(obj->body);
		obj->collider->updateInternals();
		obj->name.assign("boxTall");
		objects.push_back(obj);
		pcd.addCollider((OBBCollider*)obj->collider);
	}

	{
		PhysicsObject *obj = new PhysicsObject();
		obj->body = NULL;
		obj->mesh = meshes["plane"];
		obj->color = glm::vec3(1, 1, 1);
		obj->collider = new PlaneCollider(glm::vec3(0, 1, 0), -0.1f, obj);
		obj->collider->setRigidBody(obj->body);
		obj->name.assign("planeXoZ");
		objects.push_back(obj);
		pcd.addCollider((PlaneCollider*)obj->collider);
	}

	selectedObjIndex = 0;
}

void TestApp::FrameStart()
{
	// clears the color buffer (using the previously set color) and depth buffer
	glClearColor(0, 0, 0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glm::ivec2 resolution = window->GetResolution();
	// sets the screen area where to draw
	glViewport(0, 0, resolution.x, resolution.y);
}

void TestApp::Update(float deltaTime)
{
	glLineWidth(3);
	glPointSize(5);
	glPolygonMode(GL_FRONT_AND_BACK, polygonMode);
	/*
	RenderSimpleMesh(meshes["box"], shaders["phong"], boxDefault->transform, glm::vec3(1, 1, 0));
	RenderSimpleMesh(meshes["box"], shaders["phong"], boxSmall->transform, glm::vec3(1, 0, 1));
	RenderSimpleMesh(meshes["box"], shaders["phong"], boxLarge->transform, glm::vec3(0, 1, 1));
	RenderSimpleMesh(meshes["box"], shaders["phong"], boxHeavy->transform, glm::vec3(1, 0, 0));
	RenderSimpleMesh(meshes["box"], shaders["phong"], boxSmallHeavy->transform, glm::vec3(0, 1, 0));

	// Render ground
	{
		glm::mat4 modelMatrix = glm::mat4(1);
		modelMatrix = glm::translate(modelMatrix, glm::vec3(0, -0.1, 0));
		RenderSimpleMesh(meshes["plane"], shaders["phong"], modelMatrix);
	}
	*/

	for (auto & obj : objects) {
		RenderSimpleMesh(obj->mesh, shaders["phong"], obj->getTransformMatrix(), obj->color);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		RenderSimpleMesh(meshes[obj->collider->meshName], shaders["phong"], obj->getColTransformMatrix(), glm::vec3(1) - obj->color);
		glPolygonMode(GL_FRONT_AND_BACK, polygonMode);
	}
	// Render the point light in the scene
	{
		glm::mat4 modelMatrix = glm::mat4(1);
		modelMatrix = glm::translate(modelMatrix, lightPosition);
		modelMatrix = glm::scale(modelMatrix, glm::vec3(0.1f));
		RenderMesh(meshes["sphere"], shaders["phong"], modelMatrix);
	}
	/*
	boxDefault->integrate(deltaTime);
	boxSmall->integrate(deltaTime);
	boxLarge->integrate(deltaTime);
	boxHeavy->integrate(deltaTime);
	boxSmallHeavy->integrate(deltaTime);
	*/
	for (auto & obj : objects)
		obj->update(deltaTime);
}

void TestApp::FrameEnd()
{
	DrawCoordinatSystem();
}


void TestApp::RenderSimpleMesh(Mesh *mesh, Shader *shader, const glm::mat4 & modelMatrix, const glm::vec3 &color)
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

void TestApp::OnInputUpdate(float deltaTime, int mods)
{
	if (!window->MouseHold(GLFW_MOUSE_BUTTON_2)) {
		if (window->KeyHold(GLFW_KEY_W))
			objects[selectedObjIndex]->body->position += glm::vec3(0, 0, -1) * deltaTime;
		if (window->KeyHold(GLFW_KEY_S))
			objects[selectedObjIndex]->body->position += glm::vec3(0, 0, +1) * deltaTime;
		if (window->KeyHold(GLFW_KEY_A))
			objects[selectedObjIndex]->body->position += glm::vec3(-1, 0, 0) * deltaTime;
		if (window->KeyHold(GLFW_KEY_D))
			objects[selectedObjIndex]->body->position += glm::vec3(+1, 0, 0) * deltaTime;
		if (window->KeyHold(GLFW_KEY_Q))
			objects[selectedObjIndex]->body->position += glm::vec3(0, -1, 0) * deltaTime;
		if (window->KeyHold(GLFW_KEY_E))
			objects[selectedObjIndex]->body->position += glm::vec3(0, +1, 0) * deltaTime;
	}
}

void TestApp::OnKeyPress(int key, int mods)
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
	else if (key == GLFW_KEY_Z)
	{
		boxDefault->applyForceAtWorldPoint(glm::vec3(12, 0, 8), glm::vec3(-0.5, 0.5, -0.5));
	}
	else if (key == GLFW_KEY_T)
	{
		pcd.fillPotentialCollisions();
		for (auto & pc : pcd.potentialCollisions)
			std::cout << "potential collision between " << pc->one->name << " and " << pc->two->name << "\n\n";
	}
	else if (key == GLFW_KEY_V)
	{
		for (auto & obj : objects)
			std::cout << obj->toString();
	}
	else if (key >= GLFW_KEY_0 && key < GLFW_KEY_7)
		std::cout << objects[key - GLFW_KEY_0]->toString();
	else if (key == GLFW_KEY_TAB) {
		selectedObjIndex = (selectedObjIndex + 1) % 6;
		std::cout << "selected object = " << objects[selectedObjIndex]->name << "\n";
	}
}

void TestApp::OnKeyRelease(int key, int mods)
{
}

void TestApp::OnMouseMove(int mouseX, int mouseY, int deltaX, int deltaY)
{
}

void TestApp::OnMouseBtnPress(int mouseX, int mouseY, int button, int mods)
{
}

void TestApp::OnMouseBtnRelease(int mouseX, int mouseY, int button, int mods)
{
}

void TestApp::OnMouseScroll(int mouseX, int mouseY, int offsetX, int offsetY)
{
}

void TestApp::OnWindowResize(int width, int height)
{
}
