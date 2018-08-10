#include "TestCollisionResolution.h"

#include <Core/Engine.h>
#include <glm/gtx/string_cast.hpp>

#include "../debug.h"
#include "../contactgeneration.h"
#include "../contactresolution.h"

TestCollisionResolution::TestCollisionResolution()
{
}

TestCollisionResolution::~TestCollisionResolution()
{
}

void TestCollisionResolution::Init()
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
		boxAbove = new RigidBody(glm::vec3(0, 4, 0), glm::vec3(1));
		boxAbove->setMass(32.0f);
		boxAbove->updateTransformMatrix();
		boxAbove->setInertiaTensor(RigidBody::inertiaTensorCube(glm::vec3(0.5) * boxAbove->scale));
		boxAbove->orientation = glm::rotate(boxAbove->orientation, (float)M_PI * 0.5f, glm::vec3(1, 0, 1));
		/*boxAbove->applyForce(glm::vec3(0, 8, 0));
		boxAbove->applyForceAtLocalPoint(glm::vec3(8, 0, 4), glm::vec3(-0.5, 0.5, -0.5));*/
		PhysicsObject *obj = new PhysicsObject();
		obj->body = boxAbove;
		obj->mesh = meshes["box"];
		obj->color = glm::vec3(1, 1, 0);
		obj->collider = new OBBCollider(glm::vec3(0.5) * obj->body->scale, obj);
		obj->collider->setRigidBody(obj->body);
		obj->collider->updateInternals();
		obj->name.assign("boxAbove");
		obj->shape = new Box(0.5f * obj->body->scale.x, 0.5f * obj->body->scale.y, 0.5f * obj->body->scale.z);
		objects.push_back(obj);
		pcd.addCollider((OBBCollider*)obj->collider);
	}


	//{
	//	boxSmall = new RigidBody(glm::vec3(-8, 0, 0), glm::vec3(0.25f));
	//	boxSmall->setMass(32.0f);
	//	boxSmall->updateTransformMatrix();
	//	boxSmall->setInertiaTensor(RigidBody::inertiaTensorCube(glm::vec3(0.5) * boxSmall->scale));
	//	boxSmall->applyForce(glm::vec3(0, 8, 0));
	//	boxSmall->applyForceAtLocalPoint(glm::vec3(8, 0, 4), glm::vec3(-0.5, 0.5, -0.5));
	//	PhysicsObject *obj = new PhysicsObject();
	//	obj->body = boxSmall;
	//	obj->mesh = meshes["box"];
	//	obj->color = glm::vec3(1, 0, 0);
	//	obj->collider = new OBBCollider(glm::vec3(0.5) * obj->body->scale, obj);
	//	obj->collider->setRigidBody(obj->body);
	//	obj->collider->updateInternals();
	//	obj->name.assign("boxSmall");
	//	objects.push_back(obj);
	//	pcd.addCollider((OBBCollider*)obj->collider);
	//}
	//{
	//	boxLarge = new RigidBody(glm::vec3(-4, 0, 0), glm::vec3(2.0f));
	//	boxLarge->setMass(32.0f);
	//	boxLarge->updateTransformMatrix();
	//	boxLarge->setInertiaTensor(RigidBody::inertiaTensorCube(glm::vec3(0.5) * boxDefault->scale));
	//	boxLarge->applyForce(glm::vec3(0, 8, 0));
	//	boxLarge->applyForceAtLocalPoint(glm::vec3(8, 0, 4), glm::vec3(-0.5, 0.5, -0.5));
	//	PhysicsObject *obj = new PhysicsObject();
	//	obj->body = boxLarge;
	//	obj->mesh = meshes["box"];
	//	obj->color = glm::vec3(0, 1, 0);
	//	obj->collider = new OBBCollider(glm::vec3(0.5) * obj->body->scale, obj);
	//	obj->collider->setRigidBody(obj->body);
	//	obj->collider->updateInternals();
	//	obj->name.assign("boxLarge");
	//	objects.push_back(obj);
	//	pcd.addCollider((OBBCollider*)obj->collider);
	//}
	//{
	//	boxHeavy = new RigidBody(glm::vec3(4, 0, 0));
	//	boxHeavy->setMass(128.0f);
	//	boxHeavy->updateTransformMatrix();
	//	boxHeavy->setInertiaTensor(RigidBody::inertiaTensorCube());
	//	boxHeavy->applyForce(glm::vec3(0, 8, 0));
	//	boxHeavy->applyForceAtLocalPoint(glm::vec3(8, 0, 4), glm::vec3(-0.5, 0.5, -0.5));
	//	PhysicsObject *obj = new PhysicsObject();
	//	obj->body = boxHeavy;
	//	obj->mesh = meshes["box"];
	//	obj->color = glm::vec3(0, 0, 1);
	//	obj->collider = new OBBCollider(glm::vec3(0.5) * obj->body->scale, obj);
	//	obj->collider->setRigidBody(obj->body);
	//	obj->collider->updateInternals();
	//	obj->name.assign("boxHeavy");
	//	objects.push_back(obj);
	//	pcd.addCollider((OBBCollider*)obj->collider);
	//}
	//{
	//	boxSmallHeavy = new RigidBody(glm::vec3(8, 0, 0), glm::vec3(0.25f));
	//	boxSmallHeavy->setMass(128.0f);
	//	boxSmallHeavy->updateTransformMatrix();
	//	boxSmallHeavy->setInertiaTensor(RigidBody::inertiaTensorCube(glm::vec3(0.5) * boxDefault->scale));
	//	boxSmallHeavy->applyForce(glm::vec3(0, 8, 0));
	//	boxSmallHeavy->applyForceAtLocalPoint(glm::vec3(8, 0, 4), glm::vec3(-0.5, 0.5, -0.5));
	//	PhysicsObject *obj = new PhysicsObject();
	//	obj->body = boxSmallHeavy;
	//	obj->mesh = meshes["box"];
	//	obj->color = glm::vec3(0, 1, 1);
	//	obj->collider = new OBBCollider(glm::vec3(0.5) * obj->body->scale, obj);
	//	obj->collider->setRigidBody(obj->body);
	//	obj->collider->updateInternals();
	//	obj->name.assign("boxSmallHeavy");
	//	objects.push_back(obj);
	//	pcd.addCollider((OBBCollider*)obj->collider);
	//}

	{
		boxBelow = new RigidBody(glm::vec3(0, 1, 1), glm::vec3(2, 1, 2));
		boxBelow->setMass(0.0f, true);
		boxBelow->updateTransformMatrix();
		boxBelow->setInertiaTensor(RigidBody::inertiaTensorCube(glm::vec3(0.5) * boxBelow->scale));
		//boxBelow->orientation = glm::rotate(boxBelow->orientation, 0.1f, glm::vec3(0, 1, 0));
		/*boxBelow->applyForce(glm::vec3(0, 8, 0));
		boxBelow->applyForceAtLocalPoint(glm::vec3(8, 0, 4), glm::vec3(-0.5, 0.5, -0.5));*/
		PhysicsObject *obj = new PhysicsObject();
		obj->body = boxBelow;
		obj->mesh = meshes["box"];
		obj->color = glm::vec3(1, 0, 1);
		obj->collider = new OBBCollider(glm::vec3(0.5) * obj->body->scale, obj);
		obj->collider->setRigidBody(obj->body);
		obj->collider->updateInternals();
		obj->name.assign("boxBelow");
		obj->shape = new Box(0.5f * obj->body->scale.x, 0.5f * obj->body->scale.y, 0.5f * obj->body->scale.z);
		objects.push_back(obj);
		pcd.addCollider((OBBCollider*)obj->collider);
	}

	{
		boxExtra = new RigidBody(glm::vec3(3, 1, -2));
		boxExtra->setMass(32.0f);
		boxExtra->updateTransformMatrix();
		boxExtra->setInertiaTensor(RigidBody::inertiaTensorCube(glm::vec3(0.5f) * boxExtra->scale));
		PhysicsObject *obj = new PhysicsObject();
		obj->body = boxExtra;
		obj->mesh = meshes["box"];
		obj->color = glm::vec3(0, 1, 1);
		obj->collider = new OBBCollider(glm::vec3(0.5) * obj->body->scale, obj);
		obj->collider->setRigidBody(obj->body);
		obj->collider->updateInternals();
		obj->name.assign("boxExtra");
		obj->shape = new Box(0.5f * obj->body->scale.x, 0.5f * obj->body->scale.y, 0.5f * obj->body->scale.z);
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
		obj->shape = new Plane(25.0f, 25.0f);
		objects.push_back(obj);
		pcd.addCollider((PlaneCollider*)obj->collider);
	}

	selectedObjIndex = 0;

	PRINT_APP("Positions = {\n\t");
	for (glm::vec3 &p : meshes["box"]->positions)
		PRINT_APP(glm::to_string(p) << "\n\t");
	PRINT_APP("}\n");

	PRINT_APP("Vertices = {\n\t");
	for (VertexFormat &p : meshes["box"]->vertices)
		PRINT_APP("pos = " << glm::to_string(p.position) << ", normal = " << glm::to_string(p.normal) << "\n\t");
	PRINT_APP("}\n");

	PRINT_APP("Indices = {\n\t");
	for (unsigned short &p : meshes["box"]->indices)
		PRINT_APP(p << ", ");
	PRINT_APP("\n}\n");
}

void TestCollisionResolution::FrameStart()
{
	// clears the color buffer (using the previously set color) and depth buffer
	glClearColor(0, 0, 0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glm::ivec2 resolution = window->GetResolution();
	// sets the screen area where to draw
	glViewport(0, 0, resolution.x, resolution.y);
}

void TestCollisionResolution::Update(float deltaTime)
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
	for (auto & obj : objects) {
		///* apply gravity */
		//if (obj->body != NULL)
		//	obj->body->applyForce(glm::vec3(0, -10.0f, 0));
		obj->update(deltaTime);
	}

	pcd.fillPotentialCollisions();

	//PRINT_APP(pcd.potentialCollisions.size() << " potential collisions\n");

	std::vector<Contact*> contacts;

	for (auto & col : pcd.potentialCollisions) {
		GJK::GJKContactGenerator cg = GJK::GJKContactGenerator(col->one, col->two);

		if (cg.testIntersection()) {
			PRINT_APP("found intersection found between " << col->one->name << " and " << col->two->name << "\n");

			ContactInfo contactInfo;
			bool ok = cg.createContact(&contactInfo);

			if (ok) {

				contacts.push_back(new Contact(contactInfo));

				glm::mat4 t = glm::scale(glm::translate(glm::mat4(1), contactInfo.points[0]), glm::vec3(0.25f));
				RenderSimpleMesh(meshes["sphere"], shaders["phong"], t, glm::vec3(1.0f, 0.0f, 0.0f));
				t = glm::scale(glm::translate(glm::mat4(1), contactInfo.points[1]), glm::vec3(0.25f));
				RenderSimpleMesh(meshes["sphere"], shaders["phong"], t, glm::vec3(0.0f, 1.0f, 0.0f));
				PRINT_APP("contact info = {\n\t");
				PRINT_APP("point = " << contactInfo.points[0] << "\n\t");
				PRINT_APP("normal = " << contactInfo.normal << "\n\t");
				PRINT_APP("penetration = " << contactInfo.penetration << "\n}\n");
			}
		}
	}

	ImpulseContactResolver *icr = new ImpulseContactResolver(contacts);
	icr->solve();
	delete icr;
}

void TestCollisionResolution::FrameEnd()
{
	DrawCoordinatSystem();
}


void TestCollisionResolution::RenderSimpleMesh(Mesh *mesh, Shader *shader, const glm::mat4 & modelMatrix, const glm::vec3 &color)
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

void TestCollisionResolution::OnInputUpdate(float deltaTime, int mods)
{
	if (!window->MouseHold(GLFW_MOUSE_BUTTON_2)) {
		if (window->KeyHold(GLFW_KEY_W)) {
			objects[selectedObjIndex]->body->position += glm::vec3(0, 0, -1) * deltaTime;
			objects[selectedObjIndex]->body->isAwake = true;
			objects[selectedObjIndex]->body->reset();
		}
		if (window->KeyHold(GLFW_KEY_S)) {
			objects[selectedObjIndex]->body->position += glm::vec3(0, 0, +1) * deltaTime;
			objects[selectedObjIndex]->body->isAwake = true;
			objects[selectedObjIndex]->body->reset();
		}
		if (window->KeyHold(GLFW_KEY_A)) {
			objects[selectedObjIndex]->body->position += glm::vec3(-1, 0, 0) * deltaTime;
			objects[selectedObjIndex]->body->isAwake = true;
			objects[selectedObjIndex]->body->reset();
		}
		if (window->KeyHold(GLFW_KEY_D)) {
			objects[selectedObjIndex]->body->position += glm::vec3(+1, 0, 0) * deltaTime;
			objects[selectedObjIndex]->body->isAwake = true;
			objects[selectedObjIndex]->body->reset();
		}
		if (window->KeyHold(GLFW_KEY_Q)) {
			objects[selectedObjIndex]->body->position += glm::vec3(0, -1, 0) * deltaTime;
			objects[selectedObjIndex]->body->isAwake = true;
			objects[selectedObjIndex]->body->reset();
		}
		if (window->KeyHold(GLFW_KEY_E)) {
			objects[selectedObjIndex]->body->position += glm::vec3(0, +1, 0) * deltaTime;
			objects[selectedObjIndex]->body->isAwake = true;
			objects[selectedObjIndex]->body->reset();
		}
		if (window->KeyHold(GLFW_KEY_SEMICOLON)) {
			//objects[selectedObjIndex]->body->applyTorqueAtLocalPoint(glm::vec3(0, 0, 1) * deltaTime, glm::vec3(0.5, 0, 0));
			objects[selectedObjIndex]->body->orientation = glm::normalize(glm::rotate(objects[selectedObjIndex]->body->orientation, 1.0f * deltaTime, glm::vec3(0, 1, 0)));
			objects[selectedObjIndex]->body->isAwake = true;
			objects[selectedObjIndex]->body->reset();
		}
		if (window->KeyHold(GLFW_KEY_APOSTROPHE)) {
			objects[selectedObjIndex]->body->orientation = glm::normalize(glm::rotate(objects[selectedObjIndex]->body->orientation, -1.0f * deltaTime, glm::vec3(0, 1, 0)));
			//objects[selectedObjIndex]->body->applyTorqueAtLocalPoint(glm::vec3(0, 0, -1) * deltaTime, glm::vec3(0.5, 0, 0));
			objects[selectedObjIndex]->body->isAwake = true;
			objects[selectedObjIndex]->body->reset();
		}
		if (window->KeyHold(GLFW_KEY_LEFT_BRACKET)) {
			objects[selectedObjIndex]->body->orientation = glm::normalize(glm::rotate(objects[selectedObjIndex]->body->orientation, 1.0f * deltaTime, glm::vec3(0, 0, 1)));
			//objects[selectedObjIndex]->body->applyTorqueAtLocalPoint(glm::vec3(0, 1, 0) * deltaTime, glm::vec3(0, 0, 0.5));
			objects[selectedObjIndex]->body->isAwake = true;
			objects[selectedObjIndex]->body->reset();
		}
		if (window->KeyHold(GLFW_KEY_RIGHT_BRACKET)) {
			objects[selectedObjIndex]->body->orientation = glm::normalize(glm::rotate(objects[selectedObjIndex]->body->orientation, -1.0f * deltaTime, glm::vec3(0, 0, 1)));
			//objects[selectedObjIndex]->body->applyTorqueAtLocalPoint(glm::vec3(0, -1, 0) * deltaTime, glm::vec3(0, 0, 0.5));
			objects[selectedObjIndex]->body->isAwake = true;
			objects[selectedObjIndex]->body->reset();
		}
	}
}

void TestCollisionResolution::OnKeyPress(int key, int mods)
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
		objects[selectedObjIndex]->body->applyForceAtWorldPoint(glm::vec3(12, 0, 8), glm::vec3(-0.5, 0.5, -0.5));
	}
	else if (key == GLFW_KEY_T)
	{
		pcd.fillPotentialCollisions();
		for (auto & pc : pcd.potentialCollisions)
			PRINT_APP("potential collision between " << pc->one->name << " and " << pc->two->name << "\n\n");
	}
	else if (key == GLFW_KEY_V)
	{
		for (auto & obj : objects)
			PRINT_APP(obj->toString());
	}
	else if (key >= GLFW_KEY_0 && key < GLFW_KEY_0 + objects.size() - 1)
		PRINT_APP(objects[key - GLFW_KEY_0]->toString());
	else if (key == GLFW_KEY_TAB) {
		selectedObjIndex = (selectedObjIndex + 1) % (objects.size() - 1);
		PRINT_APP("selected object = " << objects[selectedObjIndex]->name << "\n");
	}
}

void TestCollisionResolution::OnKeyRelease(int key, int mods)
{
}

void TestCollisionResolution::OnMouseMove(int mouseX, int mouseY, int deltaX, int deltaY)
{
}

void TestCollisionResolution::OnMouseBtnPress(int mouseX, int mouseY, int button, int mods)
{
}

void TestCollisionResolution::OnMouseBtnRelease(int mouseX, int mouseY, int button, int mods)
{
}

void TestCollisionResolution::OnMouseScroll(int mouseX, int mouseY, int offsetX, int offsetY)
{
}

void TestCollisionResolution::OnWindowResize(int width, int height)
{
}
