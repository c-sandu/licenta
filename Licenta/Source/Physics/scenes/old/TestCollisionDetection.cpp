#include "TestCollisionDetection.h"

#include <Core/Engine.h>
#include <glm/gtx/string_cast.hpp>

#include <Physics/debug.h>
#include <Physics/collision/detection/contactgeneration.h>

TestCollisionDetection::TestCollisionDetection()
{
}

TestCollisionDetection::~TestCollisionDetection()
{
}

void TestCollisionDetection::Init()
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
		lightPosition = PhysicsSettings::get().sceneProperties.lightning.lightPosition;
		lightDirection = PhysicsSettings::get().sceneProperties.lightning.lightDirection;
		materialShininess = PhysicsSettings::get().sceneProperties.lightning.materialShininess;
		materialKd = PhysicsSettings::get().sceneProperties.lightning.materialKd;
		materialKs = PhysicsSettings::get().sceneProperties.lightning.materialKs;
	}

	GetSceneCamera()->SetPosition(PhysicsSettings::get().sceneProperties.defaultCameraPosition);

	polygonMode = GL_FILL;

	{
		boxDefault = new RigidBody(glm::vec3(0, 2, 0), glm::vec3(1));
		boxDefault->setMass(PhysicsSettings::get().rigidBodies.defaultMass / 2);
		boxDefault->updateTransformMatrix();
		boxDefault->setInertiaTensor(RigidBody::inertiaTensorCube(PhysicsSettings::get().shapes.box.halfSizes * boxDefault->scale));
		boxDefault->orientation = glm::normalize(glm::rotate(boxDefault->orientation, (float)M_PI * 0.5f, glm::vec3(1, 0, 1)));
		/*boxDefault->applyForce(glm::vec3(0, 8, 0));
		boxDefault->applyForceAtLocalPoint(glm::vec3(8, 0, 4), glm::vec3(-0.5, 0.5, -0.5));*/
		PhysicsObject *obj = new PhysicsObject();
		obj->body = boxDefault;
		obj->mesh = meshes["box"];
		obj->color = glm::vec3(1, 1, 0);
		obj->collider = new OBBCollider(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale, obj, meshes["box"]);
		obj->collider->setRigidBody(obj->body);
		obj->collider->updateInternals();
		obj->name.assign("boxDefault");
		obj->shape = new Box(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale);
		objects.push_back(obj);
		pcd.addCollider((OBBCollider*)obj->collider);
	}
	//{
	//	boxSmall = new RigidBody(glm::vec3(-8, 0, 0), glm::vec3(0.25f));
	//	boxSmall->setMass(PhysicsSettings::get().rigidBodies.defaultMass);
	//	boxSmall->updateTransformMatrix();
	//	boxSmall->setInertiaTensor(RigidBody::inertiaTensorCube(PhysicsSettings::get().shapes.box.halfSizes * boxSmall->scale));
	//	boxSmall->applyForce(glm::vec3(0, 8, 0));
	//	boxSmall->applyForceAtLocalPoint(glm::vec3(8, 0, 4), glm::vec3(-0.5, 0.5, -0.5));
	//	PhysicsObject *obj = new PhysicsObject();
	//	obj->body = boxSmall;
	//	obj->mesh = meshes["box"];
	//	obj->color = glm::vec3(1, 0, 0);
	//	obj->collider = new OBBCollider(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale, obj);
	//	obj->collider->setRigidBody(obj->body);
	//	obj->collider->updateInternals();
	//	obj->name.assign("boxSmall");
	//	objects.push_back(obj);
	//	pcd.addCollider((OBBCollider*)obj->collider);
	//}
	//{
	//	boxLarge = new RigidBody(glm::vec3(-4, 0, 0), glm::vec3(2.0f));
	//	boxLarge->setMass(PhysicsSettings::get().rigidBodies.defaultMass);
	//	boxLarge->updateTransformMatrix();
	//	boxLarge->setInertiaTensor(RigidBody::inertiaTensorCube(PhysicsSettings::get().shapes.box.halfSizes * boxDefault->scale));
	//	boxLarge->applyForce(glm::vec3(0, 8, 0));
	//	boxLarge->applyForceAtLocalPoint(glm::vec3(8, 0, 4), glm::vec3(-0.5, 0.5, -0.5));
	//	PhysicsObject *obj = new PhysicsObject();
	//	obj->body = boxLarge;
	//	obj->mesh = meshes["box"];
	//	obj->color = glm::vec3(0, 1, 0);
	//	obj->collider = new OBBCollider(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale, obj);
	//	obj->collider->setRigidBody(obj->body);
	//	obj->collider->updateInternals();
	//	obj->name.assign("boxLarge");
	//	objects.push_back(obj);
	//	pcd.addCollider((OBBCollider*)obj->collider);
	//}
	//{
	//	boxHeavy = new RigidBody(glm::vec3(4, 0, 0));
	//	boxHeavy->setMass(PhysicsSettings::get().rigidBodies.defaultMass * 4);
	//	boxHeavy->updateTransformMatrix();
	//	boxHeavy->setInertiaTensor(RigidBody::inertiaTensorCube());
	//	boxHeavy->applyForce(glm::vec3(0, 8, 0));
	//	boxHeavy->applyForceAtLocalPoint(glm::vec3(8, 0, 4), glm::vec3(-0.5, 0.5, -0.5));
	//	PhysicsObject *obj = new PhysicsObject();
	//	obj->body = boxHeavy;
	//	obj->mesh = meshes["box"];
	//	obj->color = glm::vec3(0, 0, 1);
	//	obj->collider = new OBBCollider(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale, obj);
	//	obj->collider->setRigidBody(obj->body);
	//	obj->collider->updateInternals();
	//	obj->name.assign("boxHeavy");
	//	objects.push_back(obj);
	//	pcd.addCollider((OBBCollider*)obj->collider);
	//}
	//{
	//	boxSmallHeavy = new RigidBody(glm::vec3(8, 0, 0), glm::vec3(0.25f));
	//	boxSmallHeavy->setMass(PhysicsSettings::get().rigidBodies.defaultMass * 4);
	//	boxSmallHeavy->updateTransformMatrix();
	//	boxSmallHeavy->setInertiaTensor(RigidBody::inertiaTensorCube(PhysicsSettings::get().shapes.box.halfSizes * boxDefault->scale));
	//	boxSmallHeavy->applyForce(glm::vec3(0, 8, 0));
	//	boxSmallHeavy->applyForceAtLocalPoint(glm::vec3(8, 0, 4), glm::vec3(-0.5, 0.5, -0.5));
	//	PhysicsObject *obj = new PhysicsObject();
	//	obj->body = boxSmallHeavy;
	//	obj->mesh = meshes["box"];
	//	obj->color = glm::vec3(0, 1, 1);
	//	obj->collider = new OBBCollider(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale, obj);
	//	obj->collider->setRigidBody(obj->body);
	//	obj->collider->updateInternals();
	//	obj->name.assign("boxSmallHeavy");
	//	objects.push_back(obj);
	//	pcd.addCollider((OBBCollider*)obj->collider);
	//}

	{
		boxTall = new RigidBody(glm::vec3(0, 3, -3), glm::vec3(1, 2, 1));
		boxTall->setMass(PhysicsSettings::get().rigidBodies.defaultMass);
		boxTall->updateTransformMatrix();
		boxTall->setInertiaTensor(RigidBody::inertiaTensorCube(PhysicsSettings::get().shapes.box.halfSizes * boxTall->scale));
		//boxTall->orientation = glm::rotate(boxTall->orientation, 0.1f, glm::vec3(0, 1, 0));
		/*boxTall->applyForce(glm::vec3(0, 8, 0));
		boxTall->applyForceAtLocalPoint(glm::vec3(8, 0, 4), glm::vec3(-0.5, 0.5, -0.5));*/
		PhysicsObject *obj = new PhysicsObject();
		obj->body = boxTall;
		obj->mesh = meshes["box"];
		obj->color = glm::vec3(1, 0, 1);
		obj->collider = new OBBCollider(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale, obj, meshes["box"]);
		obj->collider->setRigidBody(obj->body);
		obj->collider->updateInternals();
		obj->name.assign("boxTall");
		obj->shape = new Box(PhysicsSettings::get().shapes.box.halfSizes * obj->body->scale);
		objects.push_back(obj);
		pcd.addCollider((OBBCollider*)obj->collider);
	}

	//{
	//	PhysicsObject *obj = new PhysicsObject();
	//	obj->body = NULL;
	//	obj->mesh = meshes["plane"];
	//	obj->color = glm::vec3(1, 1, 1);
	//	obj->collider = new PlaneCollider(glm::vec3(0, 1, 0), -0.1f, obj, meshes["plane"]);
	//	obj->collider->setRigidBody(obj->body);
	//	obj->name.assign("planeXoZ");
	//	obj->shape = new Plane(25.0f, 25.0f);
	//	objects.push_back(obj);
	//	pcd.addCollider((PlaneCollider*)obj->collider);
	//}

	selectedObjIndex = 0;

	PRINT_APP("Positions = {\n\t");
	for (glm::vec3 &p : meshes["box"]->positions)
		PRINT_APP(glm::to_string(p) << "\n\t");
	PRINT_APP("}\n");

	PRINT_APP("Vertices = {\n\t");
	for (VertexFormat &p : meshes["box"]->vertices)
		PRINT_APP("pos = " << glm::to_string(p.position) << ", normal = " << glm::to_string(p.normal) <<"\n\t");
	PRINT_APP("}\n");

	PRINT_APP("Indices = {\n\t");
	for (unsigned short &p : meshes["box"]->indices)
		PRINT_APP(p << ", ");
	PRINT_APP("\n}\n");
}

void TestCollisionDetection::FrameStart()
{
	// clears the color buffer (using the previously set color) and depth buffer
	glClearColor(0, 0, 0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glm::ivec2 resolution = window->GetResolution();
	// sets the screen area where to draw
	glViewport(0, 0, resolution.x, resolution.y);
}

void TestCollisionDetection::Update(float deltaTime)
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
		RenderSimpleMesh(obj->collider->mesh, shaders["phong"], obj->getColTransformMatrix(), glm::vec3(1) - obj->color);
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

	pcd.fillPotentialCollisions();

	//PRINT_APP(pcd.potentialCollisions.size() << " potential collisions\n");

	for (auto & col : pcd.potentialCollisions) {
		GJKEPA::GJKEPACollisionPointGenerator cg = GJKEPA::GJKEPACollisionPointGenerator(col->one, col->two);

		if (cg.testIntersection()) {
			PRINT_APP("found intersection found between " << col->one->name << " and " << col->two->name << "\n");

			CollisionPoint contact;
			if (cg.createCollisionPoint(&contact)) {
				glm::mat4 t = glm::scale(glm::translate(glm::mat4(1), contact.points[0]), glm::vec3(0.25f));
				RenderSimpleMesh(meshes["sphere"], shaders["phong"], t, glm::vec3(1.0f, 0.0f, 0.0f));
				t = glm::scale(glm::translate(glm::mat4(1), contact.points[1]), glm::vec3(0.25f));
				RenderSimpleMesh(meshes["sphere"], shaders["phong"], t, glm::vec3(0.0f, 1.0f, 0.0f));
				PRINT_APP("contact info = {\n\t");
				PRINT_APP("point = " << contact.points[0] << "\n\t");
				PRINT_APP("normal = " << contact.normal << "\n\t");
				PRINT_APP("penetration = " << contact.penetration << "\n}\n");
			}
		}
	}
}

void TestCollisionDetection::FrameEnd()
{
	DrawCoordinatSystem();
}


void TestCollisionDetection::RenderSimpleMesh(Mesh *mesh, Shader *shader, const glm::mat4 & modelMatrix, const glm::vec3 &color)
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

void TestCollisionDetection::OnInputUpdate(float deltaTime, int mods)
{
	if (!window->MouseHold(GLFW_MOUSE_BUTTON_2)) {
		if (window->KeyHold(GLFW_KEY_W)) {
			objects[selectedObjIndex]->body->position += glm::vec3(0, 0, -1) * deltaTime;
			objects[selectedObjIndex]->body->isAwake = true;
			objects[selectedObjIndex]->body->resetMovement();
		}
		if (window->KeyHold(GLFW_KEY_S)) {
			objects[selectedObjIndex]->body->position += glm::vec3(0, 0, +1) * deltaTime;
			objects[selectedObjIndex]->body->isAwake = true;
			objects[selectedObjIndex]->body->resetMovement();
		}
		if (window->KeyHold(GLFW_KEY_A)) {
			objects[selectedObjIndex]->body->position += glm::vec3(-1, 0, 0) * deltaTime;
			objects[selectedObjIndex]->body->isAwake = true;
			objects[selectedObjIndex]->body->resetMovement();
		}
		if (window->KeyHold(GLFW_KEY_D)) {
			objects[selectedObjIndex]->body->position += glm::vec3(+1, 0, 0) * deltaTime;
			objects[selectedObjIndex]->body->isAwake = true;
			objects[selectedObjIndex]->body->resetMovement();
		}
		if (window->KeyHold(GLFW_KEY_Q)) {
			objects[selectedObjIndex]->body->position += glm::vec3(0, -1, 0) * deltaTime;
			objects[selectedObjIndex]->body->isAwake = true;
			objects[selectedObjIndex]->body->resetMovement();
		}
		if (window->KeyHold(GLFW_KEY_E)) {
			objects[selectedObjIndex]->body->position += glm::vec3(0, +1, 0) * deltaTime;
			objects[selectedObjIndex]->body->isAwake = true;
			objects[selectedObjIndex]->body->resetMovement();
		}
		if (window->KeyHold(GLFW_KEY_SEMICOLON)) {
			//objects[selectedObjIndex]->body->applyTorqueAtLocalPoint(glm::vec3(0, 0, 1) * deltaTime, glm::vec3(0.5, 0, 0));
			objects[selectedObjIndex]->body->orientation = glm::normalize(glm::rotate(objects[selectedObjIndex]->body->orientation, 1.0f * deltaTime, glm::vec3(0, 1, 0)));
			objects[selectedObjIndex]->body->isAwake = true;
			objects[selectedObjIndex]->body->resetMovement();
		}
		if (window->KeyHold(GLFW_KEY_APOSTROPHE)) {
			objects[selectedObjIndex]->body->orientation = glm::normalize(glm::rotate(objects[selectedObjIndex]->body->orientation, -1.0f * deltaTime, glm::vec3(0, 1, 0)));
			//objects[selectedObjIndex]->body->applyTorqueAtLocalPoint(glm::vec3(0, 0, -1) * deltaTime, glm::vec3(0.5, 0, 0));
			objects[selectedObjIndex]->body->isAwake = true;
			objects[selectedObjIndex]->body->resetMovement();
		}
		if (window->KeyHold(GLFW_KEY_LEFT_BRACKET)) {
			objects[selectedObjIndex]->body->orientation = glm::normalize(glm::rotate(objects[selectedObjIndex]->body->orientation, 1.0f * deltaTime, glm::vec3(0, 0, 1)));
			//objects[selectedObjIndex]->body->applyTorqueAtLocalPoint(glm::vec3(0, 1, 0) * deltaTime, glm::vec3(0, 0, 0.5));
			objects[selectedObjIndex]->body->isAwake = true;
			objects[selectedObjIndex]->body->resetMovement();
		}
		if (window->KeyHold(GLFW_KEY_RIGHT_BRACKET)) {
			objects[selectedObjIndex]->body->orientation = glm::normalize(glm::rotate(objects[selectedObjIndex]->body->orientation, -1.0f * deltaTime, glm::vec3(0, 0, 1)));
			//objects[selectedObjIndex]->body->applyTorqueAtLocalPoint(glm::vec3(0, -1, 0) * deltaTime, glm::vec3(0, 0, 0.5));
			objects[selectedObjIndex]->body->isAwake = true;
			objects[selectedObjIndex]->body->resetMovement();
		}
	}
}

void TestCollisionDetection::OnKeyPress(int key, int mods)
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
	} else if (key == GLFW_KEY_SPACE) {
		objects[selectedObjIndex]->body->isAwake = objects[selectedObjIndex]->body->isAwake ? false : true;
	}
}

void TestCollisionDetection::OnKeyRelease(int key, int mods)
{
}

void TestCollisionDetection::OnMouseMove(int mouseX, int mouseY, int deltaX, int deltaY)
{
}

void TestCollisionDetection::OnMouseBtnPress(int mouseX, int mouseY, int button, int mods)
{
}

void TestCollisionDetection::OnMouseBtnRelease(int mouseX, int mouseY, int button, int mods)
{
}

void TestCollisionDetection::OnMouseScroll(int mouseX, int mouseY, int offsetX, int offsetY)
{
}

void TestCollisionDetection::OnWindowResize(int width, int height)
{
}
