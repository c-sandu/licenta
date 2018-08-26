#include "ui.h"

#include <Core/Engine.h>
#include <imgui/addons/imguifilesystem/imguifilesystem.h>

#define D_SCL_SECURE_NO_WARNINGS

void PhysicsUI::initUI()
{
	/* Init ImGui */
	{
		const char* glsl_version = "#version 150";
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
		glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only

																		// Setup Dear ImGui binding
		IMGUI_CHECKVERSION();
		ImGui::CreateContext();
		ImGuiIO& io = ImGui::GetIO(); (void)io;
		//io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
		//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;   // Enable Gamepad Controls

		ImGui_ImplGlfw_InitForOpenGL(Engine::GetWindow()->GetGLFWWindow(), false);
		ImGui_ImplOpenGL3_Init(glsl_version);

		// Setup style
		ImGui::StyleColorsDark();
		//ImGui::StyleColorsClassic();

		// Load Fonts
		// - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them. 
		// - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple. 
		// - If the file cannot be loaded, the function will return NULL. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
		// - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
		// - Read 'misc/fonts/README.txt' for more instructions and details.
		// - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
		//io.Fonts->AddFontDefault();
		//io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
		//io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
		//io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
		//io.Fonts->AddFontFromFileTTF("../../misc/fonts/ProggyTiny.ttf", 10.0f);
		//ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, NULL, io.Fonts->GetGlyphRangesJapanese());
		//IM_ASSERT(font != NULL);
	}
}

void PhysicsUI::startUIFrame()
{
	// Start the Dear ImGui frame
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
}

void PhysicsUI::showDemoWindow()
{
	bool show_demo_window = true;
	ImGui::ShowDemoWindow(&show_demo_window);
}


static bool DragFloatNEx(const char* labels[], float* v, int components, float v_speed, float v_min, float v_max,
	const char* display_format, float power)
{
	bool value_changed = false;

	static unsigned int counter = 0;

	ImGui::BeginGroup();
	ImGui::PushID(labels);
	ImGui::PushItemWidth(128);
	for (int i = 0; i < components; i++)
	{
		ImGui::PushID(i);
		std::string s = std::string(labels[i]) + " = " + display_format;
		value_changed |= ImGui::DragFloat("", &v[i], v_speed, v_min, v_max, (std::string(labels[i+1]) + " = " + display_format).c_str(), power);
		ImGui::PopID();
		ImGui::SameLine();
	}
	ImGui::PopItemWidth();
	ImGui::Text(labels[0]);
	ImGui::PopID();
	ImGui::EndGroup();

	return value_changed;
}

void PhysicsUI::showMainWindow(ObjectSpawner *objSpawner)
{
	
	ImGui::Begin("Physics User Interface", NULL/*, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize*/);
	
	ImGui::PushItemWidth(ImGui::GetFontSize() * -12);           // Use fixed width for labels (by passing a negative value), the rest goes to widgets. We choose a width proportional
	
	const unsigned int myOne = 1;

	if (ImGui::CollapsingHeader("Simulation"))
	{
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::Text((std::to_string(objSpawner->objects.size()) + " objects in scene ").c_str());
		{
			std::string nextObject;

			switch (objSpawner->nextObject) {
			case box:
				nextObject.assign("box");
				break;
			case longBox:
				nextObject.assign("longbox");
				break;
			case sphere:
				nextObject.assign("sphere");
				break;
			case cylinder:
				nextObject.assign("cylinder");
				break;
			case capsule:
				nextObject.assign("capsule");
				break;
			}
			ImGui::Text(("next spawned object: " + nextObject).c_str());
		}

		ImGui::Checkbox("render colliders", &PhysicsSettings::get().rendering.renderColliders);
		ImGui::Checkbox("render contacts", &PhysicsSettings::get().rendering.renderContacts);
		if (PhysicsSettings::get().rendering.renderContacts) {
			ImGui::SameLine();
			ImGui::Checkbox("render contact normals", &PhysicsSettings::get().rendering.renderContactNormals);
		}
		ImGui::Checkbox("render object spawner", &PhysicsSettings::get().rendering.renderSpawner);
		ImGui::Checkbox("highlight selected object", &PhysicsSettings::get().rendering.renderSelection);

		if (ImGui::TreeNode("Spawner Properties"))
		{
			ImGui::Checkbox("randomize next object", &objSpawner->randomizeNextObject);
			ImGui::Checkbox("randomize object properties", &objSpawner->randomizeProperties);
			ImGui::Checkbox("apply starting impulse", &objSpawner->applyStartingImpulse);
			ImGui::TreePop();
		}

		if (ImGui::TreeNode("Simulation settings"))
		{
			if (ImGui::TreeNode("velocity damping factors")) {
				ImGui::PushItemWidth(128);
				ImGui::DragFloat("linear", &PhysicsSettings::get().damping.linear, 0.005f, 0.5f, 1.0f, "x%.2f");
				ImGui::DragFloat("angular", &PhysicsSettings::get().damping.angular, 0.005f, 0.5f, 1.0f, "x%.2f");
				ImGui::PopItemWidth();
				ImGui::TreePop();
			}

			if (ImGui::TreeNode("force, torque and impulse multipliers"))
			{
				ImGui::PushItemWidth(128);
				ImGui::DragFloat("torque", &PhysicsSettings::get().forceMultipliers.torque, 0.05f, 1.0f, 2048.0f, "x%.2f", 2.0f);
				ImGui::DragFloat("force", &PhysicsSettings::get().forceMultipliers.force, 0.05f, 1.0f, 2048.0f, "x%.2f", 2.0f);
				ImGui::DragFloat("impulse", &PhysicsSettings::get().forceMultipliers.impulse, 0.05f, 1.0f, 2048.0f, "x%.2f", 2.0f);
				ImGui::PopItemWidth();
				ImGui::TreePop();
			}

			if (ImGui::TreeNode("collision detection algorithms"))
			{
				ImGui::PushItemWidth(128);
				ImGui::InputScalar("GJK max iterations", ImGuiDataType_U32, &PhysicsSettings::get().gjkepa.gjkMaxIters, &myOne);
				ImGui::InputScalar("EPA max iterations", ImGuiDataType_U32, &PhysicsSettings::get().gjkepa.epaMaxIters, &myOne);
				ImGui::PopItemWidth();
				ImGui::TreePop();
			}

			if (ImGui::TreeNode("collision resolution"))
			{
				ImGui::PushItemWidth(128);
				ImGui::InputScalar("max penetration iterations", ImGuiDataType_U32, &PhysicsSettings::get().collisionResolution.penMaxIterations, &myOne);
				ImGui::InputScalar("max velocity iterations", ImGuiDataType_U32, &PhysicsSettings::get().collisionResolution.velMaxIterations, &myOne);

				ImGui::DragFloat("minimum velocity for restitution", &PhysicsSettings::get().collisionResolution.minVelocityForRestitution, 0.005f, 0.0f, 10.0f, "> %.2f");
				ImGui::DragFloat("angular movement limit factor", &PhysicsSettings::get().collisionResolution.angularMovementLimitFactor, 0.005f, 0.0f, 5.0f);
				ImGui::DragFloat("persistent contact distance threshold", &PhysicsSettings::get().collisionResolution.minVelocityForRestitution, 0.005f, 0.0f, 1.0f, "< %.3f");
				
				ImGui::DragFloat("coefficient interpolation alpha", &PhysicsSettings::get().collisionResolution.coefInterpAlpha, 0.005f, 0.0f, 1.0f, "min + %.3f * max");
				ImGui::PopItemWidth();
				
				ImGui::TreePop();
			}

			if (ImGui::TreeNode("rigid bodies default values"))
			{
				ImGui::PushItemWidth(128);
				ImGui::DragFloat("mass", &PhysicsSettings::get().rigidBodies.defaultMass, 1.0f, 0.0f, 10000.0f, "%.1f");
				
				ImGui::DragFloat("friction coefficient", &PhysicsSettings::get().rigidBodies.defaultFrictionCoef, 0.005f, 0.0f, 1.0f);
				ImGui::DragFloat("restitution coefficient", &PhysicsSettings::get().rigidBodies.defaultRestitutionCoef, 0.005f, 0.0f, 1.0f);
				ImGui::PopItemWidth();
				ImGui::TreePop();
			}

			const char* names[] = { "gravity", "Ox", "Oy", "Oz" };
			DragFloatNEx(names, glm::value_ptr(PhysicsSettings::get().gravity), 3, 0.05f, -1000, 1000, "%.2f", 1);

			ImGui::PushItemWidth(400);
			ImGui::DragFloat("timescale", &PhysicsSettings::get().timeScale, 0.005f, 0.0f, 8.0f, "x%.2f");
			ImGui::DragFloat("sleep motion threshold", &PhysicsSettings::get().rigidBodies.sleepMotionThreshold, 0.005f, 0.0f, 2.0f, "%.3f");
			ImGui::PopItemWidth();
			ImGui::TreePop();
		}
	}
	
	if (ImGui::CollapsingHeader("Selected object control"))
	{
		static bool editObject = false;
		ImGui::Text(("selected object = " + objSpawner->selectedObject->name).c_str());
		{
			const char* names[] = { "position", "X", "Y", "Z" };
			if (DragFloatNEx(names, glm::value_ptr(objSpawner->selectedObject->body->position), 3, 0.05f, -1000, 1000, "%.2f", 1))
				objSpawner->selectedObject->body->resetMovement();
		}

		{
			glm::vec3 euler = glm::eulerAngles(objSpawner->selectedObject->body->orientation);
			euler.x = DEGREES(euler.x);
			euler.y = DEGREES(euler.y);
			euler.z = DEGREES(euler.z);
			const char* names[] = { "orientation(Euler Angles)", "pitch", "yaw", "roll" };
			if (DragFloatNEx(names, glm::value_ptr(euler), 3, 0.05f, -180.0f, 180.0f, "%.2f", 1)) {
				euler.x = RADIANS(euler.x);
				euler.y = RADIANS(euler.y);
				euler.z = RADIANS(euler.z);
				objSpawner->selectedObject->body->orientation = glm::quat(euler);
				objSpawner->selectedObject->body->resetMovement();
			}
		}

		{
			const char* names[] = { "scale", "X", "Y", "Z" };
			DragFloatNEx(names, glm::value_ptr(objSpawner->selectedObject->body->scale), 3, 0.05f, 0.001f, 8.0f, "%.2f", 1);
		}

		ImGui::PushItemWidth(400);
		if (ImGui::DragFloat("mass", &objSpawner->selectedObject->body->mass, 0.05f, 0.001f, 1024.0f, "%.2f"))
			objSpawner->selectedObject->body->setMass(objSpawner->selectedObject->body->mass, false);
		ImGui::PopItemWidth();

		ImGui::Checkbox("is static", &objSpawner->selectedObject->body->isStatic);
		ImGui::SameLine();
		ImGui::Checkbox("is awake", &objSpawner->selectedObject->body->isAwake);

		ImGui::PushItemWidth(400);
		ImGui::DragFloat("friction coeff", &objSpawner->selectedObject->body->frictionCoef, 0.005f, 0.0f, 1.0f);
		ImGui::DragFloat("restitution coeff", &objSpawner->selectedObject->body->restitutionCoef, 0.005f, 0.0f, 1.0f);
		ImGui::PopItemWidth();

		ImGui::PushItemWidth(400);
		ImGui::Text(objSpawner->selectedObject->body->toStringPrivateFields().c_str());
		ImGui::PopItemWidth();
	}
	
	static unsigned int saveCounter = 0;
	static char buf1[1024] = "";
	sprintf_s(buf1, "scene_%02d.json", saveCounter);
	ImGui::Text("file: ");
	ImGui::SameLine();
	ImGui::InputText("", buf1, 1024);
	ImGui::SameLine();
	if (ImGui::Button("save scene")) {
		objSpawner->saveToFile(buf1);
		saveCounter++;
	}

	// Inside a ImGui window:

	static char buf2[1024] = "";
	if (!strcmp(buf2, ""))
		strcpy_s(buf2, PhysicsSettings::get().defaultLoadedScene.c_str());
	ImGui::Text("file: %s", buf2);
	ImGui::SameLine();
	const bool browseButtonPressed = ImGui::Button("...");                          // we need a trigger boolean variable
	static ImGuiFs::Dialog dlg;                                                     // one per dialog (and must be static)
	const char* chosenPath = dlg.chooseFileDialog(browseButtonPressed);             // see other dialog types and the full list of arguments for advanced usage
	if (strlen(chosenPath)>0) {
		// A path (chosenPath) has been chosen RIGHT NOW. However we can retrieve it later more comfortably using: dlg.getChosenPath()
	}
	if (strlen(dlg.getChosenPath())>0) {
		sprintf_s(buf2, "%s", dlg.getChosenPath());
	}
	ImGui::SameLine();
	if (ImGui::Button("load scene")) {
		objSpawner->loadFromFile(buf2);
	}
	ImGui::End(); /* Main Window */
}

void PhysicsUI::renderUI()
{
	// Rendering
	ImGui::Render();
	int display_w, display_h;
	glfwMakeContextCurrent(Engine::GetWindow()->GetGLFWWindow());
	glfwGetFramebufferSize(Engine::GetWindow()->GetGLFWWindow(), &display_w, &display_h);
	glViewport(0, 0, display_w, display_h);
	/*glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
	glClear(GL_COLOR_BUFFER_BIT);*/
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

#undef D_SCL_SECURE_NO_WARNINGS
