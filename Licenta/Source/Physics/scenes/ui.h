#pragma once
#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>

#include <Core/Engine.h>

#include <Physics/scenes/object.h>

namespace PhysicsUI {
void initUI();
void startUIFrame();
void showDemoWindow();
void showMainWindow(ObjectSpawner* objSpawner);
void renderUI();
}  /* end of namespace PhysicsUI */