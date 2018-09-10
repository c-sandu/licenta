#include <ctime>
#include <iostream>

using namespace std;

#include <Core/Engine.h>

#include <Physics/scenes/DemoScene.h>
#include <Physics/scenes/old/TestCollisionDetection.h>
#include <Physics/scenes/old/TestCollisionResolution.h>
#include <Physics/scenes/old/TestImGui.h>
#include <Physics/scenes/old/TestObjectSpawner.h>

int main(int argc, char** argv) {
  srand((unsigned int)time(NULL));

  // Create a window property structure
  WindowProperties wp;
  wp.resolution = glm::ivec2(1366, 768);
  wp.name.assign("Physics Simulation");

  // Init the Engine and create a new window with the defined properties
  WindowObject* window = Engine::Init(wp);

  // Create a new 3D world and start running it
  World* world = new DemoScene();
  world->Init();
  world->Run();

  // Signals to the Engine to release the OpenGL context
  Engine::Exit();

  return 0;
}