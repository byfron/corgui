#include <corgui.hpp>

int main(int, char**)
{
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
	CorGui::CreateContext();

	// Create OpenGL window
	CorGui::CreateWindow("examle", 800, 600);
	
	CorGuiWindowFlags flags;
	
    // Main loop
    while (!CorGui::ShouldClose())
    {

		CorGui::BeginFrame();
		CorGui::Begin("view0", flags);
		CorGui::Sphere("sphere0", 0.1);
		CorGui::End();		
		CorGui::EndFrame();
    }

    // Cleanup
	CorGui::ShutDown();

    return 0;
}
