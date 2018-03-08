#pragma once
#include <imgui.h>
#include <memory>
#include <map>

struct CorGuiContext;

struct CorGuiWindowFlags {
	int width = 800;
	int height = 600;
};

typedef std::shared_ptr<CorGuiContext> CorGuiContextPtr;

namespace CorGui {
	
	CorGuiContextPtr CreateContext();
	int CreateWindow(const std::string& name, int w, int h);
	void DestroyContext(CorGuiContextPtr ctx);
	bool Begin(const std::string& name, CorGuiWindowFlags flags);	
	bool End();
	void BeginFrame();
	void EndFrame();
	bool ShouldClose();
	void ShutDown();

	void PushState();
	void PopState();
	
	// Transforms
	void Translate(float x, float y, float z);
	
	// Geometry
	void Sphere(const std::string& name, float radius);

	// TODO template types
	void PointCloud(const std::string& name, const float* points, int npoints);
};
