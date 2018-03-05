#pragma once
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
	void DestroyContext(CorGuiContextPtr ctx);
	bool Begin(const std::string& name, CorGuiWindowFlags flags);
	bool End();

	template <typename T>
	void SetUniform(const std::string& name, const T& object);

	void Sphere(const std::string& name, float radius);
};
