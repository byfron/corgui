# topologui / corgui
Layer on top of imgui to support OpenGL 3D visualization

Usage:

CorGui::Begin("View1", 800, 600, CorGui::USER_CAM);

CorGui::Sphere()
CorGui::Points()
CorGui::Mesh("mesh_file.fbx")

CorGui::GlBegin("view_name")
CorGui::Translate()
CorGui::Color()
CorGui::Rotate()
CorGui::Sphere("sphere1")
CorGui::Cube()
CorGui::Vector()
CorGui::GlEnd()

CorGui::Points()

CorGui::End()