#include "corgui.hpp"
#include <examples/opengl3_example/imgui_impl_glfw_gl3.h>
#include "GL/gl3w.h"
#include "imgui.h"
#include "shaders.hpp"
#include <string>
#include <vector>
#include <stack>
#include <Eigen/Dense>
#include <iostream>
#include <stdio.h>
#include <GLFW/glfw3.h>

typedef Eigen::Matrix4f Mat4f;
typedef Eigen::Vector2f Vec2f;
typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector4f Vec4f;


struct CorGuiShader {
	CorGuiShader(const std::string& shader_code, GLenum shader_type) {
		id = glCreateShader(shader_type);
		compileShader(id, shader_code);
	}

	bool compileShader(GLuint shader_id, const std::string& shader_code) {
			
		char const * code_ptr = shader_code.c_str();
		glShaderSource(shader_id, 1, &code_ptr , NULL);
		glCompileShader(shader_id);
		
		// sanity check
		int info_length;
		GLint result = GL_FALSE;
		glGetShaderiv(shader_id, GL_COMPILE_STATUS, &result);
		glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &info_length);
		if (result == GL_FALSE) {
			std::vector<char> error_msg(info_length+1);
			glGetShaderInfoLog(shader_id, info_length, NULL, error_msg.data());
			//mmlog_error("Error compiling shader: %s", error_msg.data());
			return false;
		}
		
		return true;
	}

	GLuint id;
};

struct CorGuiProgram {

	static constexpr GLuint undefined_program_id = (std::numeric_limits<GLuint>::max)();

	CorGuiProgram(const std::string& vs = shaders::single_color_vs, //default_vs,
				  const std::string& fs = shaders::single_color_fs) { //default_fs) {
		shaders.push_back(CorGuiShader(vs, GL_VERTEX_SHADER));
		shaders.push_back(CorGuiShader(fs, GL_FRAGMENT_SHADER));
		compileProgram();
	}
	
	bool compileProgram() {

		id = glCreateProgram();

		for (auto shader : shaders) {
			glAttachShader(id, shader.id);
		}

		glLinkProgram(id);

		for (auto shader : shaders) {
			glDetachShader(id, shader.id);
			glDeleteShader(shader.id);
		}

		return true;	}

	std::vector<CorGuiShader> shaders;
	GLuint id;
};
typedef std::shared_ptr<CorGuiProgram> CorGuiProgramPtr;

struct CorGuiCamera {

	CorGuiCamera(float fov, float near_plane, float far_plane, float aspect_ratio) :
		fov(fov),
		near_plane(near_plane),
		far_plane(far_plane),
		aspect_ratio(aspect_ratio) {
		
		float tanHalfFovy = tan(fov / 2.0);
		projection <<
			1.0 / (aspect_ratio * tanHalfFovy), 0, 0, 0,
			0, 1.0 / (tanHalfFovy), 0, 0,
			0, 0, -(far_plane+near_plane)/(far_plane-near_plane), -1,
			0, 0, -(2*near_plane*far_plane)/(far_plane-near_plane), 0;

		pitch = 0.0f;
		yaw = 0.0f;
		pan_horizontal = 0;
		pan_vertical = 0;
		aim = Vec3f(0.0f, -1.0f, 0.0f);
		distance = 0.9;
		
		updateTransformation();
	}
	
	void mouseDrag(const Vec2f &mouse_delta)
	{
		if (mouse_delta.norm() < 50.0f) {
			pitch += mouse_delta[1];
			yaw   += mouse_delta[0];
			updateTransformation();
		}
	}

	void moveForward(float speed)
	{
		distance -= speed;
		updateTransformation();
	}

	void moveRight(float speed)
	{
		pan_horizontal -= speed;
		updateTransformation();
	}
	
	void moveUp(float speed)
	{
		pan_vertical -= speed;
		updateTransformation();
	}

	void setAim(const Vec3f& a) {
		aim = a;
		updateTransformation();
	}
	
	void pan(const Vec2f &mouse_delta, float speed) {
		if (mouse_delta.norm() < 50.0f) { // fix jump
			if (mouse_delta[1] > 0.0f) {
				moveUp(mouse_delta[1]*speed);
			} else if (mouse_delta[1] < 0.0f) {
				moveUp(mouse_delta[1]*speed);
			}
			if (mouse_delta[0] > 0.0f) {
				moveRight(-mouse_delta[0]*speed);
			} else if (mouse_delta[0] < 0.0f) {
				moveRight(-mouse_delta[0]*speed);
			}
		}
	}	

	void processImguiInput() {
		ImGuiIO& io = ImGui::GetIO();
		float mouseSensitivity = 0.01f;

		if (io.WantCaptureMouse) {
			if (ImGui::IsMouseDragging(1)) {
				ImVec2 md = ImGui::GetMouseDragDelta(1);
				mouseDrag(mouseSensitivity * Vec2f((float)(md.x),
												   (float)(md.y)));
				ImGui::ResetMouseDragDelta(1);
			}

			if (ImGui::IsMouseDragging(2)) {
				ImVec2 md = ImGui::GetMouseDragDelta(2);
				pan(mouseSensitivity * Vec2f((float)(md.x),
											 (float)(md.y)), 0.2);
				ImGui::ResetMouseDragDelta(2);
			}

			moveForward(10.0 * mouseSensitivity * io.MouseWheel);
		}
	}
	
	Mat4f getViewProjectionMatrix() const {
		return projection.transpose() * view;
	}

	void updateTransformation()
	{
		view =
			(Eigen::Translation3f(Vec3f(pan_horizontal, pan_vertical, -distance)) *
			 Eigen::AngleAxisf(-pitch, -Vec3f::UnitX()) *
			 Eigen::AngleAxisf(-yaw, -Vec3f::UnitY()) *
			 Eigen::Translation3f(-aim)).matrix();
		
		Vec4f getXZ =
			(Eigen::Translation3f(aim) *
			 Eigen::AngleAxisf(-pitch, -Vec3f::UnitX()) *
			 Eigen::AngleAxisf(-yaw, -Vec3f::UnitY()) *
			 Eigen::Translation3f(Vec3f(-pan_horizontal, -pan_vertical, -distance))).matrix() *
			Vec4f(0, 0, 0, 1);

		// to solve gimbal lock, a separate transformation is created
		// with only pitch movement
		Vec4f getY =
			Eigen::Translation3f(aim) *
			Eigen::AngleAxisf(-pitch, -Vec3f::UnitX()) *
			Eigen::Translation3f(Vec3f(-pan_horizontal, -pan_vertical, -distance)) *
			Vec4f(0, 0, 0, 1);
		
		eye = Vec3f(getXZ[0], getY[1], -getXZ[2]);		
	}   

	Vec3f eye;
    Mat4f view;
    float fov;
    float near_plane;
    float far_plane;
    float aspect_ratio;
    Mat4f projection;

	float pitch = 1.59;
    float yaw = 3.07;
    float distance;
    float pan_horizontal;
    float pan_vertical;
    Vec3f aim;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef std::shared_ptr<CorGuiCamera> CorGuiCameraPtr;

struct CorGuiGeometry {
	virtual void render() = 0;
	Eigen::Matrix4f model_matrix = Eigen::Matrix4f::Identity();
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
typedef std::shared_ptr<CorGuiGeometry> CorGuiGeometryPtr;

struct CorGuiView {

	CorGuiView(const std::string& n, int w, int h) :
		name(n), width(w), height(h) {}
		
	std::string name;
	CorGuiCameraPtr camera;
	int width;
	int height;
	GLuint view_texture_handle;
	GLuint depth_buffer_handle;
	GLuint frame_buffer_handle;
	
	std::map<std::string, CorGuiGeometryPtr> geometry;
	std::map<std::string, CorGuiProgramPtr> shaders;
	
	CorGuiProgramPtr current_shader;
};

typedef std::shared_ptr<CorGuiView> CorGuiViewPtr;

struct CorGuiState {
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		
};
typedef std::shared_ptr<CorGuiState> CorGuiStatePtr;

struct CorGuiContext {
	std::map<std::string, CorGuiViewPtr> views;
	CorGuiViewPtr current_view;
	GLFWwindow* window;
	std::stack<CorGuiStatePtr> state_stack;
	CorGuiStatePtr current_state;
};

#ifndef gCorGuiCtx
CorGuiContextPtr gCorGuiCtx;
#endif


struct CorGuiTexture {
	
};


struct CorGuiVector : public CorGuiGeometry {

};

namespace {

const float FILM_DIAGONAL = 43.266615300557f;
const float PI = 3.14159265358979f;

template <typename T>
struct PackedVertex {
	T position;
	T normal;
	bool operator<(const PackedVertex that) const {
		return memcmp((void*)this, (void*)&that, sizeof(PackedVertex))>0;
	};
};

template <typename T>
bool getSimilarVertexIndex(PackedVertex<T> & packed,
						   std::map<PackedVertex<T>,
						   unsigned int> & vertexToOutIndex,
						   unsigned int & result	) {

	typename std::map<PackedVertex<T>,unsigned int>::iterator it = vertexToOutIndex.find(packed);
	if ( it == vertexToOutIndex.end() ){
		return false;
	}else{
		result = it->second;
		return true;
	}
}	
	
template <typename T>
void indexVBO(const std::vector<T> & in_vertices,
			  const std::vector<T> & in_normals,
			  std::vector<unsigned int> & out_indices,
			  std::vector<T> & out_vertices,
			  std::vector<T> & out_normals) {

	std::map<PackedVertex<T> ,unsigned int> vertexToOutIndex;

	// For each input vertex
	for ( unsigned int i=0; i<in_vertices.size(); i++ ){

		PackedVertex<T> packed = {in_vertices[i], in_normals[i]};
		// Try to find a similar vertex in out_XXXX
		unsigned int index;
		bool found = getSimilarVertexIndex<T>( packed, vertexToOutIndex, index);

		if ( found ){ // A similar vertex is already in the VBO, use it instead !
			out_indices.push_back( index );
		}else{ // If not, it needs to be added in the output data.
			out_vertices.push_back( in_vertices[i]);
			out_normals .push_back( in_normals[i]);
			unsigned int newindex = (unsigned int)out_vertices.size() - 1;
			out_indices .push_back( newindex );
			vertexToOutIndex[ packed ] = newindex;
		}
	}
}
	
}

struct CorGuiPointCloud : public CorGuiGeometry {

	static const int MAX_SIZE = 360000;
	
	CorGuiPointCloud(const float *_data, int _npoints) : data(_data), npoints(_npoints) {
		glGenBuffers(1, &vertex_buffer);
		glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
		glBufferData(GL_ARRAY_BUFFER, MAX_SIZE * 3 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
	}
	
	void render() {

		if (data && npoints > 0) {		
			glEnableVertexAttribArray(0);
			glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
			glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * 3 * npoints, data);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float) * 3, nullptr);
	   		
			glDrawArrays(GL_POINTS, 0, npoints);
			glDisableVertexAttribArray(0);
		}
	}

	GLuint vertex_buffer = 0;
	const float *data;
	int npoints = 0;
};
typedef std::shared_ptr<CorGuiPointCloud> CorGuiPointCloudPtr;

struct CorGuiMesh : public CorGuiGeometry {

	CorGuiMesh() {
		initBuffers();
	}
	
	void initBuffers() {
		glGenBuffers(1, &vertex_buffer);
		glGenBuffers(1, &normal_buffer);
		glGenBuffers(1, &index_buffer);
	}
	
	template <typename T>
	void generate(const std::vector<T>& vbuffer,
				  const std::vector<T>& nbuffer,
				  const std::vector<unsigned int>& indices) {

		num_triangles = indices.size();

		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);
	
		glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
		glBufferData(GL_ARRAY_BUFFER, vbuffer.size() * sizeof(T), vbuffer.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

		glBindBuffer(GL_ARRAY_BUFFER, normal_buffer);
		glBufferData(GL_ARRAY_BUFFER, nbuffer.size() * sizeof(T), nbuffer.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

		// Generate a buffer for the indices as well
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data() , GL_STATIC_DRAW);

		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, 0);	
		
	}

	template <typename T>
	void generate(const std::vector<T>& vbuffer,
				  const std::vector<T>& nbuffer) {

		// create the buffers and bind the data to them
		std::vector<unsigned int> indices;
		std::vector<T> indexed_vertices;
		std::vector<T> indexed_normals;
		indexVBO<T>(vbuffer, nbuffer, indices, indexed_vertices, indexed_normals);
		generate(indexed_vertices, indexed_normals, indices);
	}

	
	template <typename T>
	void updateMesh(const std::vector<T>& vbuffer,
					const std::vector<T>& nbuffer) {
		glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
		glBufferSubData(GL_ARRAY_BUFFER, 0, vbuffer.size() * sizeof(T), vbuffer.data());		
		glBindBuffer(GL_ARRAY_BUFFER, normal_buffer);
		glBufferSubData(GL_ARRAY_BUFFER, 0, nbuffer.size() * sizeof(T), nbuffer.data());
		glBindBuffer(GL_ARRAY_BUFFER, 0);	
	}

	int num_triangles = 0;
	GLuint vertex_buffer = 0;
	GLuint normal_buffer = 0;
	GLuint index_buffer = 0;
	
};
	
struct CorGuiSphere : public CorGuiMesh {

	CorGuiSphere(float radius, int resolution, const Vec3f& c) : CorGuiMesh() {
			
		color = color;
		std::vector<Vec3f> vertex_buffer;
		std::vector<Vec3f> normal_buffer;

		for(int w = 0; w < resolution; w++) {
			for(int h = (-resolution/2); h < (resolution/2); h++){

				float inc1 = (w/(float)resolution)*2*PI;
				float inc2 = ((w+1)/(float)resolution)*2*PI;
				float inc3 = (h/(float)resolution)*PI;
				float inc4 = ((h+1)/(float)resolution)*PI;
				float X1 = sin(inc1);
				float Y1 = cos(inc1);
				float X2 = sin(inc2);
				float Y2 = cos(inc2);
				float radius1 = radius*cos(inc3);
				float radius2 = radius*cos(inc4);
				float Z1 = radius*sin(inc3);
				float Z2 = radius*sin(inc4);

				// insert the triangle coordinates
				vertex_buffer.push_back(Vec3f(radius1*X1,Z1,radius1*Y1));
				vertex_buffer.push_back(Vec3f(radius1*X2,Z1,radius1*Y2));
				vertex_buffer.push_back(Vec3f(radius2*X2,Z2,radius2*Y2));

				vertex_buffer.push_back(Vec3f(radius1*X1,Z1,radius1*Y1));
				vertex_buffer.push_back(Vec3f(radius2*X2,Z2,radius2*Y2));
				vertex_buffer.push_back(Vec3f(radius2*X1,Z2,radius2*Y1));

				// insert the normal data
				normal_buffer.push_back(Vec3f(X1,Z1,Y1).normalized());
				normal_buffer.push_back(Vec3f(X2,Z1,Y2).normalized());
				normal_buffer.push_back(Vec3f(X2,Z2,Y2).normalized());
				normal_buffer.push_back(Vec3f(X1,Z1,Y1).normalized());
				normal_buffer.push_back(Vec3f(X2,Z2,Y2).normalized());
				normal_buffer.push_back(Vec3f(X1,Z2,Y1).normalized());
			}
		}

		generate<Vec3f>(vertex_buffer, normal_buffer);
	}

	void render() {		
		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);

		glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

		glBindBuffer(GL_ARRAY_BUFFER, normal_buffer);	
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
	
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer);
		glDrawElements(GL_TRIANGLES, num_triangles, GL_UNSIGNED_INT, nullptr);
		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	Vec3f color;
	
};


void SetCurrentContext(CorGuiContextPtr ctx) {
	gCorGuiCtx = ctx;
}

void InitializeContext(CorGuiContextPtr ctx) {
	
	ctx->state_stack = std::stack<CorGuiStatePtr>();

	// push first state to the context
	ctx->state_stack.push(CorGuiStatePtr(new CorGuiState()));
	ctx->current_state = ctx->state_stack.top();
	
}

void PreRender(CorGuiViewPtr view) {

	CorGuiContext& ctx = *gCorGuiCtx;
	ctx.current_view->camera->processImguiInput();
	
	glBindFramebuffer(GL_FRAMEBUFFER, view->frame_buffer_handle);
	glViewport(0, 0, view->width, view->height);

	// Clear the screen
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);	// Dark Background
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	// Enable depth test
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	
	// Cull triangles which normal is not towards the camera
	glEnable(GL_CULL_FACE);
}

void PostRender(CorGuiViewPtr view) {

	glUseProgram(0);

	// unbind frame buffer
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	static int borderx = 15;
	static int bordery = 35;

	// paint texture as Imgui image
	ImVec2 pos = ImGui::GetCursorScreenPos();
	ImVec2 max_pos = ImVec2(pos.x + view->width - borderx,
							pos.y + view->height - bordery);

	ImGui::GetWindowDrawList()->AddImage(
		(void*)(uintptr_t)view->view_texture_handle,
		ImVec2(pos.x, pos.y),
		ImVec2(max_pos),
		ImVec2(0, 1), ImVec2(1, 0)
	);
}

static void error_callback(int error, const char* description)
{
    fprintf(stderr, "Error %d: %s\n", error, description);
}

int CorGui::CreateWindow(const std::string& name, int w, int h) {

	CorGuiContext& ctx = *gCorGuiCtx;

    glfwSetErrorCallback(error_callback);
    if (!glfwInit())
        return 1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#if __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
    ctx.window = glfwCreateWindow(w, h, name.c_str(), NULL, NULL);
    glfwMakeContextCurrent(ctx.window);
    glfwSwapInterval(1); // Enable vsync
    gl3wInit();

	ImGui_ImplGlfwGL3_Init(ctx.window, true);
	return 0;
}

CorGuiContextPtr CorGui::CreateContext() {
	
	CorGuiContextPtr ctx = std::shared_ptr<CorGuiContext>(new CorGuiContext());
	SetCurrentContext(ctx);
	InitializeContext(ctx);
	return ctx;
}

void FreeBuffers(CorGuiContext& ctx) {

}

CorGuiViewPtr FindViewByName(const std::string& name) {
	CorGuiContext& ctx = *gCorGuiCtx;

	if (ctx.views.count(name)) {
		return ctx.views[name];
	}
	return nullptr;
}

void InitializeView(CorGuiViewPtr view) {

	glGenFramebuffers(1, &view->frame_buffer_handle);
	glGenTextures(1, &view->view_texture_handle);
	glGenRenderbuffers(1, &view->depth_buffer_handle);
	glBindFramebuffer(GL_FRAMEBUFFER, view->frame_buffer_handle);
	glBindTexture(GL_TEXTURE_2D, view->view_texture_handle);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, view->width, view->height, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);

	// Poor filtering. Needed !
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	glBindRenderbuffer(GL_RENDERBUFFER, view->depth_buffer_handle);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, view->width, view->height);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
		GL_RENDERBUFFER, view->depth_buffer_handle);
	// Set "view_texture_handle" as our colour attachement #0
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, view->view_texture_handle, 0);

	// Set the list of draw buffers.
	GLenum DrawBuffers[1] = { GL_COLOR_ATTACHMENT0 };
	glDrawBuffers(1, DrawBuffers); // "1" is the size of DrawBuffers

   // Always check that our framebuffer is ok
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
		// LOG "Something went wrong");
		std::cout << "Something went wrong" << std::endl;
		assert(false);
	}

	view->shaders["default"] = CorGuiProgramPtr(new CorGuiProgram());
	view->current_shader = view->shaders["default"];
	
	view->camera = CorGuiCameraPtr(new CorGuiCamera(45.0f, 0.01f, 50.0f, 1.34f));
		
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	// Bind a vertex array. Not sure exactly why we need to bind this.
	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);
}

CorGuiViewPtr CreateView(const std::string& name, CorGuiWindowFlags flags) {

	CorGuiContext& ctx = *gCorGuiCtx;

	if (ctx.views.count(name)) {
		// LOG view already exists
		return ctx.views[name];
	}
	
	CorGuiViewPtr view = std::shared_ptr<CorGuiView>(
		new CorGuiView(name, flags.width, flags.height));
	
	ctx.views[name] = view;
	InitializeView(view);

	return view;
}

bool CorGui::Begin(const std::string& name, CorGuiWindowFlags flags) {

	ImGui::Begin(name.c_str());
	CorGuiContext& ctx = *gCorGuiCtx;
	CorGuiViewPtr view = FindViewByName(name);
	if (!view) {
		view = CreateView(name, flags);
	}

	ctx.current_view = view;
	
	PreRender(view);
	return true;
}

bool CorGui::End() {
	CorGuiContext& ctx = *gCorGuiCtx;
	PostRender(ctx.current_view);
	ImGui::End();		
	return true;
}

void CorGui::ShutDown() {
	CorGuiContext& ctx = *gCorGuiCtx;
	ImGui_ImplGlfwGL3_Shutdown();
	FreeBuffers(ctx);
    ImGui::DestroyContext();
    glfwTerminate();
}

bool CorGui::ShouldClose() {
	CorGuiContext& ctx = *gCorGuiCtx;
	return glfwWindowShouldClose(ctx.window);
}

void CorGui::BeginFrame() {
	glfwPollEvents();
	ImGui_ImplGlfwGL3_NewFrame();
}

void CorGui::EndFrame() {
	// Rendering
	CorGuiContext& ctx = *gCorGuiCtx;
	static ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
	int display_w, display_h;
	glfwGetFramebufferSize(ctx.window, &display_w, &display_h);
	glViewport(0, 0, display_w, display_h);
	glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
	glClear(GL_COLOR_BUFFER_BIT);
	ImGui::Render();
	ImGui_ImplGlfwGL3_RenderDrawData(ImGui::GetDrawData());
	glfwSwapBuffers(ctx.window);
}

CorGuiGeometryPtr FindGeometryByName(const std::string& name) {
	CorGuiContext& ctx = *gCorGuiCtx;
	if (ctx.current_view->geometry.count(name)) {
		return ctx.current_view->geometry[name];
	}
	return nullptr;
}

CorGuiGeometryPtr CreateSphere(const std::string& name, float radius) {
	CorGuiContext& ctx = *gCorGuiCtx;
	CorGuiGeometryPtr sphere = CorGuiGeometryPtr(new CorGuiSphere(radius, 20, Vec3f(1.0f,0.0f,1.0f)));
	ctx.current_view->geometry[name] = sphere;
	return sphere;
}

CorGuiGeometryPtr CreatePointCloud(const std::string& name, const float* data, int npoints) {
	CorGuiContext& ctx = *gCorGuiCtx;
	CorGuiGeometryPtr cloud = CorGuiGeometryPtr(new CorGuiPointCloud(data, npoints));
	ctx.current_view->geometry[name] = cloud;
	return cloud;
}

namespace {
	void SetUniform_(GLuint uniform, bool, const float& object) {
		glUniform1fv(uniform, 1, &object);
	}
	void SetUniform_(GLuint uniform, bool, const Eigen::Vector2f& object) {
		glUniform2fv(uniform, 1, object.data());
	}
	void SetUniform_(GLuint uniform, bool, const Eigen::Vector3f& object) {
		glUniform3fv(uniform, 1, object.data());
	}
	void SetUniform_(GLuint uniform, bool transpose, const Eigen::Matrix4f& object) {
		glUniformMatrix4fv(uniform, 1, transpose, object.data());
	}
}

template <typename T>
void SetUniform(const std::string& name, const T& object) {
	CorGuiContext& ctx = *gCorGuiCtx;
	GLuint uniform = glGetUniformLocation(ctx.current_view->current_shader->id, name.c_str());
	SetUniform_(uniform, false, object);
}

void RenderGeometry(CorGuiGeometryPtr geometry) {

	CorGuiContext& ctx = *gCorGuiCtx;
	CorGuiStatePtr state = ctx.current_state;

	// use the shader from the state instead!
	glUseProgram(ctx.current_view->current_shader->id);

	
	Vec3f color = Vec3f(1.0, 1.0, 0.0);
	Mat4f mvp = (ctx.current_view->camera->getViewProjectionMatrix() *
				 state->transform); //	geometry->model_matrix 
	
	SetUniform<Mat4f>("mvp", mvp);
	SetUniform<Mat4f>("m", state->transform); //geometry->model_matrix);
	SetUniform<Mat4f>("v", ctx.current_view->camera->view);
	SetUniform<Vec3f>("lightpos_worldspace", Vec3f(0,1,0));
	SetUniform<Vec3f>("fragment_color", color);

	geometry->render();	
}

void CorGui::Sphere(const std::string& name, float radius) {
	CorGuiGeometryPtr sphere = FindGeometryByName(name);
	if (!sphere) {
		sphere = CreateSphere(name, radius);
	}	
	RenderGeometry(sphere);	
}

void CorGui::PointCloud(const std::string&name, const float* points, int npoints) {
	CorGuiGeometryPtr cloud = FindGeometryByName(name);
	if (!cloud) {
		cloud = CreatePointCloud(name, points, npoints);
	}	
	RenderGeometry(cloud);	
}


void CorGui::PushState() {
	CorGuiContext& ctx = *gCorGuiCtx;
	ctx.state_stack.push(CorGuiStatePtr(new CorGuiState()));
	ctx.current_state = ctx.state_stack.top();	
}

void CorGui::PopState() {
	CorGuiContext& ctx = *gCorGuiCtx;
	if (ctx.state_stack.size() == 1) return;
	
	ctx.state_stack.pop();
	ctx.current_state = ctx.state_stack.top();
}
	
// Transforms
void CorGui::Translate(float x, float y, float z) {
	CorGuiContext& ctx = *gCorGuiCtx;
	CorGuiStatePtr state = ctx.current_state;
	state->transform.col(3).head(3) = Eigen::Vector3f(x,y,z);	
	
}
