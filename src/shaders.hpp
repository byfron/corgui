#pragma once

namespace shaders {

std::string default_vs = R"(
#version 330 core

uniform mat4 mvp;
uniform mat4 m;
uniform mat4 v;
uniform vec3 lightpos_worldspace;

layout(location = 0) in vec3 vertex_pos; // vertex pos in model space
layout(location = 1) in vec3 normals;

out vec3 normal_cameraspace;
out vec3 eyedir_cameraspace;
out vec3 lightdir_cameraspace;
out vec3 vertpos_worldspace;

void main(){
// vertex in clip space for the fragment shader
  gl_Position = mvp * vec4(vertex_pos, 1);

// pos of vertex in world space
  vertpos_worldspace = (m * vec4(vertex_pos, 1)).xyz;

// Vector that goes from the vertex to the camera, in camera space.
  vec3 vertpos_cameraspace = (v * m * vec4(vertex_pos,1)).xyz;
  eyedir_cameraspace = vec3(0,0,0) - vertpos_cameraspace;

// Vector that goes from the vertex to the light, in camera space.
  vec3 lightpos_cameraspace = ( v * vec4(lightpos_worldspace,1)).xyz;
  lightdir_cameraspace = lightpos_cameraspace + eyedir_cameraspace;

// Normal of the the vertex, in camera space. Note. Modelmatrix should
// not scale the model or our normals will scale too
  normal_cameraspace = ( v * m * vec4(normals,0)).xyz; 

}
)";

std::string default_fs = R"(
#version 330 core

in vec3 normal_cameraspace;
in vec3 eyedir_cameraspace;
in vec3 lightdir_cameraspace;
in vec3 vertpos_worldspace;

out vec3 color;
uniform vec3 lightpos_worldspace;
uniform vec3 fragment_color;

void main(){

  // Light emission properties
  // You probably want to put them as uniforms
  vec3 lightColor = vec3(1,1,1);
  float lightPower = 5.0f;

  vec3 materialDiffuseColor = fragment_color;
  vec3 materialAmbientColor = vec3(0.1,0.1,0.1) * materialDiffuseColor;
  vec3 materialSpecularColor = vec3(0.3,0.3,0.3);

  // Distance to the light
  float distance = length( lightpos_worldspace - vertpos_worldspace);

  // Normal of the computed fragment, in camera space
  vec3 n = normalize( normal_cameraspace );
  // Direction of the light (from the fragment to the light)
  vec3 l = normalize( lightdir_cameraspace );
  // Cosine of the angle between the normal and the light direction, 
  // clamped above 0
  //  - light is at the vertical of the triangle -> 1
  //  - light is perpendicular to the triangle -> 0
  //  - light is behind the triangle -> 0
  float cosTheta = clamp( dot( n,l ), 0,1 );

  // Eye vector (towards the camera)
  vec3 E = normalize(eyedir_cameraspace);
  // Direction in which the triangle reflects the light
  vec3 R = reflect(-l,n);
  // Cosine of the angle between the Eye vector and the Reflect vector,
  // clamped to 0
  //  - Looking into the reflection -> 1
  //  - Looking elsewhere -> < 1
  float cosAlpha = clamp( dot( E,R ), 0,1 );

  color = 
		// Ambient : simulates indirect lighting
		materialAmbientColor +
		// Diffuse : color of the object
        materialDiffuseColor * lightColor * lightPower * cosTheta / (distance*distance) +
		// Specular : reflective highlight, like a mirror
		materialSpecularColor * lightColor * lightPower * pow(cosAlpha,5) / (distance*distance);

}
)";

std::string single_color_vs = R"(
#version 330 core
uniform mat4 mvp;
layout(location = 0) in vec3 vertex_pos;
void main(){
  gl_Position = mvp * vec4(vertex_pos, 1);
}
)";

std::string single_color_fs = R"(
#version 330 core
uniform vec3 fragment_color;
out vec3 color;
void main(){
  color = fragment_color;
}
)";
	
}
