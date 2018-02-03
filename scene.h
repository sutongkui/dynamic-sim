#pragma once

#define GLEW_STATIC

#include <vector>
#include <armadillo>

#include <cuda_runtime.h>

#include "Simulator.h"
#include "GLSLShader.h"
#include "VAOMesh.h"

class BVHAccel;


using namespace arma;
class DoudouHead_Merge
{
public:
	static GLfloat model[16];
	static bool Enabled;
private:
	mat Pts_Head;
	mat Pts_Head_Cor;
	mat Pts_Body;
	mat Pts_Body_Cor;
	mat Ones;
	uword Len;
public:
	DoudouHead_Merge();
	void Init(Mesh& _Pts_Head_obj);
	DoudouHead_Merge(Mesh& _Pts_Head_obj);
	GLfloat *Calc(Vec4s& _Pts_Body_vec);
};

//singleton
class Scene
{
public:
	// 单子模式
	static Scene &instance();
	~Scene() { }  

	void initialize(int argc, char **argv); 
		
	//add objects,bind VAOs 
	void add(VAOMesh& mesh);   

	void update_simulating_cloth(Cloth &cloth);
	void update_simulating_body(Mesh &body);

	void initiate_body_template(const VAOMesh &body)
	{
		template_body = body; 
	}

	void render();

	void update_body_data();
private:
	Scene() { }  

	inline void check_GL_error();
	void loadShader();

private:
	void screenshot();
	void DrawGrid();                  // OPENGL场景的各种函数
	void RenderGPU_CUDA();


	static void onRender();
	static void OnReshape(int nw, int nh);
	static void OnIdle();
	static void OnMouseMove(int x, int y);
	static void OnMouseDown(int button, int s, int x, int y);
	static void OnKey(unsigned char key, int, int);
	static void OnShutdown();

private:
	VAOMesh template_body;

	GLSLShader renderShader;
	enum attributes { position, texture, normal };

	void RenderBuffer(VAO_Buffer vao_buffer);
	vector<VAO_Buffer> obj_vaos;

	Simulator simulator;

	// 指向OPENGL buffer中vertex的地址
	glm::vec4 *body_p_vertex;       
	// 指向OPENGL buffer中normal的地址
	glm::vec3 *body_p_normal;           

	cudaGraphicsResource *body_vbo_resource;

	// OPENGL场景的各种参数declaration
	static int oldX, oldY;    
	static float rX, rY;
	static int state;
	static float dist, dy;
	static GLint viewport[4];
	static GLfloat view[16];
	static GLfloat model[16];
	static GLfloat projection[16];
	static glm::vec3 Up, Right, viewDir;
	static int selected_index;
	static const int width = 1024, height = 1024;

};



