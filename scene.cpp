#include <queue>
#include <iostream>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <FreeImage.h>

#include <cuda_gl_interop.h>

#define GLEW_STATIC

#include "wglew.h"
#include "scene.h"
#include "VAOMesh.h"
#include "ObjLoader.h"

using namespace std;

// OPENGL场景的各种参数declaration

int Scene::oldX = 0, Scene::oldY = 0;
float Scene::rX = 15, Scene::rY = 0;
int Scene::state = 1;
float Scene::dist = -15;
float Scene::dy = 0;
GLint Scene::viewport[4];
GLfloat Scene::model[16];
GLfloat Scene::view[16];
GLfloat Scene::projection[16];
glm::vec3 Scene::Up = glm::vec3(0, 1, 0),
Scene::Right = glm::vec3(0, 0, 0),
Scene::viewDir = glm::vec3(0, 0, 0);
int Scene::selected_index = -1;
static int current_width;
static int current_height;

extern int StartNum;
extern int StopNum;
int Iter = 0;
int MaxIteration = 5000;
int pose = 0;
int poser = StopNum - StartNum;

static int num_screenshot = 0;
GLenum GL_MODE = GL_LINE_LOOP;

extern queue<Mesh> BodyQueue;
extern queue<Vec4s> VQueue;
extern queue<Vec3s> NQueue;

extern HANDLE hMutex;


extern DoudouHead_Merge DoudouHead_Solver;

Scene &Scene::instance()
{
	static Scene scene;
	return scene;
}

void Scene::initialize(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(width, height);
	glutCreateWindow("ViSG_RealTime_DCSim");

	GLenum err = glewInit();
	if (err != GLEW_OK) {
		fprintf(stderr, "%s\n", glewGetErrorString(err));
		return;
	}
	wglSwapIntervalEXT(0);  // disable Vertical synchronization
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
}

void Scene::render()
{
	loadShader(); //InitGL(); //load shader

	glutDisplayFunc(onRender);
	glutReshapeFunc(OnReshape);
	glutIdleFunc(OnIdle);

	glutMouseFunc(OnMouseDown);
	glutMotionFunc(OnMouseMove);
	glutKeyboardFunc(OnKey);
	glutCloseFunc(OnShutdown);

	glutMainLoop();
}

void Scene::RenderBuffer(VAO_Buffer vao_buffer)
{
	GLfloat eyeDir[3] = { viewDir.x,viewDir.y,viewDir.z };

	renderShader.Use();
	glUniformMatrix4fv(renderShader("view"), 1, GL_FALSE, view);
	if (DoudouHead_Merge::Enabled)
	{
		glUniformMatrix4fv(renderShader("model"), 1, GL_FALSE, DoudouHead_Merge::model);
		DoudouHead_Merge::Enabled = false;
		//for (int i = 0; i < 16; i++)cout << DoudouHead_Merge::modelview[i] << ' '; cout << endl;
	}
	else
	{
		glUniformMatrix4fv(renderShader("model"), 1, GL_FALSE, model);   // the platform does not support "glUniformMatrix4dv"
	}
	glUniformMatrix4fv(renderShader("projection"), 1, GL_FALSE, projection);
	glUniform3fv(renderShader("viewPos"), 1, eyeDir);

	//glPointSize(1);
	glBindVertexArray(vao_buffer.vao);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vao_buffer.index_buffer);
	glBindTexture(GL_TEXTURE_2D, vao_buffer.texture);
	glDrawElements(GL_TRIANGLES, (GLsizei)vao_buffer.index_size, GL_UNSIGNED_INT, 0);
	glBindTexture(GL_TEXTURE_2D, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	renderShader.UnUse();
}

void Scene::add(VAOMesh& mesh)
{
	//add VAOs and Buffers
	VAO_Buffer tem_vao;

	glGenVertexArrays(1, &tem_vao.vao);
	glGenBuffers(1, &tem_vao.array_buffer);
	glGenBuffers(1, &tem_vao.index_buffer);
	tem_vao.texture = mesh.gl_texture;
	tem_vao.index_size = mesh.faces.size() * 3;
	check_GL_error();

	glBindVertexArray(tem_vao.vao);
	glBindBuffer(GL_ARRAY_BUFFER, tem_vao.array_buffer);

	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec4) * mesh.vertices.size() + sizeof(glm::vec2) * mesh.texures.size() + sizeof(glm::vec3)*mesh.normals.size(), NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(glm::vec4) * mesh.vertices.size(), &mesh.vertices[0]);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(glm::vec4) * mesh.vertices.size(), sizeof(glm::vec2) * mesh.texures.size(), &mesh.texures[0]);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(glm::vec4) * mesh.vertices.size() + sizeof(glm::vec2) * mesh.texures.size(), sizeof(glm::vec3)*mesh.normals.size(), &mesh.normals[0]);
	check_GL_error();

	glVertexAttribPointer(position, 4, GL_FLOAT, GL_FALSE, sizeof(glm::vec4), 0);
	glVertexAttribPointer(texture, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (const GLvoid*)(sizeof(glm::vec4)*mesh.vertices.size()));
	glVertexAttribPointer(normal, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (const GLvoid*)(sizeof(glm::vec4)*mesh.vertices.size() + sizeof(glm::vec2)*mesh.texures.size()));

	glEnableVertexAttribArray(position);
	glEnableVertexAttribArray(texture);
	glEnableVertexAttribArray(normal);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, tem_vao.index_buffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * mesh.faces.size() * 3, &mesh.faces[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glBindVertexArray(0);

	obj_vaos.push_back(tem_vao); // add new vao to the scene
	mesh.vbo = tem_vao;
}

void Scene::update_simulating_cloth(Cloth &cloth)
{
	simulator.update_cloth(cloth);
}

void Scene::update_simulating_body(Mesh &body)
{
	simulator.update_body(body);
}

void Scene::check_GL_error()
{
	assert(glGetError() == GL_NO_ERROR);
}

void Scene::loadShader()
{
	//set light
	GLfloat lightPos[3] = { 0, 0.0f, 10.0f };
	GLfloat lightColor[3] = { 0.8f, 0.8f, 0.8f };
	GLfloat objectColor[3] = { 0.8f, 0.8f, 0.8f };

	renderShader.LoadFromFile(GL_VERTEX_SHADER, "shaders/render.vert");
	renderShader.LoadFromFile(GL_FRAGMENT_SHADER, "shaders/render.frag");
	renderShader.CreateAndLinkProgram();

	renderShader.Use();
	renderShader.AddUniform("color");
	renderShader.AddUniform("model");
	renderShader.AddUniform("view");
	renderShader.AddUniform("projection");
	renderShader.AddUniform("lightPos");
	glUniform3fv(renderShader("lightPos"), 1, lightPos);
	renderShader.AddUniform("viewPos");
	renderShader.AddUniform("lightColor");
	glUniform3fv(renderShader("lightColor"), 1, lightColor);
	renderShader.AddUniform("objectColor");
	glUniform3fv(renderShader("objectColor"), 1, objectColor);
	renderShader.UnUse();

	check_GL_error();
	glEnable(GL_DEPTH_TEST);
}

void Scene::screenshot()
{
	// Make the BYTE array, factor of 3 because it's RBG.
	BYTE *pixels = new BYTE[3 * current_width * current_height];

	glReadPixels(0, 0, current_width, current_height, GL_BGR, GL_UNSIGNED_BYTE, pixels);

	// Convert to FreeImage format & save to file
	FIBITMAP *image = FreeImage_ConvertFromRawBits(pixels, current_width, current_height, 3 * current_width, 24, 0x0000FF, 0xFF0000, 0x00FF00, false);
	string str = "../screenshot/screenshot";
	str += to_string(num_screenshot++);
	str += ".bmp";

	FreeImage_Save(FIF_BMP, image, str.c_str(), 0);

	// Free resources
	FreeImage_Unload(image);
	delete[] pixels;
	cout << str << " saved successfully!" << endl;
}

// OPENGL场景的各种函数
void Scene::DrawGrid()
{
	const int GRID_SIZE = 10;
	glBegin(GL_LINES);
	glColor3f(0.5f, 0.5f, 0.5f);
	for (int i = -GRID_SIZE; i <= GRID_SIZE; i++)
	{
		glVertex3f((float)i, -2, (float)-GRID_SIZE);
		glVertex3f((float)i, -2, (float)GRID_SIZE);

		glVertex3f((float)-GRID_SIZE, -2, (float)i);
		glVertex3f((float)GRID_SIZE, -2, (float)i);
	}

	glEnd();

}


void Scene::RenderGPU_CUDA()
{
	if (Iter > MaxIteration)
	{
		if (VQueue.size() > 1)
		{
			//WaitForSingleObject(hMutex, INFINITE);
			update_body_data();
			//ReleaseMutex(hMutex);
			MaxIteration = 25;
			Iter = 0;
		}
	}

	simulator.simulate();
	simulator.visulize();

	for (int i = 0; i < obj_vaos.size() - 1; ++i)
	{
		auto vao = obj_vaos[i];
		RenderBuffer(vao);

	}
	DoudouHead_Merge::Enabled = true;
	RenderBuffer(obj_vaos.back());

	Iter++;

}
void Scene::onRender()
{
	getFPS();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	glTranslatef(0, dy, 0);
	glTranslatef(0, 0, dist);
	glRotatef(rX, 1, 0, 0);
	glRotatef(rY, 0, 1, 0);

	for (int i = 0; i < 16; ++i)
		if (i == 0 || i == 5 || i == 10 || i == 15)
			model[i] = 1;
		else
			model[i] = 0;
	glGetFloatv(GL_MODELVIEW_MATRIX, view);
	glGetFloatv(GL_PROJECTION_MATRIX, projection);
	viewDir.x = (float)-view[2];
	viewDir.y = (float)-view[6];
	viewDir.z = (float)-view[10];
	Right = glm::cross(viewDir, Up);

	//画出包围盒，AABB TREE
	//if (Scene::instance().h_bvh)
	//{
	//	Scene::instance().h_bvh->draw(Scene::instance().h_bvh->get_root());
	//}
	//画出构建的两级弹簧
	//Scene::instance().simulation->cuda_spring->draw();

	//debug,画出检测到碰撞的点
	//Scene::instance().simulation->draw_collided_vertex();

	Scene::instance().RenderGPU_CUDA();

	glutSwapBuffers();
}
void Scene::OnReshape(int nw, int nh)
{
	current_width = nw;
	current_height = nh;
	glViewport(0, 0, nw, nh);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(30, (GLfloat)nw / (GLfloat)nh, 0.1f, 100.0f);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glutPostRedisplay();
}

void Scene::OnIdle()
{
	glutPostRedisplay();
}

void Scene::OnMouseMove(int x, int y)
{
	if (selected_index == -1) {
		if (state == 0)
			dist *= (1 + (y - oldY) / 60.0f);
		else
		{
			rY += (x - oldX) / 5.0f;
			rX += (y - oldY) / 5.0f;
		}
	}
	else {
		float delta = 1500 / abs(dist);
		float valX = (x - oldX) / delta;
		float valY = (oldY - y) / delta;
		if (abs(valX) > abs(valY))
			glutSetCursor(GLUT_CURSOR_LEFT_RIGHT);
		else
			glutSetCursor(GLUT_CURSOR_UP_DOWN);


		glm::vec4 *ptr = (glm::vec4*)glMapBuffer(GL_ARRAY_BUFFER, GL_READ_ONLY);
		glm::vec4 oldVal = ptr[selected_index];
		glUnmapBuffer(GL_ARRAY_BUFFER); // unmap it after use

		glm::vec4 newVal;
		newVal.w = 1;
		// if the pointer is valid(mapped), update VBO
		if (ptr) {
			// modify buffer data				
			oldVal.x += Right[0] * valX;

			float newValue = oldVal.y + Up[1] * valY;
			if (newValue > 0)
				oldVal.y = newValue;
			oldVal.z += Right[2] * valX + Up[2] * valY;
			newVal = oldVal;
		}

	}
	oldX = x;
	oldY = y;

	glutPostRedisplay();
}

void Scene::OnMouseDown(int button, int s, int x, int y)
{
	if (s == GLUT_DOWN)
	{
		oldX = x;
		oldY = y;
		int window_y = (height - y);
		float norm_y = float(window_y) / float(height / 2.0);
		int window_x = x;
		float norm_x = float(window_x) / float(width / 2.0);

		float winZ = 0;
		glReadPixels(x, height - y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);
		if (winZ == 1)
			winZ = 0;
		double objX = 0, objY = 0, objZ = 0;
		GLdouble MV1[16], P1[16];
		gluUnProject(window_x, window_y, winZ, MV1, P1, viewport, &objX, &objY, &objZ);
		glm::vec3 pt(objX, objY, objZ);
		int i = 0;

	}

	if (button == GLUT_MIDDLE_BUTTON)
		state = 0;
	else
		state = 1;

	if (s == GLUT_UP) {
		selected_index = -1;
		glutSetCursor(GLUT_CURSOR_INHERIT);
	}
}

void Scene::OnKey(unsigned char key, int, int)
{
	switch (key)
	{
	case 'w':
	case 'W':dy -= 0.1f; break;
	case 'S':
	case 's':dy += 0.1f; break;
	case 'x':
	case 'X':
		Scene::instance().screenshot(); 
		break;
	case 'M':
	case 'm':
		if (GL_MODE == GL_LINE_LOOP)
			GL_MODE = GL_TRIANGLES;
		else if (GL_MODE == GL_TRIANGLES)
			GL_MODE = GL_POINTS;
		else
			GL_MODE = GL_LINE_LOOP;
		break;
	default:
		break;
	}

	glutPostRedisplay();
}

void Scene::OnShutdown()
{

}

void Scene::update_body_data()
{
	//WaitForSingleObject(hMutex, INFINITE);

	VAOMesh now_body = template_body;
	now_body.vertices = VQueue.front();
	now_body.normals = NQueue.front();

	VQueue.pop();
	NQueue.pop();

	now_body.scale(0.3f);
	now_body.translate(0.0f, 0.6f, 0.0f);

	//////////////////保持有一只脚在地面
	//int index = (now_body.vertices[3367].y < now_body.vertices[6758].y) ? 3367 : 6758;
	//float dy = now_body.vertices[index].y - Scene::instance().template_body.vertices[index].y;
	//for (int i = 0;i < now_body.vertices.size();i++)
	//	now_body.vertices[i].y -= dy;
	////////////////////////////////////////////

	update_simulating_body(now_body);

	simulator.ccd();
	simulator.visulize();

	cudaError_t cudaStatus = cudaGraphicsGLRegisterBuffer(&body_vbo_resource, now_body.vbo.array_buffer, cudaGraphicsMapFlagsWriteDiscard);   	//register vbo


	if (cudaStatus != cudaSuccess)
		fprintf(stderr, "register failed\n");

	size_t num_bytes;
	cudaStatus = cudaGraphicsMapResources(1, &body_vbo_resource, 0);
	cudaStatus = cudaGraphicsResourceGetMappedPointer((void **)&body_p_vertex, &num_bytes, body_vbo_resource);
	body_p_normal = (glm::vec3*)((float*)body_p_vertex + 4 * this->template_body.vertices.size() + 2 * this->template_body.texures.size());   // 获取normal位置指针

	const size_t vertices_bytes = sizeof(glm::vec4)  * this->template_body.vertices.size();       //点的索引
	cudaStatus = cudaMemcpy(body_p_vertex, &now_body.vertices[0], vertices_bytes, cudaMemcpyHostToDevice);

	const size_t normal_bytes = sizeof(glm::vec3)  * this->template_body.normals.size();       //点的索引
	cudaStatus = cudaMemcpy(body_p_normal, &now_body.normals[0], normal_bytes, cudaMemcpyHostToDevice);
	cudaStatus = cudaGraphicsUnmapResources(1, &body_vbo_resource, 0);
	

	for (int i = 0; i < obj_vaos.size() - 1; ++i)
	{
		auto vao = obj_vaos[i];
		RenderBuffer(vao);
	}
	DoudouHead_Solver.Calc(now_body.vertices);
	RenderBuffer(obj_vaos.back());
}

