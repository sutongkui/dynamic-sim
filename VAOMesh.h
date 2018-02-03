#pragma once
#include <GL/glew.h>
#include <GL/freeglut.h>

#include "Mesh.h"

struct VAO_Buffer
{
	GLuint vao;
	GLuint array_buffer;
	GLuint index_buffer;
	GLuint texture;
	GLuint index_size;
};

class VAOMesh : public Mesh
{
public:
	VAO_Buffer vbo;
};


