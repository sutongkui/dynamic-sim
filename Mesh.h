#pragma once

#pragma once
#include <vector>
#include <string>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <glm/glm.hpp>

#include "Common.h"

class Mesh
{
public:
#if 0
	typedef std::vector<std::vector<unsigned int>> V2FIndices;
	typedef std::vector<std::vector<unsigned int>> V2VIndices;

	struct Indices
	{
		F2VIndices f2v;
		F2TIndices f2t;
		F2NIndices f2n;
	};
#endif

	struct Face
	{
		unsigned int v0;
		unsigned int v1;
		unsigned int v2;
	};
	typedef std::vector<Face> Faces;

	typedef std::vector<std::pair<std::string, unsigned int>> MeshObjects;
	typedef std::vector<std::pair<std::string, unsigned int>> FaceGroups;

public:
	void clear(); 
	void scale(float s);
	void translate(float x, float y, float z);
	void rotation(float x, float y, float z);

	//沿着NORMAL方向扩展点，不同于SCALE
	void extend(float dist);
	glm::vec3 get_center();

	void get_euler_coordinates(Vec3s &result) const;
	
	// 创建顶点-三角形邻接矩阵
	void get_vertex_adjface_matrix(
		std::vector<unsigned int> &vertex_adjface, 
		unsigned int maxneighbor, 
		unsigned int pad
	) const;

public:
	Vec4s vertices;
	Vec3s normals;
	Vec2s texures;

	Faces faces; 

	// for vertices region division 
	MeshObjects objects;  
	FaceGroups  fgroups;

	GLuint gl_texture;

	friend class ObjLoader;
};


