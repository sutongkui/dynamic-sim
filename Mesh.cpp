#include "Mesh.h"

void Mesh::clear()
{
	vertices.clear();
	normals.clear();
	texures.clear();
	faces.clear();

	objects.clear();
	fgroups.clear();
}

// 和数学意义上的scale怎么是反的？
// 如果center不为0，减了不加回来？
void Mesh::scale(float s)
{
	//获取模型中心坐标
	glm::vec3 center = glm::vec3(0.0f, 0.0f, 0.0f);//get_center();
												//const float up = 1.2;
	for (int i = 0; i < vertices.size(); ++i)
	{
		vertices[i] -= glm::vec4(center, 0); 
		vertices[i].x /= s;
		vertices[i].y /= s;
		vertices[i].z /= s;
	}
}

void Mesh::translate(float x, float y, float z)
{
	//获取模型中心坐标
	for (int i = 0; i < vertices.size(); ++i)
	{
		vertices[i] += glm::vec4(x, y, z, 0); 
	}
}

void Mesh::rotation(float x, float y, float z)
{
	x = x / 180.0f * 3.1415f;
	y = y / 180.0f * 3.1415f;
	z = z / 180.0f * 3.1415f;

	glm::vec3 center = get_center();

	glm::mat4x4 Rx = {
		{ 1.0, 0.0, 0.0, 0.0 },
		{ 0.0, cos(x), -sin(x), 0.0 },
		{ 0.0, sin(x), cos(x), 0.0 },
		{ 0.0, 0.0, 0.0, 1.0 }
	};
	glm::mat4x4 Ry = {
		{ cos(y), 0, -sin(y), 0 },
		{ 0, 1.0, 0, 0 },
		{ sin(y), 0, cos(y), 0 },
		{ 0, 0, 0, 1.0 }
	};
	glm::mat4x4 Rz = {
		{ cos(z), -sin(z), 0, 0 },
		{ sin(z), cos(z), 0, 0 },
		{ 0, 0, 1, 0 },
		{ 0, 0, 0, 1 }
	};

	glm::mat4x4 R = Rx * Ry * Rz;

	for (auto &vertex : vertices)
	{
		vertex -= glm::vec4(center, 0.0);
		vertex = R * vertex;
		vertex += glm::vec4(center, 0.0);
	}
}

void Mesh::extend(float dist)
{
	for (int i = 0; i < vertices.size(); ++i)
	{
		glm::vec3 n = normals[i];
		vertices[i] += dist * glm::vec4(n, 0);
	}
}

glm::vec3 Mesh::get_center()
{
	glm::vec3 center;
	for (int i = 0; i < vertices.size(); ++i)
	{
		glm::vec4 v = vertices[i]; 
		center += glm::vec3(v.x, v.y, v.z);
	}
	center /= vertices.size(); 
	return center;
}

void Mesh::get_euler_coordinates(Vec3s &result) const
{
	result.resize(vertices.size());
	for (int i = 0; i < vertices.size(); i++)
	{
		const glm::vec4 &vertex = vertices[i]; 

		float w = vertex.w;
		result[i] = glm::vec3(vertex.x / w, vertex.y / w, vertex.z / w);
	}
}

void Mesh::get_vertex_adjface_matrix(
	std::vector<unsigned int> &adjface, 
	unsigned int maxneighbor, 
	unsigned int pad) const
{
	unsigned int v; 
	std::vector<size_t> indices(vertices.size(), 0);

	adjface.resize(vertices.size() * maxneighbor);

	for (unsigned int i = 0; i < faces.size(); ++i)
	{
		v = faces[i].v0;
		if (indices[v] < maxneighbor)
		{
			adjface[v * maxneighbor + indices[v]] = i;
			++indices[v];
		}
		
		v = faces[i].v1;
		if (indices[v] < maxneighbor)
		{
			adjface[v * maxneighbor + indices[v]] = i;
			++indices[v];
		}

		v = faces[i].v2;
		if (indices[v] < maxneighbor)
		{
			adjface[v * maxneighbor + indices[v]] = i;
			++indices[v];
		}
	}

	for (unsigned int i = 0; i < vertices.size(); ++i)
	{
		if (indices[i] < maxneighbor)
		{
			adjface[i * maxneighbor + indices[i]] = pad;
		}
	}
}