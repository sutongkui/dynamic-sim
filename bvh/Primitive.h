#pragma once
#include <vector>

#include <glm/glm.hpp>

#include "../Common.h"
#include "BBox.h"

// here primitive refer to triangle
class Primitive
{
public:
	Primitive() : _vertices(NULL) { };
	Primitive(
		const glm::vec3 *vertices, 
		const size_t v0, 
		const size_t v1, 
		const size_t v2
	) : _vertices(vertices), _v0(v0), _v1(v1), _v2(v2) { }
	
	/**
	 *Get the world space bounding box of the primitive.
	 *\return world space bounding box of the primitive
	*/
	__host__ __device__ const glm::vec3 &v0() const
	{
		return _vertices[_v0]; 
	}

	__host__ __device__ const glm::vec3 &v1() const
	{
		return _vertices[_v1];
	}

	__host__ __device__ const glm::vec3 &v2() const
	{
		return _vertices[_v2];
	}

	__host__ __device__ BBox get_bbox() const
	{
		BBox bbox(v0());
		bbox.expand(v1());
		bbox.expand(v2());
		return bbox;
	};

	/**
	 *Check if the given point intersects with the primitive, no intersection
	 *information is stored
	 *\return true if the given point intersects with the primitive,
	false otherwise
	*/
	__host__ __device__ float distance_to(const glm::vec3 &point, glm::vec3 &normal) const
	{
		// use normal or barycentric coordinates
		normal = get_normal();
		
		glm::vec3 tem = point - _vertices[_v0];
		return glm::dot(tem, normal);
	}

	__host__ __device__ float distance_to(const glm::vec3 &point) const
	{
		// use normal or barycentric coordinates
		glm::vec3 normal = get_normal();

		glm::vec3 tem = point - _vertices[_v0];
		return glm::dot(tem, normal);
	}

	__host__ __device__ float udistance_to(const glm::vec3 &point) const
	{
		// use normal or barycentric coordinates
		return fabs(udistance_to(point)); 
	}

	__host__ __device__ glm::vec3 get_normal() const
	{
		glm::vec3 v = v0(); 
		glm::vec3 side1 = v1() - v;
		glm::vec3 side2 = v2() - v;
		glm::vec3 normalface = glm::cross(side1, side2);
		return glm::normalize(normalface);
	}

	__host__ __device__ glm::vec3 get_center() const
	{
		glm::vec3 center = v0();
		center += v1();
		center += v2();
		center /= 3.0f;
		return center;
	}

private:
	const glm::vec3 *_vertices;  //for device, ptr to _vertices
	size_t _v0;
	size_t _v1;
	size_t _v2;
};

typedef std::vector<Primitive> Primitives;