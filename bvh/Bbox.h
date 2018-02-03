#pragma once
#include <vector>

#include <cuda_runtime.h>
#include <glm/glm.hpp>

#include "../Common.h"

#define INF_D 100.0

class BBox
{
public:
	/**
	* Constructor.
	* The default constructor creates a new bounding box which contains no
	* points.
	*/
	__host__ __device__ BBox() : _max(-INF_D), _min(INF_D), _extent(-2 * INF_D) { }

	/**
	 *Constructor.
	 *Creates a bounding box that includes a single point.
	*/
	__host__ __device__ BBox(const glm::vec3 &p) : _min(p), _max(p) { }

	/**
	 *Constructor.
	 *Creates a bounding box with given bounds.
	 *\param _min the _min corner
	 *\param _max the _max corner
	*/
	__host__ __device__ BBox(const glm::vec3 &min, const glm::vec3 &max) : _min(min), _max(max) 
	{
		_extent = _max - _min;
	}

	/**
	 *Constructor.
	 *Creates a bounding box with given bounds (component wise).
	*/
	__host__ __device__ BBox(
		const double x1, const double y1, const double z1, 
		const double x2, const double y2, const double z2) : 
		_min(x1, y1, z1), _max(x2, y2, z2)
	{
		_extent = _max - _min;
	}

	/**
	 *Expand the bounding box to include another (union).
	 *If the given bounding box is contained within *this*, nothing happens.
	 *Otherwise *this *is expanded to the minimum volume that contains the
	 *given input.
	 *\param bbox the bounding box to be included
	*/
	__host__ __device__ void expand(const BBox &rhs) 
	{
		_min = glm::min(_min, rhs._min);
		_max = glm::max(_max, rhs._max);
		_extent = _max - _min;
	}

	/**
	 *Intersects point with bounding box, does not store shading information.
	Checking if a point is inside an AABB is pretty simple ¡ª we just need to check whether the point's coordinates fall inside the AABB;
	considering each axis separately. If we assume that Px, Py and Pz are the point's coordinates,
	and BminX¨CBmaxX, BminY¨CBmaxY, and BminZ¨CBmaxZ are the ranges of each exis of the AABB,
	we can calculate whether a collision has occured between the two using the following formula:

	f(P,B)=(Px>=BminX¡ÄPx<=BmaxX)¡Ä(Py>=BminY¡ÄPy<=BmaxY)¡Ä(Pz>=BminZ¡ÄPz<=BmaxZ)
	*/
	__host__ __device__ bool intersect(const glm::vec3 &p) const
	{
		return (p.x >= _min.x && p.x <= _max.x) && 
			(p.y >= _min.y && p.y <= _max.y) &&
			(p.z >= _min.z && p.z <= _max.z);
	}

	__host__ __device__ glm::vec3 centroid() const {
		glm::vec3 sum = _min + _max;
		sum /= 2;
		return sum;
	}

	__host__ __device__ BBox offset(const glm::vec3 &p) const
	{
		BBox box = *this; 
		box._max += p; 
		box._min += p;

		return box; 
	}

	/**
	*Calculate and return an object's
	*normalized position in the unit
	*cube defined by this BBox. if the
	*object is not inside of the BBox, its
	*position will be clamped into the BBox.
	*
	*\param pos the position to be evaluated
	*\return the normalized position in the unit
	*cube, with x,y,z ranging from [0,1]
	*/
	__host__ __device__ glm::vec3 normalized_pose_of(glm::vec3 p)
	{
		if (_extent.x == 0 || _extent.y == 0 || _extent.z == 0)
		{
			return glm::vec3();
		}
		glm::vec3 o2pos = p - _min;
		o2pos /= _extent;
		return o2pos;
	}

	/**
	 *Draw box wireframe with OpenGL.
	*/
	void draw() const;
	void print() const;

	
private:
	glm::vec3 _max;		///< _max corner of the bounding box
	glm::vec3 _min;	    ///< _min corner of the bounding box
	glm::vec3 _extent;	///< _extent of the bounding box (_min -> _max)

};

typedef std::vector<BBox> BBoxes;