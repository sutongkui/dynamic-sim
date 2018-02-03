#pragma once

#include <vector>

#include <cuda_runtime.h>

#include "BBox.h"

/**
 *BRTreeNode
 *
 *BRTreeNode stands for a node in the
 *binary radix tree.
 *
 *the index of children and parent node
 *into the node array is encoded in the
 *following way:
 *
 *1) When the value is positive, it
 *refers to the node in internal node array.
 *the encoded value is (val-1)
 *
 *2) When the value is negative, it refers to
 *the node in leaf node array. And in the latter
 *situation, the encoded value is -(val+1)
 *
 *For example: If childA is 3, it means the left
 *child of the current node is in internal node
 *array with an offset of 2. If the childB is -1,
 *it means the right child of the current node
 *is in the leaf node array with an offset of 0.
 */

// Allan yu 还没有优化该函数涉及部分
class BRTreeNode
{
public:
	BRTreeNode() : _lft(-1), _rht(-1), _parent(-1), _pid(-1), _bbox() { }

	__host__ __device__ int lft() const { return _lft; }
	__host__ __device__ int rht() const { return _rht; }
	__host__ __device__ int parent() const { return _parent; }
	__host__ __device__ int pid() const { return _pid; }

	__host__ __device__ const BBox &bbox() const
	{ 
		return _bbox;
	}

	__host__ __device__ bool intersect(const glm::vec3 &point) const
	{
		return _bbox.intersect(point);
	}

	__host__ __device__ bool leaf() const
	{
		return _pid >= 0;
	}

	__host__ __device__ void set_lft(int l) { _lft = l; }
	__host__ __device__ void set_rht(int r) { _rht = r; }
	__host__ __device__ void set_parent(int p) { _parent = p; }
	__host__ __device__ void set_idx(int index) { _pid = index; }
	__host__ __device__ void set_bbox(const BBox &bbox)
	{
		_bbox = bbox;
	}

	__host__ __device__ void expand(const BBox &bbox) 
	{ 
		_bbox.expand(bbox);
	}

	__host__ void printInfo()
	{
		printf("-----\n");
		printf("childA:(%d)\n", lft());
		printf("childB:(%d)\n", rht());
		printf("parent:(%d)\n", parent());
		printf("index:%d\n", pid());
	}

private:
	BBox _bbox;

	int _lft;
	int _rht;
	int _parent;
	int _pid;

	friend class BVHBuilder; 
};

typedef std::vector<BRTreeNode> BRTreeNodes;


