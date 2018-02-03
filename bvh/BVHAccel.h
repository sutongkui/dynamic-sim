#pragma once
#include <vector> 

#include "../Utilities.h"
#include "BBox.h"
#include "Primitive.h"
#include "BRTreeNode.h"

/**
 *Bounding Volume Hierarchy for fast point-objects intersection.
 *Note that the BVHAccel is an Aggregate (A Primitive itself) that contains
 *all the _primitives_cur it was built from. Therefore once a BVHAccel Aggregate
 *is created, the original input _primitives_cur can be ignored from the scene
 *during point-objects intersection tests as they are contained in the aggregate.
*/
class BVHAccel : public RefObject
{
public:
	BVHAccel();
	~BVHAccel() { ref_release(); }

private:
	virtual void ref_auto_clean();

	void resize(unsigned int number_vertices, unsigned int number_faces);

public:
	__host__ __device__ BRTreeNode *get_root() const
	{
		return _tree_nodes;
	}

	__host__ __device__ BRTreeNode *get_left_child(BRTreeNode *node) const
	{
		int idx = node->lft();
		return idx < 0 ? NULL : _tree_nodes + idx;
	}

	__host__ __device__ BRTreeNode *get_right_child(BRTreeNode *node) const
	{
		int idx = node->rht();
		return idx < 0 ? NULL : _tree_nodes + idx;
	}

	__host__ __device__ bool intersect(const glm::vec3 point, int &idx) const
	{
		// Allocate traversal stack from thread-local memory,
		// and push NULL to indicate that there are no postponed nodes.
		BRTreeNode *stack[64];
		BRTreeNode **stackPtr = stack;
		*stackPtr++ = NULL; // push

							// Traverse nodes starting from the root.
		BRTreeNode *node = get_root();
		do
		{
			// Check each child node for overlap.
			BRTreeNode *child_lft = get_left_child(node);
			BRTreeNode *child_rht = get_right_child(node);
			bool overlap_lft = child_lft->intersect(point);
			bool overlap_rht = child_rht->intersect(point);
			bool leaf_lft = child_lft->leaf();
			bool leaf_rht = child_rht->leaf();

			// Query overlaps a leaf node => report collision with the first collision.
			if (overlap_lft && leaf_lft)
			{
				idx = child_lft->pid();       //is a leaf, and we can get it through primitive[idx]
				return true;
			}

			if (overlap_rht && leaf_rht)
			{
				idx = child_rht->pid();
				return true;
			}

			// Query overlaps an internal node => traverse.
			bool traverse_lft = (overlap_lft && !leaf_lft);
			bool traverse_rht = (overlap_rht && !leaf_rht);

			if (!traverse_lft && !traverse_rht)
			{
				node = *--stackPtr; // pop
			}
			else
			{
				node = (traverse_lft) ? child_lft : child_rht;
				if (traverse_lft && traverse_rht)
					*stackPtr++ = child_rht; // push
			}
		} while (node != NULL);

		return false;
	}

	__host__ __device__ bool coplanarIntersect(const glm::vec3 point, int &idx) const
	{
		// Allocate traversal stack from thread-local memory,
		// and push NULL to indicate that there are no postponed nodes.
		BRTreeNode *stack[64];
		BRTreeNode **stackPtr = stack;
		*stackPtr++ = NULL; // push

							// Traverse nodes starting from the root.
		BRTreeNode *node = get_root();
		do
		{
			// Check each child node for overlap.
			float dist_lst, dist_cur; 
			BRTreeNode *child_lft = get_left_child(node);
			BRTreeNode *child_rht = get_right_child(node);
			bool overlap_lft = child_lft->intersect(point);
			bool overlap_rht = child_rht->intersect(point);
			bool leaf_lft = child_lft->leaf();
			bool leaf_rht = child_rht->leaf();

			// Query overlaps a leaf node => report collision with the first collision.
			if (overlap_lft && leaf_lft)
			{
				idx = child_lft->pid();       //is a leaf, and we can get it through primitive[idx]
				if (overlap_rht && leaf_rht)
				{
					idx = (glm::distance(point, child_lft->bbox().centroid()) > glm::distance(point, child_rht->bbox().centroid())) ? child_rht->pid() : child_lft->pid();
				}
				dist_cur = _primitives_cur[idx].distance_to(point);
				dist_lst = _primitives_lst[idx].distance_to(point);
				if (dist_cur < 0 && dist_lst > 0)	return true;
			}

			if (overlap_rht && leaf_rht)
			{
				idx = child_rht->pid();
				dist_cur = _primitives_cur[idx].distance_to(point);
				dist_lst = _primitives_lst[idx].distance_to(point);
				if (dist_cur < 0 && dist_lst > 0)	return true;
			}

			// Query overlaps an internal node => traverse.
			bool traverse_lft = (overlap_lft && !leaf_lft);
			bool traverse_rht = (overlap_rht && !leaf_rht);

			if (!traverse_lft && !traverse_rht)
				node = *--stackPtr; // pop
			else
			{
				if (traverse_lft && traverse_rht)
				{
					node = (glm::distance(point, child_lft->bbox().centroid()) > glm::distance(point, child_rht->bbox().centroid())) ? child_rht : child_lft;
					*stackPtr++ = (node == child_lft) ? child_rht : child_lft;// push
				}

				else node = (traverse_lft) ? child_lft : child_rht;
			}
		} while (node != NULL);

		return false;
	}

	__host__ __device__ bool nearestIntersect(const glm::vec3 point, int &idx)
	{
		// Allocate traversal stack from thread-local memory,
		// and push NULL to indicate that there are no postponed nodes.
		BRTreeNode *stack[64];
		BRTreeNode **stackPtr = stack;
		*stackPtr++ = NULL; // push

							// Traverse nodes starting from the root.
		BRTreeNode *node = get_root();
		do
		{
			// Check each child node for overlap.
			BRTreeNode *child_lft = get_left_child(node);
			BRTreeNode *child_rht = get_right_child(node);
			bool overlap_lft = child_lft->intersect(point);
			bool overlap_rht = child_lft->intersect(point);
			bool leaf_lft = child_lft->leaf();
			bool leaf_rht = child_rht->leaf();

			// Query overlaps a leaf node => report collision with the first collision.
			if (overlap_lft && leaf_lft)
			{
				if (overlap_rht && leaf_rht)
				{
					idx = (glm::distance(point, child_lft->bbox().centroid()) > glm::distance(point, child_rht->bbox().centroid())) ? child_rht->pid() : child_lft->pid();
					return true;
				}
				idx = child_lft->pid();       //is a leaf, and we can get it through primitive[idx]
				return true;
			}

			if (overlap_rht && leaf_rht)
			{
				idx = child_rht->pid();
				return true;
			}

			// Query overlaps an internal node => traverse.
			bool traverse_lft = (overlap_lft && !leaf_lft);
			bool traverse_rht = (overlap_rht && !leaf_rht);

			if (!traverse_lft && !traverse_rht)
				node = *--stackPtr; // pop
			else
			{
				if (traverse_lft && traverse_rht)
				{
					node = (glm::distance(point, child_lft->bbox().centroid()) > glm::distance(point, child_rht->bbox().centroid())) ? child_rht : child_lft;
					*stackPtr++ = (node == child_lft) ? child_rht : child_lft;// push
				}

				else node = (traverse_lft) ? child_lft : child_rht;

			}
		} while (node != NULL);

		return false;
	}
	__host__ __device__  const Primitive &curpri(unsigned int idx) const
	{
		return _primitives_cur[idx];
	}

	__host__ __device__ const Primitive &lstpri(unsigned int idx) const
	{
		return _primitives_lst[idx];
	}

private:
	unsigned int _number_faces;
	unsigned int _number_vertices;

	glm::vec3 *_vertices_lst;
	glm::vec3 *_vertices_cur;

	Primitive *_primitives_cur;
	Primitive *_primitives_lst;

	BRTreeNode *_tree_nodes;

#if 0
public:
	//显示包围盒之前需要调用，完成数据从GPU到CPU的拷贝
	__host__
		void pre_drawoutline();  //for test

	__host__
		void draw(BRTreeNode *root);

	__host__
		void access(BRTreeNode *root, vector<BRTreeNode*>& bad_bode);

private:
	// 以后再改，但要确保不用copyFromGPUtoCPU生成空间，破坏创建和释放的对偶关系
	unsigned int _num_internal_node;
	unsigned int _num_leaf_node;

	BRTreeNode *h_leaf_nodes;
	BRTreeNode *h_internal_nodes;

public:
	__host__
		BRTreeNode *get_leaf_nodes()
	{
		copyFromGPUtoCPU((void**)&h_leaf_nodes, _leaf_nodes, _num_leaf_node * sizeof(BRTreeNode));
		return h_leaf_nodes;
	}

	__host__
		BRTreeNode *get_internal_nodes()
	{
		copyFromGPUtoCPU((void**)&h_internal_nodes, _internal_nodes, _num_internal_node * sizeof(BRTreeNode));
		return h_internal_nodes;
	}

	__host__
		inline void printLeafNode()
	{
		for (int i = 0; i < _num_leaf_node; i++)
		{
			_leaf_nodes[i].printInfo();
		}
		return;
	}

	__host__
		inline void printInternalNode()
	{
		for (int i = 0; i < _num_internal_node; i++)
		{
			_internal_nodes[i].printInfo();
		}
		return;
	}
#endif
	friend class BVHBuilder;
};

#if 0
// Allan Yu unused
// Allan Yu move to bvh.cu
//data stack overflow when recursively
static __host__ __device__ bool recursive_intersect(BRTreeNode *_leaf_nodes, BRTreeNode *_internal_nodes, BRTreeNode *root, const glm::vec3 point, int &idx)
{
	bool overlap = check_overlap(point, root);
	if (!overlap)
		return false;
	if (is_leaf(root))
	{
		idx = root->getIdx();
		return true;
	}
	else
	{
		recursive_intersect(_leaf_nodes, _internal_nodes, get_left_child(_leaf_nodes, _internal_nodes, root), point, idx);
		recursive_intersect(_leaf_nodes, _internal_nodes, get_right_child(_leaf_nodes, _internal_nodes, root), point, idx);
	}
}
#endif

