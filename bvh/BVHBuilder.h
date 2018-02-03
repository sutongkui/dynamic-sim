#pragma once

#include <vector>

#include <cuda_runtime.h>

#include "BBox.h"
#include "Primitive.h"
#include "BRTreeNode.h"

using std::vector; 

class Mesh;
class BVHAccel; 

typedef uint32 MortonCode; 
typedef std::vector<MortonCode> MortonCodes;

/**
 *build binary radix tree on GPU
*/
class BVHBuilder : public RefObject
{
public:
	BVHBuilder();
	~BVHBuilder() { ref_release(); }

	void build_bvh(BVHAccel &tree, const Mesh &body)
	{
		build_impl(false, tree, body);
	}
	void build_ccd(BVHAccel &tree, const Mesh &body)
	{
		build_impl(true, tree, body);
	}

private:
	virtual void ref_auto_clean();

	void prepare_memory(unsigned int size);

	void build_impl(bool ccd, BVHAccel &tree, const Mesh &body);

	void compute_primitives(const Mesh &body, Primitives &h_primitives, glm::vec3 *d_obj_vertices);
	void compute_bboxes(BVHAccel &tree, bool ccd, unsigned int size, const Primitives &primitives_last, const Primitives &primitives);
	void compute_bbox(unsigned int size, unsigned int num_threads, BBox &bbox);
	void compute_morton_codes(unsigned int size, const BBox &bbox);
	void compute_sorted_and_unique_codes(BVHAccel &tree, bool ccd, unsigned int &size, const Primitives &primitives_last, const Primitives &primitives);
	void compute_tree(BVHAccel &tree, unsigned int size);

private:
	// 用于重复利用时保证空间足够
	unsigned int _size;
	
	// 所有treebuilder的属性都与后续bvh搜索无关，仅用于建立树时所需的临时变量
	// 之所以做成成员变量，是考虑可以重复是用builder，避免反复开辟空间的损耗
	// 同时也可以减少函数间参数传递
	Primitives _body_lst;
	Primitives _body_cur;

	BRTreeNodes _tree_nodes;

	BBoxes _bboxes;
	BBoxes _sorted_bboxes;

	MortonCodes _morton_codes;
	MortonCodes _sorted_morton_codes;

	Primitives _sorted_primitives_lst;
	Primitives _sorted_primitives_cur;

	BBox *_d_bboxes;
	BBox *_d_bbox;
	MortonCode *_d_morton_codes;
	unsigned int *_d_counters;
};
