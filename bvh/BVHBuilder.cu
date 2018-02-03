#include <iostream>

#include "../Utilities.h"
#include "../Cloth.h"

#include "BRTreeNode.h"
#include "BVHAccel.h"
#include "BVHBuilder.h"

#define DEFAULT_THREAD_PER_BLOCK 1024

using std::cout; 
using std::endl; 

// Expands a 10-bit integer into 30 bits
// by inserting 2 zeros after each bit.
__device__ MortonCode d_expandBits(MortonCode v)
{
	v = (v * 0x00010001u) & 0xFF0000FFu;
	v = (v * 0x00000101u) & 0x0F00F00Fu;
	v = (v * 0x00000011u) & 0xC30C30C3u;
	v = (v * 0x00000005u) & 0x49249249u;
	return v;
}

__device__ MortonCode d_morton3D(float x, float y, float z)
{
	x = min(max(x * 1024.0f, 0.0f), 1023.0f);
	y = min(max(y * 1024.0f, 0.0f), 1023.0f);
	z = min(max(z * 1024.0f, 0.0f), 1023.0f);
	MortonCode xx = d_expandBits((MortonCode)x);
	MortonCode yy = d_expandBits((MortonCode)y);
	MortonCode zz = d_expandBits((MortonCode)z);
	return xx * 4 + yy * 2 + zz;
}

// Calculates a 30-bit Morton code for the
// given 3D point located within the unit cube [0,1].
__device__ MortonCode d_morton3D(glm::vec3 p)
{
	return d_morton3D(p.x, p.y, p.z);
}

// Allan Yu move to treebuilder.cu
__global__ void get_bboxes(const Primitive *primitives_lst, const Primitive *primitives_cur, unsigned int num, BBox *d_bbox)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= num)
		return;

	Primitive primitive_cur = primitives_cur[index];
	Primitive primitive_lst = primitives_lst[index];
	BBox bbox = primitive_cur.get_bbox();
	bbox.expand(primitive_lst.get_bbox());
	d_bbox[index] = bbox;
}

// Allan Yu move to treebuilder.cu
__global__ void get_bboxes(const Primitive *primitives, unsigned int num, BBox *d_bbox)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= num)
		return;

	Primitive primitive = primitives[index];
	glm::vec3 normal = primitive.get_normal();
	BBox bbox = primitive.get_bbox();
	bbox.expand(bbox.offset(normal * -0.02f)); 
	d_bbox[index] = bbox;
}

// Allan Yu move to treebuilder.cu
__global__ void get_bbox(int num, unsigned int m, const BBox *_d_bbox, BBox *d_bb)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= num)
	{
		return;
	}
	int div = m / num;
	int res = m % num;

	BBox tem;
	for (int i = 0; i < div; i++)  //use shared to replace
	{
		tem.expand(_d_bbox[i * num + index]);
	}
	if (index < res)
	{
		d_bb[index] = tem; 
		return; 
	}
	tem.expand(_d_bbox[m - res + index]);
	d_bb[index] = tem;
	__syncthreads();

	if (index == 0)
	{
		for (int i = 0; i < num; i++)
		{
			tem.expand(d_bb[i]);
		}
		d_bb[0] = tem;
	}

}

// Allan Yu move to treebuilder.cu
__global__ void get_mortons(unsigned int num, BBox bb, const BBox *_d_bbox, MortonCode *d_morton)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= num)
		return;

	d_morton[index] = d_morton3D(bb.normalized_pose_of(_d_bbox[index].centroid()));;
}

// allanyu working here
/**
 *delta operator measures the common prefix of two morton_code
 *if j is not in the range of the sorted_morton_code,
 *delta operator returns -1.
*/
__device__ int delta(int i, int j, MortonCode *sorted_morton_code, int length)
{
	if (j < 0 || j >= length)
	{
		return -1;
	}
	else
	{
		return __clz(sorted_morton_code[i] ^ sorted_morton_code[j]);
	}
}

/**
 *determine the range of an internal node
*/
__device__ int2 determineRange(MortonCode *sorted_morton_code, int num_leaf_nodes, int i)
{
	int d = delta(i, i + 1, sorted_morton_code, num_leaf_nodes) - delta(i, i - 1, sorted_morton_code, num_leaf_nodes);
	d = d > 0 ? 1 : -1;

	//compute the upper bound for the length of the range
	int delta_min = delta(i, i - d, sorted_morton_code, num_leaf_nodes);
	int lmax = 2;
	while (delta(i, i + lmax * d, sorted_morton_code, num_leaf_nodes) > delta_min)
	{
		lmax = lmax * 2;
	}

	//find the other end using binary search
	int l = 0;
	for (int t = lmax / 2; t >= 1; t /= 2)
	{
		if (delta(i, i + (l + t) * d, sorted_morton_code, num_leaf_nodes) > delta_min)
		{
			l = l + t;
		}
	}
	int j = i + l * d;

	int2 range;
	if (i <= j) { range.x = i; range.y = j; }
	else { range.x = j; range.y = i; }
	return range;
}

/**
 *to judge if two values differes
 *in bit position n
*/
__device__ bool is_diff_at_bit(MortonCode val1, MortonCode val2, int n)
{
	return val1 >> (31 - n) != val2 >> (31 - n);
}

/**
 *find the best split position for an internal node
*/
__device__ int findSplit(MortonCode *sorted_morton_code, int start, int last)
{
	//return -1 if there is only 
	//one primitive under this node.
	if (start == last)
	{
		return -1;
	}
	else
	{
		int common_prefix = __clz(sorted_morton_code[start] ^ sorted_morton_code[last]);

		//handle duplicated morton code separately
		if (common_prefix == 32)
		{
			return (start + last) / 2;
		}

		// Use binary search to find where the next bit differs.
		// Specifically, we are looking for the highest object that
		// shares more than commonPrefix bits with the first one.

		int split = start; // initial guess
		int step = last - start;
		do
		{
			step = (step + 1) >> 1; // exponential decrease
			int newSplit = split + step; // proposed new position

			if (newSplit < last)
			{
				bool is_diff = is_diff_at_bit(sorted_morton_code[start],
					sorted_morton_code[newSplit],
					common_prefix);
				if (!is_diff)
				{
					split = newSplit; // accept proposal
				}
			}
		} while (step > 1);

		return split;
	}
}

//FOR BR-TREE CONSTRUCTION
//TODO: implement internal node processing routine
//TODO: handle duplicated morton codes as special case (using their position i,j as fallback)

//FOR BVH CONSTRUCTION
//TODO: implement AABB construction process by go back from the tree node to the root
//TODO: convert BR-TREE BACK TO BVH
//TODO: debug
__global__  void processInternalNode(
	int num_internal_nodes,
	MortonCode *sorted_morton_code,
	BRTreeNode *internalNodes)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= num_internal_nodes) return;

	// Find out which range of objects the node corresponds to.
	int2 range = determineRange(sorted_morton_code, num_internal_nodes + 1, idx);
	int first = range.x;
	int last = range.y;

	// Determine where to split the range.
	int split = findSplit(sorted_morton_code, first, last);

	if (split == -1) return;

	// Select childA.
	int idx_lft = split;
	if (idx_lft == first) {
		idx_lft += num_internal_nodes;
	}
	BRTreeNode *child_lft = internalNodes + idx_lft;

	// Select childB.
	int idx_rht = split + 1;
	if (idx_rht == last) {
		idx_rht += num_internal_nodes;
	}
	BRTreeNode *child_rht = internalNodes + idx_rht;

	// Record parent-child relationships.
	internalNodes[idx].set_lft(idx_lft);
	internalNodes[idx].set_rht(idx_rht);
	child_lft->set_parent(idx);
	child_rht->set_parent(idx);
}

/**
 *construct bounding boxes from leaf up to root
*/
__global__  void calculateBoudingBox(
	int num_internal_nodes,
	unsigned int *counters,
	const BBox *bboxes, 
	BRTreeNode *internalNodes)
{
	int idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (idx >= num_internal_nodes + 1) 
	{
		return;
	}

	// handle leaf first
	BRTreeNode *node = internalNodes + num_internal_nodes + idx;
	node->set_bbox(bboxes[idx]); 
	node->set_idx(idx); 

	//terminate if it is root node
	int idx_parent = node->parent(); 
	node = internalNodes + idx_parent;

	int initial_val = atomicInc(counters + idx_parent, 1);
	while (1)
	{
		// terminate the first accesing thread
		if (initial_val == 0)
		{
			return;
		}

		// calculate bounding box by merging two children's bounding box
		int idx_lft = node->lft();
		node->expand(internalNodes[idx_lft].bbox());

		int idx_rht = node->rht();
		node->expand(internalNodes[idx_rht].bbox());

		//terminate if it is root node
		idx_parent = node->parent();;
		if (idx_parent < 0)
		{
			return;
		}
		node = internalNodes + idx_parent;
		initial_val = atomicInc(counters + idx_parent, 1);
	}
}

/////////////////////////////////////////////////
/**
 *intialize parallelBRTreeBuilder by copying the data needed
 *from host memory (CPU) to device memory (GPU), initialize
 *data members such as configuration parameters.
*/
BVHBuilder::BVHBuilder() : _size(0), _d_morton_codes(NULL), _d_bboxes(NULL)
{

}

void BVHBuilder::ref_auto_clean() {
	cudaFree(_d_bboxes);
	cudaFree(_d_morton_codes);
	cudaFree(_d_bbox);
	cudaFree(_d_counters);
}

void BVHBuilder::prepare_memory(unsigned int size)
{
	if (size <= _size )
	{
		return; 
	}
	ref_renew(); 

	_size = size; 

	_tree_nodes.resize(size * 2 - 1);
	_bboxes.resize(size);
	_morton_codes.resize(size);
	_sorted_bboxes.resize(size);
	_sorted_morton_codes.resize(size);
	_sorted_primitives_lst.resize(size);
	_sorted_primitives_cur.resize(size);

	safe_cuda(cudaMalloc(&_d_bbox, sizeof(BBox) * 128));
	safe_cuda(cudaMalloc(&_d_morton_codes, size * sizeof(MortonCode)));
	safe_cuda(cudaMalloc(&_d_bboxes, size * sizeof(BBox)));
	safe_cuda(cudaMalloc(&_d_counters, size * sizeof(unsigned int)));
}

// void BVHBuilder::build_impl(bool ccd, BVHAccel &tree, const Primitives &primitives_last, const Primitives &primitives)
void BVHBuilder::build_impl(bool ccd, BVHAccel &tree, const Mesh &body)
{
	if (body.vertices.empty() || body.faces.empty())
	{
		cout << "return" << endl; 
		return; 
	}
	unsigned int size = body.faces.size();
	unsigned int num_vertices = body.vertices.size();

	tree.resize(num_vertices, size);

	Vec3s obj(num_vertices);
	body.get_euler_coordinates(obj);
	
	if (_body_lst.size() != body.faces.size())
	{
		compute_primitives(body, _body_lst, tree._vertices_lst);
		compute_primitives(body, _body_cur, tree._vertices_cur);
	}

	if (ccd)
	{
		safe_cuda(cudaMemcpy(tree._vertices_lst, tree._vertices_cur, sizeof(glm::vec3) * num_vertices, cudaMemcpyDeviceToDevice));
		safe_cuda(cudaMemcpy(tree._vertices_cur, &obj[0], sizeof(glm::vec3) * obj.size(), cudaMemcpyHostToDevice));
	}
	else
	{
		safe_cuda(cudaMemcpy(tree._vertices_cur, &obj[0], sizeof(glm::vec3) * obj.size(), cudaMemcpyHostToDevice));
	}

	// 确保构建树所需空间足够
	prepare_memory(size);

	// 一次计算每个primitive的bbox
	compute_bboxes(tree, ccd, size, _body_lst, _body_cur);

	// 获取整个bb
	// calculate root AABB size
	BBox bbox;
	compute_bbox(size, 128, bbox);

	// 获取整个morton code
	compute_morton_codes(size, bbox);

	// 排序 去重
	compute_sorted_and_unique_codes(tree, ccd, size, _body_lst, _body_cur);

	// cout << "start building parallel brtree" << endl;
	// delegate the binary radix tree construction process to GPU
	compute_tree(tree, size);
}


void BVHBuilder::compute_primitives(const Mesh &body, Primitives &h_primitives, glm::vec3 *d_obj_vertices)
{
	// create primitives
	h_primitives.resize(body.faces.size());
	for (int i = 0; i < h_primitives.size(); ++i)
	{
		Primitive primitive(d_obj_vertices, body.faces[i].v0, body.faces[i].v1, body.faces[i].v2);
		h_primitives[i] = primitive;
	}
}

void BVHBuilder::compute_bboxes(
	BVHAccel &tree,
	bool ccd,
	unsigned int size,
	const Primitives &primitives_last,
	const Primitives &primitives)
{
	unsigned int block_size = 512;
	unsigned int num_threads = min(block_size, size);
	unsigned int num_blocks = (size % num_threads != 0) ? (size / num_threads + 1) : (size / num_threads);

	// 初始内存
	safe_cuda(cudaMemcpy(tree._primitives_cur, &primitives[0], sizeof(Primitive) * size, cudaMemcpyHostToDevice));
	if (ccd)
	{
		safe_cuda(cudaMemcpy(tree._primitives_lst, &primitives_last[0], sizeof(Primitive) * size, cudaMemcpyHostToDevice));
	}

	if (ccd)
	{
		get_bboxes << <num_blocks, num_threads >> > (tree._primitives_lst, tree._primitives_cur, size, _d_bboxes);
	}
	else
	{
		get_bboxes << <num_blocks, num_threads >> > (tree._primitives_cur, size, _d_bboxes);
	}
	cudaMemcpy(&_bboxes[0], _d_bboxes, sizeof(BBox) * size, cudaMemcpyDeviceToHost);
}

void BVHBuilder::compute_bbox(
	unsigned int size,
	unsigned int num_threads,
	BBox &bbox)
{
#if 0 // cpu
	BBox bb;
	for (unsigned int i = 0; i < size; ++i) {
		bbox.expand(_bboxes[i]);
	}
#endif

	get_bbox << <1, num_threads >> > (num_threads, size, _d_bboxes, _d_bbox);
	safe_cuda(cudaMemcpy(&bbox, _d_bbox, sizeof(BBox), cudaMemcpyDeviceToHost));
}

void BVHBuilder::compute_morton_codes(unsigned int size, const BBox &bbox)
{
	unsigned int block_size = 512;
	unsigned int num_threads = min(block_size, size);
	unsigned int num_blocks = (size % num_threads != 0) ? (size / num_threads + 1) : (size / num_threads);

	get_mortons << <num_blocks, num_threads >> > (size, bbox, _d_bboxes, _d_morton_codes);
	cudaMemcpy(&_morton_codes[0], _d_morton_codes, sizeof(MortonCode) * size, cudaMemcpyDeviceToHost);
}

void BVHBuilder::compute_sorted_and_unique_codes(BVHAccel &tree, bool ccd, unsigned int &size, const Primitives &primitives_last, const Primitives &primitives)
{
	// 排序
	// thrust::sort(thrust::host, primitives.begin(),primitives.end());
	// cpu is faster than gpu, are u kidding me?
	vector<unsigned int> indices(size);
	indices_sort(_morton_codes, indices);

	// 去重
	remove_redundant(_morton_codes, indices);
	size = indices.size();

	if (ccd)
	{
		filter(primitives_last, indices, _sorted_primitives_lst);
		safe_cuda(cudaMemcpy(tree._primitives_lst, &_sorted_primitives_lst[0], sizeof(Primitive) * size, cudaMemcpyHostToDevice));
	}

	filter(primitives, indices, _sorted_primitives_cur);
	filter(_bboxes, indices, _sorted_bboxes);
	filter(_morton_codes, indices, _sorted_morton_codes);

	safe_cuda(cudaMemcpy(tree._primitives_cur, &_sorted_primitives_cur[0], sizeof(Primitive) * size, cudaMemcpyHostToDevice));
	safe_cuda(cudaMemcpy(_d_bboxes, &_sorted_bboxes[0], sizeof(BBox) * size, cudaMemcpyHostToDevice));
	safe_cuda(cudaMemcpy(_d_morton_codes, &_sorted_morton_codes[0], sizeof(MortonCode) * size, cudaMemcpyHostToDevice));

}

void BVHBuilder::compute_tree(BVHAccel &tree, unsigned int size)
{
	unsigned int num_internal_nodes = size - 1;
	unsigned int num_leaf_nodes = size;

	safe_cuda(cudaMemcpy(tree._tree_nodes, &_tree_nodes[0], sizeof(BRTreeNode) * (num_leaf_nodes + num_internal_nodes), cudaMemcpyHostToDevice));
	safe_cuda(cudaMemset(_d_counters, 0, sizeof(unsigned int) * num_internal_nodes));
	
	unsigned int numBlock, threadPerBlock = DEFAULT_THREAD_PER_BLOCK;

	//////////////////////////////////////////////////////////////////
	// build the bvh
	numBlock = (num_internal_nodes + DEFAULT_THREAD_PER_BLOCK - 1) / threadPerBlock; 
	processInternalNode << <numBlock, threadPerBlock >> >(num_internal_nodes, _d_morton_codes, tree._tree_nodes);

	//fix << <1, 1 >> > (d_leaf_nodes, d_internal_nodes);

	//calculate bounding box
	numBlock = (num_leaf_nodes + DEFAULT_THREAD_PER_BLOCK - 1) / threadPerBlock;
	calculateBoudingBox << <numBlock, threadPerBlock >> >(num_internal_nodes, _d_counters, _d_bboxes, tree._tree_nodes);
}