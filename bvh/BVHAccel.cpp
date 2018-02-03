#include <iostream>
#include <algorithm>
#include <bitset>

#include <cuda.h>
#include <device_functions.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <thrust/sort.h>
#include <thrust/execution_policy.h>

#include "../Utilities.h"
#include "BVHAccel.h"
#include "Primitive.h"
#include "BVHBuilder.h"

using namespace std;

///////////////////////////////////////////////////////////
BVHAccel::BVHAccel() :
	_number_faces(0),
	_number_vertices(0),
	_vertices_lst(NULL),
	_vertices_cur(NULL),
	_primitives_lst(NULL),
	_primitives_cur(NULL),
	_tree_nodes(NULL)
{

}

void BVHAccel::resize(unsigned int number_vertices, unsigned int number_faces)
{
	if (number_vertices <= _number_vertices && number_faces <= _number_faces)
	{
		return;
	}
	ref_renew();

	_number_faces = number_faces;
	_number_vertices = number_vertices;

	safe_cuda(cudaMalloc(&_vertices_lst, sizeof(glm::vec3) * number_vertices));
	safe_cuda(cudaMalloc(&_vertices_cur, sizeof(glm::vec3) * number_vertices));

	safe_cuda(cudaMalloc(&_primitives_cur, sizeof(Primitive) * number_faces));
	safe_cuda(cudaMalloc(&_primitives_lst, sizeof(Primitive) * number_faces));

	safe_cuda(cudaMalloc(&_tree_nodes, sizeof(BRTreeNode) * number_faces * 2 - 1));
}

void BVHAccel::ref_auto_clean()
{
	cudaFree(_vertices_lst);
	cudaFree(_vertices_cur);

	cudaFree(_primitives_cur);
	cudaFree(_primitives_lst);
	cudaFree(_tree_nodes);

#if 0
	free(h_leaf_nodes);
	free(h_internal_nodes);
#endif
}


#if 0
void BVHAccel::access(BRTreeNode *root, vector<BRTreeNode*> &bad_bode)
{
	if (root->bbox.min.x > root->bbox.max.x)
	{
		if (is_leaf(root))
		{
			bad_bode.push_back(root);
			return;
		}
		else
		{
			access(get_left_child(root), bad_bode);
			access(get_right_child(root), bad_bode);
		}
	}
}

void BVHAccel::pre_drawoutline()
{
	copyFromGPUtoCPU((void**)&h_internal_nodes, _internal_nodes, sizeof(BRTreeNode) * num_internal_node);
	copyFromGPUtoCPU((void**)&h_leaf_nodes, _leaf_nodes, sizeof(BRTreeNode) * num_leaf_node);
}

void BVHAccel::draw(BRTreeNode *root)
{
	root->bbox.draw();
	if (is_leaf(root))
	{
		return;
	}
	else
	{
		draw(get_left_child(root));
		draw(get_right_child(root));
	}
}
#endif


