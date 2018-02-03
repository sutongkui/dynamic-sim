#include <iostream>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include "Utilities.h"
#include "SpringsBuilder.h"
#include "bvh\BRTreeNode.h"
#include "bvh\BVHAccel.h"
#include "Cloth.h"
#include "ObjLoader.h"
#include "Verlet.h"
#include "Simulator.h"

using namespace std;

extern GLenum GL_MODE;

Simulator::Simulator() :
	_size_faces(0), _size_vertices(0),
	_num_faces(0), _num_vertices(0),
	_id_in(0), _id_out(1),
	_d_x_orignal(NULL),
	_d_dir_collision_force(NULL),
	_d_dir_face_normals(NULL),
	_d_adj_spring_st(NULL),
	_d_adj_spring_bd(NULL),
	_d_adj_face_to_vertices(NULL),
	_d_adj_vertex_to_faces(NULL),
#ifdef DEBUG_COLLISION
	collided_vertex(NULL),
#endif
	_d_vbo_resource(NULL),
	_d_vbo_vertices(NULL),
	_d_vbo_normals(NULL)
{
	_d_x_cur[0] = NULL;
	_d_x_cur[1] = NULL;
	_d_x_lst[0] = NULL;
	_d_x_lst[1] = NULL;

}

Simulator::~Simulator()
{
	ref_release();
}

void Simulator::update_body(const Mesh &body)
{
	if (body.vertices.empty()) {
		cout << "return" << endl;
		return;
	}

	Mesh ex_body = body;
	ex_body.extend(0.01f);

	_bvh_builder.build_bvh(_bvh_tree, ex_body);
	_ccd_builder.build_ccd(_ccd_tree, body);
}

// 管理空间
void Simulator::create_buffer()
{
	size_t heap_size = 256 * 1024 * 1024;  //set heap size, the default is 8M
	cudaDeviceSetLimit(cudaLimitMallocHeapSize, heap_size);

	//将sim_cloth的点的坐标发送到GPU
	const size_t vertices_bytes = sizeof(glm::vec3) * _size_vertices;
	safe_cuda(cudaMalloc((void**)&_d_x_orignal, vertices_bytes));
	safe_cuda(cudaMalloc((void**)&_d_x_cur[0], vertices_bytes));
	safe_cuda(cudaMalloc((void**)&_d_x_cur[1], vertices_bytes));
	safe_cuda(cudaMalloc((void**)&_d_x_lst[0], vertices_bytes));
	safe_cuda(cudaMalloc((void**)&_d_x_lst[1], vertices_bytes));

	_d_x_cur_in = _d_x_cur[_id_in];
	_d_x_lst_in = _d_x_lst[_id_in];

	_d_x_cur_out = _d_x_cur[_id_out];
	_d_x_lst_out = _d_x_lst[_id_out];

	safe_cuda(cudaMalloc((void**)&_d_dir_collision_force, sizeof(glm::vec3) * _size_vertices));
	// 面的法向量
	safe_cuda(cudaMalloc((void**)&_d_dir_face_normals, sizeof(glm::vec3) * _size_faces));

	// 每个点邻接的面的索引
	safe_cuda(cudaMalloc((void**)&_d_adj_vertex_to_faces, sizeof(unsigned int) * _size_vertices * sim_parameter.NUM_PER_VERTEX_ADJ_FACES));
	// 点的索引
	safe_cuda(cudaMalloc((void**)&_d_adj_face_to_vertices, sizeof(unsigned int) * _size_faces * 3));

	safe_cuda(cudaMalloc((void**)&_d_adj_spring_st, sizeof(unsigned int) * _size_vertices * sim_parameter.NUM_PER_VERTEX_SPRING_STRUCT));
	safe_cuda(cudaMalloc((void**)&_d_adj_spring_bd, sizeof(unsigned int) * _size_vertices * sim_parameter.NUM_PER_VERTEX_SPRING_STRUCT));

#ifdef DEBUG_COLLISION
	safe_cuda(cudaMalloc((void**)&collided_vertex, sizeof(unsigned int) * _size_vertices));
#endif
}


void Simulator::ref_auto_clean()
{
	cudaFree(_d_x_orignal);
	cudaFree(_d_x_cur[0]);
	cudaFree(_d_x_cur[1]);
	cudaFree(_d_x_lst[0]);
	cudaFree(_d_x_lst[1]);
	cudaFree(_d_dir_collision_force);
	cudaFree(_d_dir_face_normals);

	cudaFree(_d_adj_face_to_vertices);
	cudaFree(_d_adj_vertex_to_faces);
	cudaFree(_d_adj_spring_st);
	cudaFree(_d_adj_spring_bd);


#ifdef DEBUG_COLLISION
	cudaFree(collided_vertex);
#endif
}

void Simulator::update_cloth(const Cloth &cloth)
{
	//register vbo
	safe_cuda(cudaGraphicsGLRegisterBuffer(
		&_d_vbo_resource, cloth.vbo.array_buffer, cudaGraphicsMapFlagsWriteDiscard));

	_num_textures = cloth.texures.size();
	_num_vertices = cloth.vertices.size();
	_num_faces = cloth.faces.size();

	if (_size_vertices < _num_vertices || _size_faces < _num_faces)
	{
		_size_vertices = _num_textures;
		_size_faces = _num_faces;

		ref_renew();
		create_buffer();
	}

	// 每个点最大包含NUM_PER_VERTEX_ADJ_FACES个邻近面，不足者以UINT_MAX作为结束标志
	std::vector<unsigned int> vertex_adjfaces(_num_vertices * sim_parameter.NUM_PER_VERTEX_ADJ_FACES);
	cloth.get_vertex_adjface_matrix(vertex_adjfaces, sim_parameter.NUM_PER_VERTEX_ADJ_FACES, UINT_MAX);

	// 将相关数据传送GPU
	const size_t vertices_bytes = sizeof(glm::vec3) * _num_vertices;

	Vec3s cloth_v3(_num_vertices);
	for (size_t idx = 0; idx < _num_vertices; ++idx)
	{
		glm::vec4 v = cloth.vertices[idx];
		cloth_v3[idx] = glm::vec3(v.x, v.y, v.z);
	}

	safe_cuda(cudaMemcpy(_d_x_orignal, &cloth_v3[0], vertices_bytes, cudaMemcpyHostToDevice));
	safe_cuda(cudaMemcpy(_d_x_cur_in, &cloth_v3[0], vertices_bytes, cudaMemcpyHostToDevice));
	safe_cuda(cudaMemcpy(_d_x_lst_in, &cloth_v3[0], vertices_bytes, cudaMemcpyHostToDevice));

	//计算normal所需的数据：每个点邻接的面的索引 + 每个面的3个点的索引 + 以及所有点的索引（虽然OPENGL有该数据）
	const size_t vertices_index_bytes = sizeof(unsigned int) * _num_faces * 3;
	safe_cuda(cudaMemcpy(_d_adj_face_to_vertices, &cloth.faces[0], vertices_index_bytes, cudaMemcpyHostToDevice));

	//initilize to 0
	safe_cuda(cudaMemset(_d_dir_collision_force, 0, sizeof(glm::vec3) * _num_vertices));

	const size_t vertex_adjface_bytes = sizeof(unsigned int) * _num_vertices * sim_parameter.NUM_PER_VERTEX_ADJ_FACES;
	safe_cuda(cudaMemcpy(_d_adj_vertex_to_faces, &vertex_adjfaces[0], vertex_adjface_bytes, cudaMemcpyHostToDevice));

	//弹簧信息，即两级邻域点信息传送GPU
	_springs_builder.build(cloth, _d_adj_spring_st, _d_adj_spring_bd);

#ifdef DEBUG_COLLISION
	//debug
	// a safe_cuda(cudaMalloc((void**)&collided_vertex, sizeof(int) * _num_vertices));
	cudaMemset(collided_vertex, 0, sizeof(int) * _num_vertices);
	cpu_collided_veretx.resize(_num_vertices);
	updated_vertex.resize(_num_vertices);
	faces1 = cloth.faces1;
#endif
}

void Simulator::simulate()
{
	unsigned int num_threads, num_blocks;

	computeGridSize(_num_vertices, 512, num_blocks, num_threads);
	verlet << < num_blocks, num_threads >> > (
		_bvh_tree,
		_num_vertices,
		_d_x_cur_in, _d_x_lst_in, _d_x_cur_out, _d_x_lst_out, _d_x_orignal,
		_d_adj_spring_st, _d_adj_spring_bd,
		_d_dir_collision_force
#ifdef DEBUG_COLLISION
		, collided_vertex
#endif
		);

	safe_cuda(cudaDeviceSynchronize());

#ifdef DEBUG_COLLISION
	cudaMemcpy(&cpu_collided_veretx[0], collided_vertex, sizeof(int)*numParticles, cudaMemcpyDeviceToHost);
	cudaMemcpy(&updated_vertex[0], _d_vbo_vertices, sizeof(glm::vec4)*numParticles, cudaMemcpyDeviceToHost);
	cout << "*****collided veretx index************" << endl;
	for (int i = 0; i < cpu_collided_veretx.size(); i++)
	{
		if (cpu_collided_veretx[i] == 1)
			cout << i << "  ";
	}
	cout << endl;
#endif

	swap_buffer();
}

void Simulator::ccd()
{
	unsigned int num_threads, num_blocks;

	computeGridSize(_num_vertices, 512, num_blocks, num_threads);
	CCD << < num_blocks, num_threads >> > (
		_ccd_tree,
		_num_vertices,
		_d_x_cur_in, _d_x_lst_in, _d_x_cur_out, _d_x_lst_out, _d_x_orignal,
		_d_dir_collision_force
		);

	// stop the CPU until the kernel has been executed
	safe_cuda(cudaDeviceSynchronize());

	//debug
	//cudaMemcpy(&cpu_collided_veretx[0],collided_vertex,sizeof(int)*numParticles, cudaMemcpyDeviceToHost);
	//cudaMemcpy(&updated_vertex[0], _d_vbo_vertices,sizeof(glm::vec4)*numParticles, cudaMemcpyDeviceToHost);
	//cout << "*****collided veretx index************" << endl;
	//for (int i = 0; i < cpu_collided_veretx.size(); i++)
	//{
	//	if (cpu_collided_veretx[i] == 1)
	//		cout << i << "  ";
	//}
	//cout << endl;

	swap_buffer();
}

void Simulator::visulize()
{
	size_t num_bytes;
	safe_cuda(cudaGraphicsMapResources(1, &_d_vbo_resource, 0));
	safe_cuda(cudaGraphicsResourceGetMappedPointer((void **)&_d_vbo_vertices, &num_bytes, _d_vbo_resource));
	
	// 获取normal位置指针
	_d_vbo_normals = (glm::vec3*)((float*)_d_vbo_vertices + 4 * _num_vertices + 2 * _num_textures);   
	
	unsigned int num_threads, num_blocks;		
	computeGridSize(_num_faces, 512, num_blocks, num_threads);
	// _num_faces
	get_face_normal << <num_blocks, num_threads >> > (_num_faces, _d_x_cur_in, _d_adj_face_to_vertices, _d_dir_face_normals);
	safe_cuda(cudaDeviceSynchronize());

	computeGridSize(_num_vertices, 512, num_blocks, num_threads);
	show_vbo << <num_blocks, num_threads >> > (_num_vertices, _d_vbo_vertices, _d_vbo_normals, _d_x_cur_in, _d_adj_vertex_to_faces, _d_dir_face_normals);
	safe_cuda(cudaDeviceSynchronize());
	
	safe_cuda(cudaGraphicsUnmapResources(1, &_d_vbo_resource, 0));
}

void Simulator::swap_buffer()
{
	int tmp = _id_in;
	_id_in = _id_out;
	_id_out = tmp;

	_d_x_cur_in = _d_x_cur[_id_in];
	_d_x_lst_in = _d_x_lst[_id_in];
	_d_x_cur_out = _d_x_cur[_id_out];
	_d_x_lst_out = _d_x_lst[_id_out];

}

#ifdef DEBUG_COLLISION

void Simulator::draw_collided_vertex()
{

	//draw outline first
	for (int i = 0; i < _num_faces; i++)
	{
		glm::vec4 ver[3];
		glm::vec3 normal[3];
		for (int j = 0; j < 3; j++)
		{
			ver[j] = updated_vertex[faces1[i].vertex_index[j]];
		}
		glPointSize(1.0);
		glBegin(GL_MODE);
		glColor3f(1.0, 1.0, 1.0);
		for (int j = 0; j < 3; j++)
		{
			glVertex3f(ver[j].x, ver[j].y, ver[j].z);
		}

		glEnd();
	}


	for (int i = 0; i < cpu_collided_veretx.size(); i++)
	{
		glm::vec4 v = updated_vertex[i];
		if (cpu_collided_veretx[i] == 1)
		{
			//draw it
			glPointSize(10.0);
			glBegin(GL_POINTS);
			glColor3f(1.0, 0, 0);
			glVertex3f(v.x, v.y, v.z);
			glEnd();
		}
	}
}

#endif