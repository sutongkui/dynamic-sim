#pragma once
#include <vector>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include <glm/glm.hpp>

#include "SpringsBuilder.h"
#include "bvh\BVHAccel.h"
#include "bvh\BVHBuilder.h"

class Cloth;
class Mesh;
class BRTreeNode;
class Primitive;

class Simulator : public RefObject
{
public:
	// 用于赋予Simulation用的衣物
	Simulator();
	~Simulator();

	// 一般情况下做静态碰撞
	void simulate();
	// 人体更新位置时做ccd碰撞
	void ccd();

	// 显示结果
	void visulize();

	// debug
	void draw_collided_vertex();

	// 初始各种模拟时需要用到的参数空间，规模和衣物规模有关，与人体无关
	void update_cloth(const Cloth &cloth);
	void update_body(const Mesh &body);

private:
	virtual void ref_auto_clean();

	void create_buffer();

	// 静态碰撞具体算法
	void verlet_cuda();

	// 动态碰撞具体算法
	void ccd_cuda();

	void computeGridSize(unsigned int n, unsigned int block_size, unsigned int &num_blocks, unsigned int &num_threads)
	{
		num_threads = std::min(block_size, n);
		num_blocks = (n % num_threads != 0) ? (n / num_threads + 1) : (n / num_threads);
	}

	// 交换输入、输出空间
	// 每组两个，当前位置、上一次位置
	// 本可用三个，而不是2 * 2 = 4 个
	// 但因在计算过程中，因碰撞可能导致下一次的pose_last != 这一次的pose
	// 如不将pose_last数据，另开空间存储，则会在计算时，直接影响这一次的pose
	// 而这一次的pose，会被其他线程计算其他节点时用到
	void swap_buffer();

private:
	//////////////////bvh数据///////////////////
	BVHBuilder _bvh_builder;
	BVHBuilder _ccd_builder;
	BVHAccel _bvh_tree;
	BVHAccel _ccd_tree;

	//////////////////cloth数据///////////////////
	SpringsBuilder _springs_builder;

	// 当前buffer容量
	unsigned int _size_faces;
	unsigned int _size_vertices;

	// 当前实际数量
	unsigned int _num_faces;
	unsigned int _num_vertices;
	unsigned int _num_textures;

	// 一共两组buffer，每组分别存储了当前位置和前一次位置，两组轮流做输入、输出
	// 该id会配合swap_buffer交换，从而实现输入、输出组的互换
	int _id_in, _id_out;

	// 以下数据全为显存指针
private:
	// 弹簧原长，衣物不变该数据永远不变
	glm::vec3 *_d_x_orignal;

	// 每次迭代的输入数据和输出数据指针，分当前值和上一次值
	// 无实际空间，通过指向_d_x_cur与_d_x_lst得到地址
	// 通过交换指向实现输入和输出组的轮换
	glm::vec3 *_d_x_cur_in, *_d_x_cur_out;
	glm::vec3 *_d_x_lst_in, *_d_x_lst_out;

	// 实际用于存储的空间，_d_x_cur_in等指向它们
	glm::vec3 *_d_x_cur[2];
	glm::vec3 *_d_x_lst[2];

	glm::vec3 *_d_dir_collision_force;		// 碰撞的人体三角形的法向量，如果没有碰撞则为0
	glm::vec3 *_d_dir_face_normals;		// 面的法向量

	// 三角形-顶点，邻接矩阵
	// 三角面片对应顶点的索引，每个三角面片有三个索引
	// 第i个三角面的第j个顶点，位置为i * 3 + j
	unsigned int *_d_adj_face_to_vertices;

	// 顶点-三角形，邻接矩阵
	// 每个顶点对应的三角面片，考虑每个顶点对应三角形个数不定
	// 统一开辟NUM_PER_VERTEX_ADJ_FACES个邻接三角面空间
	// 不足填UINT_MAX
	// 计算每个点法向量时需要其周围平面的索引
	unsigned int *_d_adj_vertex_to_faces;

	// 顶点-顶点，邻接矩阵
	// （注意）一阶领域 + 满足边界约束的不连接点
	// 第i个顶点的第j个邻居，位置为i * NUM_PER_VERTEX_SPRING_STRUCT + j
	// 不足填UINT_MAX
	unsigned int *_d_adj_spring_st;

	// 顶点-顶点，邻接矩阵
	// 相邻三角形不共享的那两个顶点，作为弯曲弹簧
	// 第i个顶点的第j个邻居，位置为i * NUM_PER_VERTEX_SPRING_BEND + j
	// 不足填UINT_MAX
	unsigned int *_d_adj_spring_bd;

	// 用于管理衣物绘制的opengl资源，
	// 通过cuda代码，直接在gpu中改写衣物用于显示的数据
	cudaGraphicsResource *_d_vbo_resource;
	glm::vec4 *_d_vbo_vertices;           // 指向OPENGL buffer中vertex的地址
	glm::vec3 *_d_vbo_normals;            // 指向OPENGL buffer中normal的地址


// 用于建立该衣服的几种弹簧，结果表现为_d_adj_spring_st与_d_adj_spring_bd
	// 将改变为局部变量，传入衣服时使用，构建好弹簧后销毁
		// Springs *cuda_spring;

#ifdef DEBUG_COLLISION
	// debug
	int *collided_vertex;
	std::vector<int> cpu_collided_veretx;
	Vec4s updated_vertex;
	vector<Face> faces;
#endif

};

