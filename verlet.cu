
#include <iostream>

#include <cuda.h>
#include <device_functions.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <glm/glm.hpp>

#include "Common.h"
#include "parameter.h"
#include "./bvh/BVHAccel.h"
#include "Verlet.h"


//physics parameter,若要修改参数，还需同时修改parameter.cpp
__device__ float spring_structure = 30.0f;
__device__ float spring_bend = 1.0f;
__device__ float damp = -0.05f;  //改变该值确实可以减小抖动！！
__device__ float mass = 0.3f;
__device__ float g = 0.000981/3.0f;
__device__ float dt = 1.0f /25.0f;
__device__ unsigned int NUM_PER_VERTEX_ADJ_FACES = 20;
__device__ unsigned int NUM_PER_VERTEX_SPRING_STRUCT = 20;
__device__ unsigned int NUM_PER_VERTEX_SPRING_BEND = 20;
__device__ float ccd_coef = 0.1f;  //0~1，可以调整ccd碰撞系数
__device__ float response = 0.001f;

__device__ uint32 Mode = 4;    // 0 for normal penalty ;1 for weight center penalty ; 2 for projection penalty ; 3 for weighted penalty
///////////////////////////////////////////////////////
__device__ void collision_response(
	const BVHAccel &tree,
	glm::vec3 &force, glm::vec3 &pos_cur, glm::vec3 &pos_lst)
{
	int idx_pri;
	bool inter = tree.intersect(pos_cur, idx_pri);
	if (inter)     
	{
		glm::vec3 normal;
		float dist = tree.curpri(idx_pri).distance_to(pos_cur, normal);
		if (dist < 0)
		{
			dist = 8.0 * glm::abs(dist);    //collision response with penalty force
			glm::vec3 temp = dist * normal;
			force = force + temp;
			pos_lst = pos_cur;
		}
	}
}

__device__ void collision_response_projection(
	const BVHAccel &tree, 
	glm::vec3 &force, glm::vec3 &pos_cur, glm::vec3 &pos_lst,
	int idx, glm::vec3 *dir_collision_force)
{
	int idx_pri;
	bool inter = tree.intersect(pos_cur, idx_pri);
	if (inter)
	{
		glm::vec3 normal;
		float dist = tree.curpri(idx_pri).distance_to(pos_cur, normal);

		// 这里每次需要计算normal，是否可保存在primitive中
		// 会不会影响到gpu计算时批量读取内存速度（primitive变大）
		if (dist < 0)
		{
			dist = glm::abs(dist)+ response;    // //collision response with penalty position
			pos_cur += dist * normal;

			pos_lst = pos_cur;
			dir_collision_force[idx] = normal;
		}
		else
			dir_collision_force[idx] = glm::vec3(0.0);

	}
	else
		dir_collision_force[idx] = glm::vec3(0.0);

}

__device__ void ccd_response_projection(
	const BVHAccel &tree,
	glm::vec3 &pos_cur, glm::vec3 &pos_lst,
	int idx, glm::vec3 *dir_collision_force)
{
	int idx_pri;
	//bool inter = intersect(leaf_nodes, internal_nodes, pos_cur, idx_pri);
	//bool inter = nearestIntersect(leaf_nodes, internal_nodes, pos_cur, idx_pri);
	bool inter = tree.coplanarIntersect(pos_cur, idx_pri);
	if (inter)
	{
		glm::vec3 normal;
		float dist = tree.curpri(idx_pri).distance_to(pos_cur, normal);
		if (dist < 0)
		{
			pos_lst = pos_cur;
			if (Mode==0)
			{
				dist = glm::abs(dist) + 0.02f;    //collision response with penalty displacement
				pos_cur += dist * normal;
			}
			else if(Mode == 1)	
				pos_cur += tree.curpri(idx_pri).get_center() - pos_cur;
			else if(Mode == 2){
				glm::vec3 PriToPoint = pos_cur - tree.lstpri(idx_pri).v0();
				float d = glm::dot(PriToPoint , tree.lstpri(idx_pri).get_normal());
				glm::vec3 ProjectInPri = pos_cur - d * tree.lstpri(idx_pri).get_normal();
				glm::vec3 dx0 = ProjectInPri - tree.lstpri(idx_pri).v0();
				glm::vec3 dx1 = ProjectInPri - tree.lstpri(idx_pri).v1();
				glm::vec3 dx2 = ProjectInPri - tree.lstpri(idx_pri).v2();
				pos_cur = dx0 + tree.curpri(idx_pri).v0()
					+ dx1 + tree.curpri(idx_pri).v1()
					+ dx2 + tree.curpri(idx_pri).v2();
				pos_cur /= 3.0f;
			}
			else if (Mode == 3)
			{
				glm::vec3 dx0 = pos_cur - tree.lstpri(idx_pri).v0() ;
				glm::vec3 dx1 = pos_cur - tree.lstpri(idx_pri).v1();
				glm::vec3 dx2 = pos_cur - tree.lstpri(idx_pri).v2();
				pos_cur = dx0 + tree.curpri(idx_pri).v0()
					+ dx1 + tree.curpri(idx_pri).v1()
					+ dx2 + tree.curpri(idx_pri).v2();
				pos_cur /= 3.0f;
			}

			else if (Mode == 4) {
				glm::vec3 PriToPoint = pos_cur - tree.lstpri(idx_pri).v0();
				float distance = glm::dot(PriToPoint, tree.lstpri(idx_pri).get_normal());
				glm::vec3 ProjectInPri = pos_cur - distance*tree.lstpri(idx_pri).get_normal();
				float a = tree.lstpri(idx_pri).v1().x - tree.lstpri(idx_pri).v0().x;
				float b = tree.lstpri(idx_pri).v2().x - tree.lstpri(idx_pri).v0().x;
				float c = tree.lstpri(idx_pri).v1().y - tree.lstpri(idx_pri).v0().y;
				float d = tree.lstpri(idx_pri).v2().y - tree.lstpri(idx_pri).v0().y;
				float x = ProjectInPri.x - tree.lstpri(idx_pri).v0().x;
				float y	= ProjectInPri.y - tree.lstpri(idx_pri).v0().y;
				float lamda1 = (d * x - b * y) / (a * d - b * c);
				float lamda2 = (c * x - a * y) / (b * c - a * d);

				//cout<<lamda1<<','<<lamda2<<std::endl;
				pos_cur = tree.curpri(idx_pri).v0()
					+ lamda1 * (tree.curpri(idx_pri).v1() - tree.curpri(idx_pri).v0())
					+ lamda2 * (tree.curpri(idx_pri).v2() - tree.curpri(idx_pri).v0())
					+ glm::abs(distance) * tree.curpri(idx_pri).get_normal();
			}
			
			pos_lst = pos_lst + (pos_cur - pos_lst) * (1.0f-ccd_coef);

			dir_collision_force[idx] = glm::vec3(0.0);
		}
		else
			dir_collision_force[idx] = glm::vec3(0.0);
		/*
		tree.curpri(idx_pri).intersect(pos_cur, dist, normal);
		dist = abs(dist);
		pos_lst = pos_cur;
		dist = dist + 0.001;    //collision response with penalty displacement
		pos_cur += dist*normal;
		pos_lst = pos_lst + (pos_cur - pos_lst)*(1.0f - ccd_coef);
		dir_collision_force[idx] = glm::vec3(0.0);*/

	}
	else
		dir_collision_force[idx] = glm::vec3(0.0);

}

//debug
__device__ bool collide(
	const BVHAccel &tree,
	glm::vec3 &force, glm::vec3 &pos_cur, glm::vec3 &pos_lst,
	int idx, glm::vec3 *dir_collision_force)
{
	int idx_pri;
	bool inter = tree.intersect(pos_cur, idx_pri);
	if (inter)
	{
		glm::vec3 normal;
		float dist = tree.curpri(idx_pri).distance_to(pos_cur, normal);
		if (dist < 0)
		{
			return true;
		}
		else
			return false;

	}
	else
		return false;
}

__global__ void get_face_normal(
	const unsigned int num_faces,
	glm::vec3 *x_cur_in,
	unsigned int *adj_face_to_vertices,
	glm::vec3 *dir_face_normals)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	unsigned int max_thread = num_faces;
	if (index >= max_thread)
		return;


	unsigned int i0 = index * 3;
	unsigned int i1 = i0 + 1;
	unsigned int i2 = i0 + 2;

	i0 = adj_face_to_vertices[i0];
	i1 = adj_face_to_vertices[i1];
	i2 = adj_face_to_vertices[i2];

	glm::vec3 v0 = x_cur_in[i0];
	glm::vec3 v1 = x_cur_in[i1];
	glm::vec3 v2 = x_cur_in[i2];

	glm::vec3 side1 = v1 - v0;
	glm::vec3 side2 = v2 - v0;
	glm::vec3 normal = glm::normalize(glm::cross(side1, side2));

	dir_face_normals[index] = normal;

}

__device__ glm::vec3 get_spring_force(
	int index, 
	glm::vec3 *x_cur_in, glm::vec3 *x_lst_in, glm::vec3 *x_orignal,
	unsigned int *adj_spring, unsigned int num_per_vertex_spring, 
	glm::vec3 pos_cur, glm::vec3 vel, float k_spring)
{
	glm::vec3 force(0.0);
	unsigned int first_neigh = index * num_per_vertex_spring;   //访问一级邻域，UINT_MAX为截至标志
	unsigned int time = 0;
	for (unsigned int k = first_neigh; 
		adj_spring[k] < UINT_MAX && time < num_per_vertex_spring; 
		k++, time++) //部分点邻域大于MAX_NEIGH(20)
	{
		float ks = k_spring;
		float kd = 0;

		int index_neigh = adj_spring[k];
		// volatile glm::vec3 p2_cur = x_cur_in[index_neigh];
		// volatile glm::vec3 p2_lst = x_lst_in[index_neigh];
		glm::vec3 p2_cur = x_cur_in[index_neigh];
		glm::vec3 p2_lst = x_lst_in[index_neigh];

		glm::vec3 v2 = (p2_cur - p2_lst) / dt;
		glm::vec3 deltaP = pos_cur - p2_cur;
		if (glm::length(deltaP) == 0) { force += glm::vec3(0.0f); continue; }  //deltaP += glm::vec3(0.0001);	//avoid '0'

		glm::vec3 deltaV = vel - v2;
		float dist = glm::length(deltaP); //avoid '0'


		float original_length = glm::distance(x_orignal[index_neigh],x_orignal[index]);
		float leftTerm = -ks * (dist - original_length);
		float rightTerm = kd * (glm::dot(deltaV, deltaP) / dist);
		glm::vec3 springForce = (leftTerm + rightTerm)*glm::normalize(deltaP);
		
		force += springForce;
	}
	return force;

}

__global__ void show_vbo(
	const unsigned int num_vertices,
	glm::vec4 *vbo_vertices, glm::vec3 *vbo_normals,
	glm::vec3 *x, unsigned int *adj_vertex_to_faces, glm::vec3 *dir_face_normals)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= num_vertices)
		return;

	// volatile glm::vec3 pose = x[index];
	glm::vec3 pose = x[index];

	// compute point normal
	glm::vec3 normal(0.0);

	int first_face_index = index * NUM_PER_VERTEX_ADJ_FACES;
	for (unsigned int i = first_face_index, time = 0;
		adj_vertex_to_faces[i] < UINT_MAX && time < NUM_PER_VERTEX_ADJ_FACES;
		++i, ++time)
	{
		int findex = adj_vertex_to_faces[i];
		glm::vec3 fnormal = dir_face_normals[findex];
		normal += fnormal;
	}
	normal = glm::normalize(normal);

	//set new vertex and new normal
	vbo_vertices[index] = glm::vec4(pose.x, pose.y, pose.z, vbo_vertices[index].w);
	vbo_normals[index] = glm::vec3(normal.x, normal.y, normal.z);
}

__global__ void verlet(
	const BVHAccel tree,
	const unsigned int num_vertices,
	glm::vec3 *x_cur_in, glm::vec3 *x_lst_in, glm::vec3 *x_cur_out, glm::vec3 *x_lst_out, glm::vec3 *x_orignal,
	unsigned int *adj_spring_st, unsigned int *adj_spring_bd,
	glm::vec3 *dir_collision_force
#ifdef DEBUG_COLLISION
	, int *collided_vertex
#endif
)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= num_vertices)
		return;

	// volatile glm::vec3 pos_cur = x_cur_in[index];
	// volatile glm::vec3 pos_lst = x_lst_in[index];

	glm::vec3 pos_cur = x_cur_in[index];
	glm::vec3 pos_lst = x_lst_in[index];

	glm::vec3 vel = (pos_cur - pos_lst) / dt;

	const glm::vec3 gravity = glm::vec3(0.0f, -1.0 * g, 0.0f); //set gravity
	glm::vec3 force = gravity * mass + vel * damp;
	force += get_spring_force(index, x_cur_in, x_lst_in, x_orignal, adj_spring_st, NUM_PER_VERTEX_SPRING_STRUCT, pos_cur, vel, spring_structure); //计算一级邻域弹簧力
	force += get_spring_force(index, x_cur_in, x_lst_in, x_orignal, adj_spring_bd, NUM_PER_VERTEX_SPRING_BEND, pos_cur, vel, spring_bend); //计算二级邻域弹簧力

	glm::vec3 inelastic_force = glm::dot(dir_collision_force[index], force) * dir_collision_force[index];       //collision response force, if intersected, keep tangential
																												//inelastic_force *= 0.5;
	force -= inelastic_force;
	glm::vec3 acc = force / mass;
	glm::vec3 tmp = pos_cur;
	pos_cur = pos_cur + pos_cur - pos_lst + acc * dt * dt;
	pos_lst = tmp;

	// 这里pose_old已经为以前的pose了
	// 经过collision_response_projection之后
	// pose_old又变成了现在的pose，pos_lst = pos_cur;
	// 弹簧过程造成的pose变化则被覆盖
	// 这会不会影响vel的计算？
	// 是特殊处理？为了防止碰撞后反弹？
	// 也因为这个机制的存在，导致必须用四个内存，而不是三个
	// 一面影响把改动后的old写入x_cur_in，导致其他线程计算邻居位移时出错？
	collision_response_projection(tree, force, pos_cur, pos_lst, index, dir_collision_force);

#ifdef DEBUG_COLLISION
	// debug
	if (collide(leaf_nodes, internal_nodes, tree.primitives_cur(), force, pos_cur, pos_lst, index, dir_collision_force))
	{
		collided_vertex[index] = 1;
	}
	else
	{
		collided_vertex[index] = 0;
	}
#endif
	x_cur_out[index] = pos_cur;
	x_lst_out[index] = pos_lst;
}

__global__ void CCD(
	const BVHAccel tree,
	const unsigned int num_vertices,
	glm::vec3 *x_cur_in, glm::vec3 *x_lst_in, glm::vec3 *x_cur_out, glm::vec3 *x_lst_out, glm::vec3 *x_orignal,
	glm::vec3 *dir_collision_force)
{
	unsigned int index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= num_vertices)
		return;

	// volatile glm::vec3 pos_cur = x_cur_in[index];
	// volatile glm::vec3 pos_lst = x_lst_in[index];
	glm::vec3 pos_cur = x_cur_in[index];
	glm::vec3 pos_lst = x_lst_in[index];

	//glm::vec3 vel = (pos_cur - pos_lst) / dt;

	glm::vec3 force;

	ccd_response_projection(tree, pos_cur, pos_lst, index, dir_collision_force);

	x_cur_out[index] = pos_cur;
	x_lst_out[index] = pos_lst;
}
