#pragma once
#include <cuda.h>
#include <device_functions.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <glm/glm.hpp>

class BRTreeNode;
class Primitive;

//update cloth face normal
__global__ void get_face_normal(
	const unsigned int num_faces,
	glm::vec3 *x_cur_in,
	unsigned int *adj_face_to_vertices,
	glm::vec3 *dir_face_normals);

__global__ void show_vbo(
	const unsigned int num_vertices,
	glm::vec4 *vbo_vertices, glm::vec3 *vbo_normals,
	glm::vec3 *x, unsigned int *adj_vertex_to_faces, glm::vec3 *dir_face_normals);

__global__ void CCD(
	const BVHAccel tree,
	const unsigned int num_vertices,
	glm::vec3 *x_cur_in, glm::vec3 *x_lst_in, glm::vec3 *x_cur_out, glm::vec3 *x_lst_out, glm::vec3 *x_orignal,
	glm::vec3 *dir_collision_force);

// verlet intergration
__global__ void verlet(
	const BVHAccel tree,
	const unsigned int num_vertices,
	glm::vec3 *x_cur_in, glm::vec3 *x_lst_in, glm::vec3 *x_cur_out, glm::vec3 *x_lst_out, glm::vec3 *x_orignal,
	unsigned int *adj_spring_st, unsigned int *adj_spring_bd,
	glm::vec3 *dir_collision_force
#ifdef DEBUG_COLLISION
	, int *collided_vertex
#endif
);
