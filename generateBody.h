#pragma once

#include <iostream>
#include <armadillo>
#include <vector>
#include <string>
#include <fstream>
#include <map>
#include <time.h>
#include <glm/glm.hpp>

#include "Common.h"
#include "Mesh.h"
#include "ObjLoader.h"

//#pragma comment(lib, "libmat.lib")
//#pragma comment(lib,"libmx.lib")
//#pragma comment(lib, "libmex.lib  ")
//#pragma comment(lib,"libeng.lib ")
#pragma comment(lib, "blas_win64_MT.lib  ")
#pragma comment(lib,"lapack_win64_MT.lib ")

using namespace std;
using namespace arma;

#define NEUTRAL 0
#define MALE 1
#define FEMALE -1

class SMPL {
public:
	mat f, JJ, J_regressor, kintree_table, posedirs, shapedirs, v_template, weights, n_template, Kinect_J_template;
	uword joints_num, pose_num, shape_num, vert_num, face_num;
	SMPL(int gender = NEUTRAL);
	void write_to_obj(mat &v, string fname);
	mat gen_pose_model(mat &pose, bool want_norm = false);
	mat gen_full_model(mat &pose, mat &shape, bool want_norm = false);

	mat J_to_pose(mat J);
	mat Exp(mat &w);
	mat vector_to_mat(vector<mat> res);
	mat compute_n(mat j1, mat j2, mat j3);
	mat compute_t(mat x_t, mat x);
	mat R_to_t(mat R);
	mat global_rigid_transformation(mat &pose, mat &J);
	mat verts_core(mat &pose, mat &v, mat &J, bool want_norm = false);
	void arma_to_GL(mat &v, Mesh &body);
private:
	mat kpart;
	map<int, string> idd;
	map<int, int> id_to_col, parent;
	mat R0;
	mat with_zeros(mat &A);
	mat pack(mat &A);
};

void genBody(mat JJ, Mesh &body,SMPL &obj);
void genBodyVector(mat JJ, Vec4s &vertices, Vec3s &normals, SMPL &obj);
void arma_to_GL(mat &v, Mesh &body);
void arma_to_vector(mat &v, Vec4s &vertices, Vec3s &normals);
void genFirstBody(mat pose, Vec4s &vertices, Vec3s &normals, SMPL &obj);
