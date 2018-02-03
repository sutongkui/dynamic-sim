
#include "scene.h"
#include "Mesh.h"
#include "ObjLoader.h"

bool DoudouHead_Merge::Enabled (false);
GLfloat DoudouHead_Merge::model[16];
DoudouHead_Merge::DoudouHead_Merge()
{

}
void DoudouHead_Merge::Init(Mesh& _Pts_Head_obj)
{
	Pts_Body_Cor.load("../DoudouHead/Neck_IdxBody.txt");
	Pts_Head_Cor.load("../DoudouHead/Neck_IdxMerge.txt");
	Len = Pts_Body_Cor.n_elem;
	Ones = ones<mat>(Len, 1);
	Pts_Head = zeros<mat>(Len, 3);
	for (int i = 0; i < Len; i++)
	{
		Pts_Head(i, 0) = _Pts_Head_obj.vertices[Pts_Head_Cor(i)].x;
		Pts_Head(i, 1) = _Pts_Head_obj.vertices[Pts_Head_Cor(i)].y;
		Pts_Head(i, 2) = _Pts_Head_obj.vertices[Pts_Head_Cor(i)].z;
	}
	Pts_Head = join_horiz(Pts_Head, Ones);
}
DoudouHead_Merge::DoudouHead_Merge(Mesh& _Pts_Head_obj)
{
	Init(_Pts_Head_obj);
}
GLfloat *DoudouHead_Merge::Calc(Vec4s& _Pts_Body_vec)
{
	Enabled = true;
	Pts_Body = zeros<mat>(Len, 3);
	for (int i = 0; i < Len; i++)
	{
		Pts_Body(i, 0) = _Pts_Body_vec[size_t(Pts_Body_Cor(i))].x;
		Pts_Body(i, 1) = _Pts_Body_vec[size_t(Pts_Body_Cor(i))].y;
		Pts_Body(i, 2) = _Pts_Body_vec[size_t(Pts_Body_Cor(i))].z;
	}
	mat solved = solve(Pts_Head, join_horiz(Pts_Body, Ones)).t();

	mat U, V;
	vec s;
	svd(U, s, V, solved.submat(0,0,2,2));
	float s_value = mean(s) * 1.08;
	mat sR = s_value*U*V.t();
	mat T = mean(Pts_Body - Pts_Head.cols(0, 2) * sR.t(), 0).t();

	mat H = join_vert(join_horiz(sR, T), join_horiz(zeros(1, 3), ones(1, 1)));


	//cout << solved << endl;
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			model[i+4*j] = H(i, j);

	return model;
}