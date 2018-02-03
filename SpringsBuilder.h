#pragma once
#include <set>
#include <vector>
#include <map>

#include "parameter.h"

class Cloth; 

// allanyu 此文件相关代码尚未优化
class SpringsBuilder
{
public:
	// allanyu 回头再看
	// void draw(Obj &cloth);

	bool build(const Cloth &cloth, unsigned int *adj_spring_st, unsigned int *adj_spring_bd);
private:
	std::vector<std::pair<unsigned int,unsigned int>> cloth_boundary_springs;   //只包含pair(1,2)
	std::vector<std::pair<unsigned int,unsigned int>> boundary_boundary_springs;   //应该已经包含pair(1,2) && pair(2,1)
	std::set<std::pair<unsigned int,unsigned int>> boundary;
	std::vector<std::vector<unsigned int>> neigh1;   //存储每个点的所有一级邻域信息(存储点的索引),即 structure spring
	std::vector<std::vector<unsigned int>> neigh2;   //存储每个点的所有二级邻域信息(存储点的索引),即 bend spring

private:
	//void ad spring(float stiffness,Vec4s& vertices,unsigned int p1,unsigned int p2);
	bool exist(const std::vector<unsigned int>& array, const unsigned int val);
	void get_cloth_boundary_spring(const Cloth &cloth);
	void get_boundary_boundary_spring(const Cloth &cloth);
};