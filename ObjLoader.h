#pragma once
#include <list>
#include <vector>
#include <string>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <glm/glm.hpp>

#include "Common.h"
#include "Mesh.h"

class ObjLoader
{
private:
	typedef std::vector<unsigned int> F2VIndices;
	typedef std::vector<unsigned int> F2TIndices;
	typedef std::vector<unsigned int> F2NIndices;

	typedef void (ObjLoader::*Parser)(Mesh &mesh, std::string token[4], std::string &buffer);
	struct PaserItem
	{
		int id;
		std::string cond;
		Parser parser;
	};

	typedef std::list<PaserItem> Parsers;
	
public:
	ObjLoader(); 
	void load(Mesh &mesh, const std::string &file);

private:
	void install_parser(int id, const std::string &cond, Parser parser);
	void parse(Mesh &mesh, std::string token[4], std::string &buffer);
	
	// unify the data, so that one vertex -> one normal -> one texture, 
	// or error acurred while rendering
	void unified(Mesh &mesh);

	void parse_mt(Mesh &mesh, std::string token[4], std::string &buffer); 
	void parse_obj_name(Mesh &mesh, std::string token[4], std::string &buffer);
	void parse_obj_mesh(Mesh &mesh, std::string token[4], std::string &buffer);
	void parse_fgroup_name(Mesh &mesh, std::string token[4], std::string &buffer);
	void parse_fgroup_mesh(Mesh &mesh, std::string token[4], std::string &buffer);
	void parse_v(Mesh &mesh, std::string token[4], std::string &buffer);
	void parse_vn(Mesh &mesh, std::string token[4], std::string &buffer);
	void parse_vt(Mesh &mesh, std::string token[4], std::string &buffer);
	void parse_f(Mesh &mesh, std::string token[4], std::string &buffer); 

	void parse_mt_file(Mesh &mesh);
	void parse_texure_file(Mesh &mesh);

private:
	F2VIndices _v_indices;
	F2TIndices _t_indices;
	F2NIndices _n_indices;

	std::string _path; 
	std::string _texfile;
	std::string _mtlfile;

	Parsers _parsers;
};


