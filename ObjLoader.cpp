#include <iostream>
#include <fstream>
#include <sstream>

#include <GL/glew.h>
#include <GL/freeglut.h>

#include <FreeImage.h>

#include "ObjLoader.h"

using namespace std;

ObjLoader::ObjLoader()
{
	install_parser(0, "mtllib", &ObjLoader::parse_mt);
	install_parser(1, "object", &ObjLoader::parse_obj_name);
	install_parser(2, "mesh.vertices", &ObjLoader::parse_obj_mesh);
	install_parser(0, "v", &ObjLoader::parse_v);
	install_parser(0, "vn", &ObjLoader::parse_vn);
	install_parser(0, "vt", &ObjLoader::parse_vt);
	install_parser(0, "g", &ObjLoader::parse_fgroup_name);
	install_parser(2, "polygons", &ObjLoader::parse_fgroup_mesh);
	install_parser(0, "f", &ObjLoader::parse_f);
}

void ObjLoader::parse(Mesh &mesh, std::string token[4], std::string &buffer)
{
	for (auto &item : _parsers)
	{
		if (token[item.id] == item.cond)
		{
			(this->*item.parser)(mesh, token, buffer);
			return; 
		}
	}
}

void ObjLoader::install_parser(int id, const std::string &cond, Parser parser)
{
	PaserItem item;
	item.id = id;
	item.cond = cond;
	item.parser = parser;

	_parsers.push_back(item);
}

void ObjLoader::parse_mt(Mesh &mesh, std::string token[4], std::string &buffer)
{
	_mtlfile = token[1];
}

void ObjLoader::parse_obj_name(Mesh &mesh, std::string token[4], std::string &buffer)
{
	mesh.objects.push_back(make_pair(token[2], 0));
}

void ObjLoader::parse_obj_mesh(Mesh &mesh, std::string token[4], std::string &buffer)
{
	mesh.objects.back().second = atoi(token[1].c_str());
}

void ObjLoader::parse_fgroup_name(Mesh &mesh, std::string token[4], std::string &buffer)
{
	mesh.fgroups.push_back(make_pair(token[1], 0));
}

void ObjLoader::parse_fgroup_mesh(Mesh &mesh, std::string token[4], std::string &buffer)
{
	stringstream tem_line(buffer);
	string	f00, f11, f22, f33, f44;
	tem_line >> f00 >> f11 >> f22 >> f33 >> f44;
	mesh.fgroups.back().second = atoi(f44.c_str());
}

void ObjLoader::parse_v(Mesh &mesh, std::string token[4], std::string &buffer)
{
	glm::vec4 ver(atof(token[1].c_str()), atof(token[2].c_str()), atof(token[3].c_str()), 1.0);
	mesh.vertices.push_back(ver);
}
void ObjLoader::parse_vn(Mesh &mesh, std::string token[4], std::string &buffer)
{
	glm::vec3 nor(atof(token[1].c_str()), atof(token[2].c_str()), atof(token[3].c_str()));
	mesh.normals.push_back(nor);
}
void ObjLoader::parse_vt(Mesh &mesh, std::string token[4], std::string &buffer)
{
	glm::vec2 tex_coords(atof(token[1].c_str()), atof(token[2].c_str()));
	mesh.texures.push_back(tex_coords);
}
void ObjLoader::parse_f(Mesh &mesh, std::string token[4], std::string &buffer)
{
	Mesh::Face face;
	unsigned int fi[3];
	unsigned int ti[3];
	unsigned int ni[3];

	size_t sPos = 0;
	size_t ePos = sPos;
	string temp;

	ePos = token[1].find_first_of("/");
	if (ePos == string::npos)  //处理不同格式的face, f  1 2 3
	{
		fi[0] = atoi(token[1].c_str()) - 1;
		fi[1] = atoi(token[2].c_str()) - 1;
		fi[2] = atoi(token[3].c_str()) - 1;

		ti[0] = 0;     //add default mesh.texures
		ti[1] = 1;
		ti[2] = 2;

		ni[0] = atoi(token[1].c_str()) - 1;
		ni[1] = atoi(token[2].c_str()) - 1;
		ni[2] = atoi(token[3].c_str()) - 1;

	}
	else     //处理不同格式的face, f  1/1/1 2/2/2 3/3/3
	{
		for (int i = 0; i < 3; ++i)
		{
			sPos = 0;
			ePos = token[i + 1].find_first_of("/");
			assert(ePos != string::npos);
			temp = token[i + 1].substr(sPos, ePos - sPos);
			fi[i] = atoi(temp.c_str()) - 1;

			sPos = ePos + 1;
			ePos = token[i + 1].find("/", sPos);
			assert(ePos != string::npos);
			temp = token[i + 1].substr(sPos, ePos - sPos);
			ti[i] = atoi(temp.c_str()) - 1;

			sPos = ePos + 1;
			ePos = token[i + 1].length();
			temp = token[i + 1].substr(sPos, ePos - sPos);
			ni[i] = atoi(temp.c_str()) - 1;
		}
	}

	face.v0 = fi[0];
	face.v1 = fi[1];
	face.v2 = fi[2];
	mesh.faces.push_back(face);
	
	for (int i = 0; i < 3; ++i)
	{
		_v_indices.push_back(fi[i]);
		_t_indices.push_back(ti[i]);
		_n_indices.push_back(ni[i]);
	}
}

void ObjLoader::parse_mt_file(Mesh &mesh)
{
	string f0, f1;

	if (_mtlfile.empty())
	{
		return;
	}
	// read material, here we just read the texture and only 1 photo
	_mtlfile = _path + _mtlfile;
	ifstream input(_mtlfile);
	if (!input)
	{
		cerr << "error: unable to open input file: " << endl;
		exit(-1);
	}

	while (!input.eof())
	{
		string buffer;
		getline(input, buffer);
		stringstream line(buffer);
		line >> f0 >> f1;
		if (f0 == "map_Ka")
		{
			_texfile = f1;
			break;
		}
	}
}

void ObjLoader::parse_texure_file(Mesh &mesh)
{
	if (_texfile.empty()) // load .png and you need to initilize opengl before load
	{
		return; 
	}

	_texfile = _path + _texfile;
	FIBITMAP *dib = FreeImage_Load(FIF_PNG, _texfile.c_str(), PNG_DEFAULT);

	if (FreeImage_GetBPP(dib) != 32)
	{
		FIBITMAP *tempImage = dib;
		dib = FreeImage_ConvertTo32Bits(tempImage);
	}

	if (dib == NULL)
	{
		return; 
	}

	glGenTextures(1, &mesh.gl_texture);
	glBindTexture(GL_TEXTURE_2D, mesh.gl_texture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	BYTE *bits = new BYTE[FreeImage_GetWidth(dib) * FreeImage_GetHeight(dib) * 4];
	BYTE *pixels = (BYTE*)FreeImage_GetBits(dib);

	for (unsigned int pix = 0; pix < FreeImage_GetWidth(dib) * FreeImage_GetHeight(dib); pix++)
	{
		bits[pix * 4 + 0] = pixels[pix * 4 + 2];
		bits[pix * 4 + 1] = pixels[pix * 4 + 1];
		bits[pix * 4 + 2] = pixels[pix * 4 + 0];
		bits[pix * 4 + 3] = pixels[pix * 4 + 3];
	}

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, FreeImage_GetWidth(dib), FreeImage_GetHeight(dib), 0, GL_RGBA, GL_UNSIGNED_BYTE, bits);

	FreeImage_Unload(dib);
	delete bits;
}

void ObjLoader::unified(Mesh &mesh)
{
	Vec2s uni_tex(mesh.vertices.size());
	Vec3s uni_normals(mesh.vertices.size());

	for (int i = 0; i < _v_indices.size(); ++i)
	{
		uni_tex[_v_indices[i]] = mesh.texures[_t_indices[i]];
		uni_normals[_v_indices[i]] = mesh.normals[_n_indices[i]];
	}

	mesh.texures.swap(uni_tex);
	mesh.normals.swap(uni_normals);
}

void ObjLoader::load(Mesh &mesh, const std::string &file)
{
	mesh.clear();
	_v_indices.clear();
	_t_indices.clear();
	_n_indices.clear();

	_path = file.substr(0, file.find_last_of("/") + 1);
	string f[4];

	ifstream input(file);
	if (!input)
	{
		cout << "error: unable to open input file: " << endl;
		exit(-1);
	}

	while (!input.eof())
	{
		string buffer;
		getline(input, buffer);
		stringstream line(buffer);
		line >> f[0] >> f[1] >> f[2] >> f[3];

		parse(mesh, f, buffer); 
	}

	parse_mt_file(mesh); 
	parse_texure_file(mesh); 

	unified(mesh); 
}
