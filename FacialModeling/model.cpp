#include <iostream>
#include <fstream>
#include <sstream>

#include "gl/glew.h"
#include <gl/glut.h>

#include "model.h"

Model::Model(const char *filename, const char *fileopen) : verts_(), faces_(), norms_(), uv_(), verts_open_(), faces_open_(), norms_open_(), uv_open_(), diffusemap_(), normalmap_(), specularmap_() {
	std::ifstream in;
	in.open(filename, std::ifstream::in);
	if (in.fail()) return;
	std::string line;
	std::cerr << "reading obj" <<std::endl;
	while (!in.eof()) {
		std::getline(in, line);
		std::istringstream iss(line.c_str());
		char trash;
		if (!line.compare(0, 2, "v ")) {
			iss >> trash;
			vec3 v;
			for (int i = 0; i<3; i++) iss >> v[i];
			verts_.push_back(v);
		}
		else if (!line.compare(0, 3, "vn ")) {
			iss >> trash >> trash;
			vec3 n;
			for (int i = 0; i<3; i++) iss >> n[i];
			norms_.push_back(n);
		}
		else if (!line.compare(0, 3, "vt ")) {
			iss >> trash >> trash;
			vec2 uv;
			for (int i = 0; i<2; i++) iss >> uv[i];
			uv_.push_back(uv);
		}
		else if (!line.compare(0, 2, "f ")) {
			std::vector<vec3> f;
			vec3 tmp;
			iss >> trash;
			while (iss >> tmp[0] >> trash >> tmp[1] >> trash >> tmp[2]) {
				for (int i = 0; i<3; i++) tmp[i]--; // in wavefront obj all indices start at 1, not zero
				f.push_back(tmp);
			}
			faces_.push_back(f);
		}
	}
	std::cerr << "# v# " << verts_.size() << " f# " << faces_.size() << " vt# " << uv_.size() << " vn# " << norms_.size() << std::endl;

	in.close();

	in.open(fileopen, std::ifstream::in);
	if (in.fail()) return;
	std::cerr << "reading open obj" << std::endl;
	while (!in.eof()) {
		std::getline(in, line);
		std::istringstream iss(line.c_str());
		char trash;
		if (!line.compare(0, 2, "v ")) {
			iss >> trash;
			vec3 v;
			for (int i = 0; i<3; i++) iss >> v[i];
			verts_open_.push_back(v);
		}
		else if (!line.compare(0, 3, "vn ")) {
			iss >> trash >> trash;
			vec3 n;
			for (int i = 0; i<3; i++) iss >> n[i];
			norms_open_.push_back(n);
		}
		else if (!line.compare(0, 3, "vt ")) {
			iss >> trash >> trash;
			vec2 uv;
			for (int i = 0; i<2; i++) iss >> uv[i];
			uv_open_.push_back(uv);
		}
		else if (!line.compare(0, 2, "f ")) {
			std::vector<vec3> f;
			vec3 tmp;
			iss >> trash;
			while (iss >> tmp[0] >> trash >> tmp[1] >> trash >> tmp[2]) {
				for (int i = 0; i<3; i++) tmp[i]--; // in wavefront obj all indices start at 1, not zero
				f.push_back(tmp);
			}
			faces_open_.push_back(f);
		}
	}
	std::cerr << "# v# " << verts_open_.size() << " f# " << faces_open_.size() << " vt# " << uv_open_.size() << " vn# " << norms_open_.size() << std::endl;
	
	std::cerr << "reading diffuse" << std::endl;
	load_texture(filename, "_diffuse.tga", diffusemap_);
	/*
	std::cerr << "reading tangent" << std::endl;
	load_texture(filename, "_nm_tangent.tga", normalmap_);
	std::cerr << "reading spec" << std::endl;
	load_texture(filename, "_spec.tga", specularmap_);
	*/
}

Model::~Model() {}

int Model::nverts() {
	return (int)verts_.size();
}

int Model::nfaces() {
	return (int)faces_.size();
}

std::vector<int> Model::face(int idx) {
	std::vector<int> face;
	for (int i = 0; i<(int)faces_[idx].size(); i++) face.push_back(faces_[idx][i][0]);
	return face;
}

vec3 Model::vert(int i) {
	return verts_[i];
}

vec3 Model::vert(int iface, int nthvert) {
	return verts_[faces_[iface][nthvert][0]];
}

vec3 Model::vertopen(int iface, int nthvert) {
	return verts_open_[faces_open_[iface][nthvert][0]];
}

void Model::load_texture(std::string filename, const char *suffix, TGAImage &img) {
	std::string texfile(filename);
	size_t dot = texfile.find_last_of(".");
	if (dot != std::string::npos) {
		texfile = texfile.substr(0, dot) + std::string(suffix);
		std::cerr << "texture file " << texfile << " loading " << (img.read_tga_file(texfile.c_str()) ? "ok" : "failed") << std::endl;
		img.flip_vertically();
	}
}

TGAColor Model::diffuse(vec2 uvf) {
	GLfloat uv0 = uvf[0] * diffusemap_.get_width();
	GLfloat uv1 = uvf[1] * diffusemap_.get_height();
	return diffusemap_.get(uv0, uv1);
}

vec3 Model::normal(vec2 uvf) {
	GLfloat uv0 = uvf[0] * normalmap_.get_width();
	GLfloat uv1 = uvf[1] * normalmap_.get_height();
	TGAColor c = normalmap_.get(uv0, uv1);
	vec3 res;
	for (int i = 0; i<3; i++)
		res[2 - i] = (float)c[i] / 255.f*2.f - 1.f;
	return res;
}

vec2 Model::uv(int iface, int nthvert) {
	return uv_[faces_[iface][nthvert][1]];
}

float Model::specular(vec2 uvf) {
	GLfloat uv0 = uvf[0] * specularmap_.get_width();
	GLfloat uv1 = uvf[1] * specularmap_.get_height();
	return specularmap_.get(uv0, uv1)[0] / 1.f;
}

vec3 Model::normal(int iface, int nthvert) {
	int idx = faces_[iface][nthvert][2];
	float x = norms_[idx][0];
	float y = norms_[idx][1];
	float z = norms_[idx][2];
	float norm = std::sqrt(x*x + y*y + z*z);
	norms_[idx][0] = x * (1 / norm);
	norms_[idx][1] = y * (1 / norm);
	norms_[idx][2] = z * (1 / norm);
	return norms_[idx];
}

