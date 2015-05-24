#ifndef __MODEL_H__
#define __MODEL_H__

#include <vector>
#include "../packages/glm.0.9.6.3/build/native/include/glm/glm.hpp"
#include "../packages/glm.0.9.6.3/build/native/include/glm/gtc/matrix_transform.hpp"
#include "tgaimage.h"

using namespace std;
using namespace glm;

class Model {
private:
	std::vector<vec3> verts_;
	std::vector<vec3> verts_open_;
	std::vector<std::vector<vec3> > faces_; // attention, this Vec3i means vertex/uv/normal
	std::vector<std::vector<vec3> > faces_open_; // attention, this Vec3i means vertex/uv/normal
	std::vector<vec3> norms_;
	std::vector<vec3> norms_open_;
	std::vector<vec2> uv_;
	std::vector<vec2> uv_open_;
	TGAImage diffusemap_;
	TGAImage normalmap_;
	TGAImage specularmap_;
	void load_texture(std::string filename, const char *suffix, TGAImage &img);

public:
	Model(const char *filename, const char *fileopen);
	~Model();
	int nverts();
	int nfaces();
	vec3 normal(int iface, int nthvert);
	vec3 normal(vec2 uv);
	vec3 vert(int i);
	vec3 vert(int iface, int nthvert);
	vec3 vertopen(int iface, int nthvert);
	vec2 uv(int iface, int nthvert);
	TGAColor diffuse(vec2 uv);
	float specular(vec2 uv);
	std::vector<int> face(int idx);
	TGAImage Model::diffusemap()
	{
		return diffusemap_;
	}

};
#endif //__MODEL_H__

