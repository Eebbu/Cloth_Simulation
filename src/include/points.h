#pragma once

struct Vertex {
public:
    glm::dvec3 position;
    glm::dvec3 normal;
    
    Vertex() {}
    Vertex(glm::dvec3 pos) {
        position = pos;
    }
    ~Vertex() {}
};

class Mass {
public:
    double      m = 1.0;           
    bool        is_fixed = false;       
    glm::dvec2  tex_coord;       
    glm::dvec3  normal;         
	glm::dvec3	position;
    glm::dvec3  last_position;
    glm::dvec3  velocity = glm::dvec3(0, 0, 0);
    glm::dvec3  force = glm::dvec3(0, 0, 0);
	glm::dvec3	acceleration = glm::dvec3(0, 0, 0);

public:
    Mass(void) {}

	Mass(glm::dvec3 _pos, glm::dvec2 _tex_coord, bool _is_fixed)
        : position(_pos), last_position(_pos), tex_coord(_tex_coord), is_fixed(_is_fixed) {}

	~Mass() {}
};
