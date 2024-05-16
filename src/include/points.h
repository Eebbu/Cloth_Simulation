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
    double  m;           
    bool    is_fixed;       
    glm::dvec2    tex_coord;       
    glm::dvec3    normal;         
	glm::dvec3	position;
    glm::dvec3    velocity;
    glm::dvec3    force;
	glm::dvec3	acceleration;
    glm::dvec3 last_position;

public:
    Mass(void) {
        m = 1.0;
        is_fixed = false;
        velocity = glm::dvec3(0, 0, 0);
        force = glm::dvec3(0, 0, 0);
        acceleration = glm::dvec3(0, 0, 0);
    }
	Mass(glm::dvec3 pos, bool _is_fixed) {
        m = 1.0;
        is_fixed = _is_fixed;
        position = pos;
        last_position = pos;
        velocity = glm::dvec3(0, 0, 0);
        force = glm::dvec3(0, 0, 0);
        acceleration = glm::dvec3(0, 0, 0);
    }

	~Mass(void) {}

};
