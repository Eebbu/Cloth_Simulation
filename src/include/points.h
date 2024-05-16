#pragma once

#include "Vectors.h"

struct Vertex {
public:
    Vec3 position;
    Vec3 normal;
    
    Vertex() {}
    Vertex(Vec3 pos) {
        position = pos;
    }
    ~Vertex() {}
};

class Mass {
public:
    double  m;           
    bool    is_fixed;       
    Vec2    tex_coord;       
    Vec3    normal;         
	Vec3	position;
    Vec3    velocity;
    Vec3    force;
	Vec3	acceleration;
    Vec3 last_position;

public:
    Mass(void) {
        m = 1.0;
        is_fixed = false;
        velocity.setZeroVec();
        force.setZeroVec();
        acceleration.setZeroVec();
    }
	Mass(Vec3 pos, bool _is_fixed) {
        m = 1.0;
        is_fixed = _is_fixed;
        position = pos;
        last_position = pos;
        velocity.setZeroVec();
        force.setZeroVec();
        acceleration.setZeroVec();
    }

	~Mass(void) {}

	void addForce(Vec3 f) {
        force += f;
	}

	void step(double time_step) {
		if (!is_fixed) {
            acceleration = force / m;
            velocity += acceleration * time_step;
            position += velocity * time_step;
        }
        force.setZeroVec();
	}
};
