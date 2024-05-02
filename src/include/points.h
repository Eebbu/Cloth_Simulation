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
    double  m;           // In this project it will always be 1
    bool    is_fixed;        // Use to pin the cloth
    Vec2    texCoord;       // Texture coord
    Vec3    normal;         // For smoothly shading
	Vec3	position;
    Vec3    velocity;
    Vec3    force;
	Vec3	acceleration;

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
        velocity.setZeroVec();
        force.setZeroVec();
        acceleration.setZeroVec();
    }

	~Mass(void) {}

	void addForce(Vec3 f) {
        force += f;
	}

	void step(double timeStep) {
		if (!is_fixed) // Verlet integration
		{
            acceleration = force/m;
            velocity += acceleration*timeStep;
            position += velocity*timeStep;
        }
        force.setZeroVec();
	}
};
