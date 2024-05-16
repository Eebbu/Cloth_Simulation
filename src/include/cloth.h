#pragma once

#include <vector>

#include "spring.h"
#include "rigid.h"

class Cloth {
public:
    const int     mass_density = 4;
    const int     iteration_freq = 25;
    const int     width = 10;
    const int     height = 10;
    const int     mass_per_row = 40;
    const int     mass_per_col = 40;
    const double  structural_coef = 300.0;
    const double  shear_coef = 300.0;
    const double  bending_coef = 300.0;
    double damp_coef = 0.1;
    Vec3 gravity = Vec3(0.0, -2.0, 0.0);
    Vec3 center = Vec3(0, 10, 0);
    
    enum DrawModeEnum {
        DRAW_NODES,
        DRAW_LINES,
        DRAW_FACES
    };

    DrawModeEnum drawMode = DRAW_LINES;
    
    Vec3 cloth_pos = Vec3(-5, 16, 0);
    std::vector<Mass*>   masses;
	std::vector<Spring*> springs;
	std::vector<Mass*>   faces;
    
	Cloth() {
        init();
	}

	~Cloth() { 
		for (int i = 0; i < masses.size(); i++) { 
            delete masses[i]; 
        }
		for (int i = 0; i < springs.size(); i++) { 
            delete springs[i]; 
        }

		masses.clear();
		springs.clear();
		faces.clear();
	}
 
public:
    Mass* get_mass(int x, int y) { 
        return masses[y * mass_per_row + x]; 
    }

    void fixed_mass(Mass* mass, Vec3 offset) {
        mass->position += offset;
        mass->is_fixed = true;
    }
    
	void init() {
        /** Add masses **/
        for (int i = 0; i < mass_per_row; i ++) {
            for (int j = 0; j < mass_per_col; j ++) {
                /** Create mass by position **/
                Mass* mass = new Mass(Vec3((double)j/mass_density, 0, ((double)i/mass_density)), false);
                /** Set texture coordinates **/
                mass->tex_coord.x = (double)j/(mass_per_row-1);
                mass->tex_coord.y = (double)i/(1-mass_per_col);
                /** Add mass to cloth **/
                masses.push_back(mass);
            }
        }
        
        /** Add springs **/
        for (int i = 0; i < mass_per_row; i ++) {
            for (int j = 0; j < mass_per_col; j ++) {
                /** Structural **/
                if (i < mass_per_row-1) {
                    springs.push_back(new Spring(get_mass(i, j), get_mass(i+1, j), structural_coef));
                }
                if (j < mass_per_col-1) {
                    springs.push_back(new Spring(get_mass(i, j), get_mass(i, j+1), structural_coef));
                }
                /** Shear **/
                if (i < mass_per_row-1 && j < mass_per_col-1) {
                    springs.push_back(new Spring(get_mass(i, j), get_mass(i+1, j+1), shear_coef));
                    springs.push_back(new Spring(get_mass(i+1, j), get_mass(i, j+1), shear_coef));
                }
                /** Bending **/
                if (i < mass_per_row-2) springs.push_back(new Spring(get_mass(i, j), get_mass(i+2, j), bending_coef));
                if (j < mass_per_col-2) springs.push_back(new Spring(get_mass(i, j), get_mass(i, j+2), bending_coef));
            }
        }

        fixed_mass(get_mass(0, 0), Vec3(1.0, 0.0, 0.0));
        fixed_mass(get_mass(mass_per_row-1, 0), Vec3(-1.0, 0.0, 0.0));

		/** Triangle faces **/
        for (int i = 0; i < mass_per_row-1; i ++) {
            for (int j = 0; j < mass_per_col-1; j ++) {
                // Left upper triangle
                faces.push_back(get_mass(i+1, j));
                faces.push_back(get_mass(i, j));
                faces.push_back(get_mass(i, j+1));

                // Right bottom triangle
                faces.push_back(get_mass(i+1, j+1));
                faces.push_back(get_mass(i+1, j));
                faces.push_back(get_mass(i, j+1));
            }
        }
	}
	
	void computeNormal() {
        for (int i = 0; i < faces.size()/3; i ++) { 
            Mass* m1 = faces[3*i+0];
            Mass* m2 = faces[3*i+1];
            Mass* m3 = faces[3*i+2];
            
            Vec3 normal = Vec3::cross(m2->position - m1->position, m3->position - m1->position);
            normal.normalize();
            m1->normal = normal;
            m2->normal = normal;
            m3->normal = normal;
        }
	}

    void step(Ball* ball, double delta_t) {
        for (auto &spring: this->springs) {
            Vec3 spring_vec = spring->mass1->position - spring->mass2->position;
            double spring_length = Vec3::dist(spring->mass1->position, spring->mass2->position);

            Vec3 force = spring_vec * spring->spring_constant  / spring_length * (spring_length - spring->rest_len);
            spring->mass1->force += force.minus();
            spring->mass2->force += force;
        }

        for (auto &mass : this->masses) {
            if (!mass->is_fixed) {
                Vec3 a = mass->force / mass->m + gravity;
                Vec3 last_position = mass->position;
                mass->position += (mass->position - mass->last_position) * (1 - this->damp_coef) + a * delta_t * delta_t;
                mass->last_position = last_position;

                // mass->force += gravity * mass->m - mass->velocity * this->damp_coef;
                // mass->velocity += mass->force/mass->m*delta_t;
                // mass->position += mass->velocity*delta_t;
            }
            mass->force = Vec3(0.0, 0.0, 0.0); 
        }
       //  collisionResponse(ball);
    }
	
	void addForce(Vec3 f) {		 
		for (int i = 0; i < masses.size(); i++) {
			masses[i]->addForce(f);
		}
	}

	void computeForce(double timeStep, Vec3 gravity) {
        /** Masses **/
		for (int i = 0; i < masses.size(); i++) {
			masses[i]->addForce(gravity * masses[i]->m);
		}
		/** Springs **/
		for (int i = 0; i < springs.size(); i++) {
			springs[i]->applyInternalForce(timeStep);
		}
	}

	void integrate(double airFriction, double timeStep) {
        /** Mass* */
        for (int i = 0; i < masses.size(); i++) {
            masses[i]->step(timeStep);
        }
	}
	
    // Vec3 getWorldPos(Mass* n) { 
    //     return cloth_pos + n->position; 
    // }

    // void setWorldPos(Mass* n, Vec3 pos) { 
    //     n->position = pos - cloth_pos; 
    // }
    
	// void collisionResponse(Ball* ball) {
    //     for (int i = 0; i < masses.size(); i++) {   
    //         /** Ball collision **/
    //         Vec3 distVec = getWorldPos(masses[i]) - ball->center;
    //         double distLen = distVec.length();
    //         double safeDist = ball->radius*1.05;
    //         if (distLen < safeDist) {
    //             distVec.normalize();
    //             setWorldPos(masses[i], distVec*safeDist+ball->center);
    //             masses[i]->velocity = masses[i]->velocity*ball->friction;
    //         }
    //     }
	// }
};
