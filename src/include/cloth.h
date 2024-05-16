#pragma once

#include <vector>

#include "spring.h"
#include "rigid.h"

class Cloth {
public:
    const int         mass_density    = 4;
    const int         width           = 10;
    const int         height          = 10;
    const int         mass_per_row    = 40;
    const int         mass_per_col    = 40;
    const double      structural_coef = 300.0;
    const double      shear_coef      = 200.0;
    const double      bending_coef    = 200.0;
    const double      damp_coef       = 0.1;
    const glm::dvec3  gravity         = glm::dvec3(0.0, -2.0, 0.0);
    const glm::vec3   cloth_pos       = glm::vec3(-5, 16, 0);
    const bool        draw_texture    = false;
    const int constraints_iterations  = 10;
    
    std::vector<Mass*>   masses;
	std::vector<Spring*> springs;
	std::vector<Mass*>   faces;
    
	Cloth() {
        initialize_masses();
        link_springs();
        initialize_face();

        fixed_mass(get_mass(0, 0), glm::dvec3(1.0, 0.0, 0.0));
        fixed_mass(get_mass(mass_per_row-1, 0), glm::dvec3(-1.0, 0.0, 0.0));
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

    void fixed_mass(Mass* mass, glm::dvec3 offset) {
        mass->position += offset;
        mass->is_fixed = true;
    }

    void initialize_masses() {
        for (int i = 0; i < mass_per_row; i ++) {
            for (int j = 0; j < mass_per_col; j ++) {
                glm::dvec2 text_coord((double)j/(mass_per_row-1), (double)i/(1-mass_per_col));
                glm::dvec3 position((double)j/mass_density, 0, (double)i/mass_density);
                Mass* mass = new Mass(position, text_coord, false);
                masses.push_back(mass);
            }
        }
    }

    void link_springs() {
        for (int i = 0; i < mass_per_row; i ++) {
            for (int j = 0; j < mass_per_col; j ++) {
                // structural springs
                if (i < mass_per_row-1) {
                    springs.push_back(new Spring(get_mass(i, j), get_mass(i+1, j), structural_coef));
                }
                if (j < mass_per_col-1) {
                    springs.push_back(new Spring(get_mass(i, j), get_mass(i, j+1), structural_coef));
                }
                
                // shear springs
                if (i < mass_per_row-1 && j < mass_per_col-1) {
                    springs.push_back(new Spring(get_mass(i, j), get_mass(i+1, j+1), shear_coef));
                    springs.push_back(new Spring(get_mass(i+1, j), get_mass(i, j+1), shear_coef));
                }
                
                // flexion springs
                if (i < mass_per_row-2) {
                    springs.push_back(new Spring(get_mass(i, j), get_mass(i+2, j), bending_coef));
                }
                if (j < mass_per_col-2) {
                    springs.push_back(new Spring(get_mass(i, j), get_mass(i, j+2), bending_coef));
                }
            }
        }
    }
    
	void initialize_face() {
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

    void step(Ball* ball, double delta_t) {
        for (auto &spring: this->springs) {
            glm::dvec3 spring_vec = spring->mass1->position - spring->mass2->position;
            double spring_length = glm::length(spring_vec);

            glm::dvec3 force = spring_vec * spring->spring_constant  / spring_length * (spring_length - spring->rest_len);
            spring->mass1->force += -force;
            spring->mass2->force += force;
        }

        for (auto &mass : this->masses) {
            if (!mass->is_fixed) {
                glm::dvec3 a = mass->force / mass->m + gravity;
                glm::dvec3 last_position = mass->position;
                mass->position += (mass->position - mass->last_position) * (1 - this->damp_coef) + a * delta_t * delta_t;
                mass->last_position = last_position;

                // mass->force += gravity * mass->m - mass->velocity * this->damp_coef;
                // mass->velocity += mass->force/mass->m*delta_t;
                // mass->position += mass->velocity*delta_t;
            }
            mass->force = glm::dvec3(0.0, 0.0, 0.0); 
        }

        solve_constraints(0);
        update_velocity_after_constraints(delta_t);
       //  collisionResponse(ball);
    }

    void solve_constraints(int iterations) {
        for (int i = 0; i< this->constraints_iterations; i++) {
            for (auto &spring : springs) {
                double current_length = spring->get_length();
                if (current_length <= spring->max_len) {
                    continue;
                }
                if (spring->mass1->is_fixed && spring->mass2->is_fixed) {
                    continue;
                }
                
                glm::dvec3 direction = (spring->mass2->position - spring->mass1->position) / current_length;
                double delta = current_length - spring->max_len;
                double mass_sum = (spring->mass1->is_fixed ? 0.0 : 1.0) + (spring->mass2->is_fixed ? 0.0 : 1.0);
                double correction = delta / mass_sum;

                if (!spring->mass1->is_fixed) {
                    spring->mass1->position += direction * correction;
                }
                if (!spring->mass2->is_fixed) {
                    spring->mass2->position -= direction * correction;
                }
            }
        }
    }

    void update_velocity_after_constraints(double delta_t) {
        for(auto &mass: masses) {
            mass->velocity = (mass->position - mass->last_position) / delta_t;
        }
    }

    void compute_normal() {
        for (int i = 0; i < faces.size()/3; i ++) { 
            Mass* m1 = faces[3*i+0];
            Mass* m2 = faces[3*i+1];
            Mass* m3 = faces[3*i+2];
            
            glm::dvec3 normal = glm::cross(m2->position - m1->position, m3->position - m1->position);
            normal = glm::normalize(normal);
            m1->normal = normal;
            m2->normal = normal;
            m3->normal = normal;
        }
	}
	
	void add_force(glm::dvec3 f) {		 
        for (int i = 0; i < masses.size(); i++) {
            masses[i]->force += f;
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
