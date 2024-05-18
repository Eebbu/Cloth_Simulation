#pragma once

#include <vector>

#include "spring.h"
#include "rigid.h"

class Cloth {
public:
    const int         mass_per_row    = 30;
    const int         mass_per_col    = 30;
    const double      mass_density    = (double)mass_per_row/14.0;
    const double      structural_coef = 300.0;
    const double      shear_coef      = 200.0;
    const double      bending_coef    = 200.0;
    const double      damp_coef       = 0.65;
    const glm::dvec3  gravity         = glm::dvec3(0.0, -1.0, 0.0);
    const glm::vec3   cloth_pos       = glm::vec3(-7, 18, -6);
    const bool        draw_texture    = false;
    const double      refine_angle    = std::cos(glm::radians(135.0f));
    const int constraints_iterations  = 6;
    const int refine_iterations       = 3;
    
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
                    Mass* mass1 = get_mass(i, j);
                    Mass* mass2 = get_mass(i+1, j);
                    Spring* spring = new Spring(mass1, mass2, structural_coef, Spring::STRUCTURAL);
                    
                    springs.push_back(spring);
                    mass1->link_springs(Mass::STRUCTURAL_ROW, spring);
                    mass2->link_springs(Mass::STRUCTURAL_ROW, spring);
                }
                if (j < mass_per_col-1) {
                    Mass* mass1 = get_mass(i, j);
                    Mass* mass2 = get_mass(i, j+1);
                    Spring* spring = new Spring(mass1, mass2, structural_coef, Spring::STRUCTURAL);

                    springs.push_back(spring);
                    mass1->link_springs(Mass::STRUCTURAL_COLUMN, spring);
                    mass2->link_springs(Mass::STRUCTURAL_COLUMN, spring);
                }
                
                // shear springs
                if (i < mass_per_row-1 && j < mass_per_col-1) {
                    Mass* mass1 = get_mass(i, j);
                    Mass* mass2 = get_mass(i+1, j+1);
                    Mass* mass3 = get_mass(i+1, j);
                    Mass* mass4 = get_mass(i, j+1);
                    Spring* spring1 = new Spring(mass1, mass2, shear_coef, Spring::SHEAR);
                    Spring* spring2 = new Spring(mass3, mass4, shear_coef, Spring::SHEAR);

                    springs.push_back(spring1);
                    springs.push_back(spring2);
                    mass1->link_springs(Mass::SHEAR_LEFT_TO_RIGHT, spring1);
                    mass2->link_springs(Mass::SHEAR_LEFT_TO_RIGHT, spring1);
                    mass3->link_springs(Mass::SHEAR_RIGHT_TO_LEFT, spring2);
                    mass4->link_springs(Mass::SHEAR_RIGHT_TO_LEFT, spring2);
                }
                
                // flexion springs
                if (i < mass_per_row-2) {
                    Mass* mass1 = get_mass(i, j);
                    Mass* mass2 = get_mass(i+2, j);
                    Spring* spring = new Spring(mass1, mass2, bending_coef, Spring::FLEXION);

                    springs.push_back(spring);
                    mass1->link_springs(Mass::FLEXION_ROW, spring);
                    mass2->link_springs(Mass::FLEXION_ROW, spring);
                }
                if (j < mass_per_col-2) {
                    Mass* mass1 = get_mass(i, j);
                    Mass* mass2 = get_mass(i, j+2);
                    Spring* spring = new Spring(mass1, mass2, bending_coef, Spring::FLEXION);
                    
                    springs.push_back(spring);
                    mass1->link_springs(Mass::FLEXION_COLUMN, spring);
                    mass2->link_springs(Mass::FLEXION_COLUMN, spring);
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

    void compute_forces() {
        for (auto &mass : this->masses) {
            mass->force = glm::dvec3(0.0);
        }

        for (auto &spring: this->springs) {
            glm::dvec3 spring_vec = spring->mass1->position - spring->mass2->position;
            double spring_length = glm::length(spring_vec);

            glm::dvec3 elastic_force = spring_vec * spring->spring_constant  / spring_length * (spring_length - spring->rest_len);
            spring->mass1->force += -elastic_force;
            spring->mass2->force += elastic_force;
        }

        for (auto &mass : this->masses) {
            if (!mass->is_fixed) {
                mass->force += -mass->velocity * this->damp_coef;
                mass->force += gravity * mass->m;
            }
        }
    }

    void step(Ball* ball, double delta_t) {
        compute_forces();

        for (auto &mass : this->masses) {
            if (!mass->is_fixed) {
                mass->last_position = mass->position;
                mass->velocity += mass->force/mass->m*delta_t;
                mass->position += mass->velocity*delta_t;
            }
            mass->force = glm::dvec3(0.0, 0.0, 0.0); 
        }

        solve_constraints(0);
        update_velocity_after_constraints(delta_t);
       //  collisionResponse(ball);
    }

    

    /**
     * Runge Kutta
     */
    void rk4_step(Ball* ball, double delta_t) {
        std::vector<glm::dvec3> initial_positions;
        std::vector<glm::dvec3> initial_velocities;
        std::vector<glm::dvec3> k1_positions;
        std::vector<glm::dvec3> k1_velocities;
        std::vector<glm::dvec3> k2_positions;
        std::vector<glm::dvec3> k2_velocities;
        std::vector<glm::dvec3> k3_positions;
        std::vector<glm::dvec3> k3_velocities;
        std::vector<glm::dvec3> k4_positions;
        std::vector<glm::dvec3> k4_velocities;

        for (auto& mass : masses) {
            mass->last_position = mass->position;
            initial_positions.push_back(mass->position);
            initial_velocities.push_back(mass->velocity);
            k1_positions.push_back(glm::dvec3(0.0));
            k1_velocities.push_back(glm::dvec3(0.0));
            k2_positions.push_back(glm::dvec3(0.0));
            k2_velocities.push_back(glm::dvec3(0.0));
            k3_positions.push_back(glm::dvec3(0.0));
            k3_velocities.push_back(glm::dvec3(0.0));
            k4_positions.push_back(glm::dvec3(0.0));
            k4_velocities.push_back(glm::dvec3(0.0));
        }

        // k1
        compute_forces();
        for (size_t i = 0; i < masses.size(); ++i) {
            if (!masses[i]->is_fixed) {
                k1_positions[i] = masses[i]->velocity * delta_t;
                k1_velocities[i] = masses[i]->force / masses[i]->m * delta_t;
            }
        }

        // k2
        for (size_t i = 0; i < masses.size(); ++i) {
            if (!masses[i]->is_fixed) {
                masses[i]->position = initial_positions[i] + 0.5 * k1_positions[i];
                masses[i]->velocity = initial_velocities[i] + 0.5 * k1_velocities[i];
            }
        }
        compute_forces();
        for (size_t i = 0; i < masses.size(); ++i) {
            if (!masses[i]->is_fixed) {
                k2_positions[i] = masses[i]->velocity * delta_t;
                k2_velocities[i] = masses[i]->force / masses[i]->m * delta_t;
            }
        }

        // k3
        for (size_t i = 0; i < masses.size(); ++i) {
            if (!masses[i]->is_fixed) {
                masses[i]->position = initial_positions[i] + 0.5 * k2_positions[i];
                masses[i]->velocity = initial_velocities[i] + 0.5 * k2_velocities[i];
            }
        }
        compute_forces();
        for (size_t i = 0; i < masses.size(); ++i) {
            if (!masses[i]->is_fixed) {
                k3_positions[i] = masses[i]->velocity * delta_t;
                k3_velocities[i] = masses[i]->force / masses[i]->m * delta_t;
            }
        }

        // k4
        for (size_t i = 0; i < masses.size(); ++i) {
            if (!masses[i]->is_fixed) {
                masses[i]->position = initial_positions[i] + k3_positions[i];
                masses[i]->velocity = initial_velocities[i] + k3_velocities[i];
            }
        }
        compute_forces();
        for (size_t i = 0; i < masses.size(); ++i) {
            if (!masses[i]->is_fixed) {
                k4_positions[i] = masses[i]->velocity * delta_t;
                k4_velocities[i] = masses[i]->force / masses[i]->m * delta_t;
            }
        }

        // Combine
        for (size_t i = 0; i < masses.size(); ++i) {
            if (!masses[i]->is_fixed) {
                masses[i]->position = initial_positions[i] + (k1_positions[i] + 2.0 * k2_positions[i] + 2.0 * k3_positions[i] + k4_positions[i]) / 6.0;
                masses[i]->velocity = initial_velocities[i] + (k1_velocities[i] + 2.0 * k2_velocities[i] + 2.0 * k3_velocities[i] + k4_velocities[i]) / 6.0;
            }
            masses[i]->force = glm::dvec3(0.0);  // Reset force for the next timestep
        }

        solve_constraints(0);
        update_velocity_after_constraints(delta_t);
       //  collisionResponse(ball);
    }

    void solve_constraints(int iterations) {
        for (int i = 0; i< this->constraints_iterations; i++) {
            bool normal = true;
            for (auto &spring : springs) {
                //skip the flexion spring(almost not limited in real cloth)
                if(spring->spring_type == Spring::FLEXION){
                    continue;
                }
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
                    normal = false;
                }
                if (!spring->mass2->is_fixed) {
                    spring->mass2->position -= direction * correction;
                    normal = false;
                }
            }
            if (normal) {
                return;
            }
        }
    }

    void update_velocity_after_constraints(double delta_t) {
        for(auto &mass: masses) {
            mass->velocity = (mass->position - mass->last_position) / delta_t;
        }
    }

    bool detect_refinement(Mass* mass, Spring* spring1, Spring* spring2) {
        glm::dvec3 vec1 = spring1->mass1 == mass 
                ? spring1->mass1->position - spring1->mass2->position 
                : spring1->mass2->position - spring1->mass1->position;
        
        glm::dvec3 vec2 = spring2->mass1 == mass 
                ? spring2->mass1->position - spring2->mass2->position 
                : spring2->mass2->position - spring2->mass1->position;

        double vecCos = glm::dot(vec1, vec2) / (glm::length(vec1) * glm::length(vec2));
        
        return vecCos > refine_angle;
    }

    void do_adaptive_refinement(Mass* mass, Spring* spring1, Spring* spring2) {

    }


    void adaptive_refinement() {
        for (int i = 0; i < refine_iterations; i++) {
            bool refine = false;
            for(auto &mass : this->masses) {
                // only structral
                std::vector<Spring*> mass_springs = mass->springs_map[Mass::STRUCTURAL_ROW];
                if ( mass_springs.size() == 2 && detect_refinement(mass, mass_springs[0], mass_springs[1])) {
                    do_adaptive_refinement(mass, mass_springs[0], mass_springs[1]);
                    refine = true;
                }

                mass_springs = mass->springs_map[Mass::STRUCTURAL_COLUMN];
                if ( mass_springs.size() == 2 && detect_refinement(mass, mass_springs[0], mass_springs[1])) {
                    do_adaptive_refinement(mass, mass_springs[0], mass_springs[1]);
                    refine = true;
                }
            }
            if (!refine) {
                return;
            }
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

    void reset() {
        // for (auto& mass : masses) {
        //         delete mass;
        //     }
        //     masses.clear();

        //     for (auto& spring : springs) {
        //         delete spring;
        //     }
        //     springs.clear();

        // initialize_masses();
        // link_springs();
        // initialize_face();
        // fixed_mass(get_mass(0, 0), glm::dvec3(1.0, 0.0, 0.0));
        // fixed_mass(get_mass(mass_per_row-1, 0), glm::dvec3(-1.0, 0.0, 0.0));

        // 重置所有质点的位置、速度和力
        for (int i = 0; i < mass_per_row; i++) {
            for (int j = 0; j < mass_per_col; j++) {
                glm::dvec3 initial_position = glm::dvec3((double)i / mass_density, 0, (double)j / mass_density);
                Mass* mass = get_mass(i, j);
                mass->position = initial_position;
                mass->velocity = glm::dvec3(0.0, 0.0, 0.0);
                mass->force = glm::dvec3(0.0, 0.0, 0.0);
            }
        }

        // 重新链接弹簧，重置所有连接和弹性特性
        springs.clear(); // 清除旧的弹簧连接
        link_springs();  // 重新创建弹簧连接
        initialize_face();  // 重建面信息

        // 重置固定质点
        fixed_mass(get_mass(0, 0), glm::dvec3(1.0, 0.0, 0.0));
        fixed_mass(get_mass(mass_per_row-1, 0), glm::dvec3(-1.0, 0.0, 0.0));

        // 重新计算法线
        compute_normal();
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
