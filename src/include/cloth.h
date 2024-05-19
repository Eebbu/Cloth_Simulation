#pragma once

#include <vector>

#include "spring.h"
#include "rigid.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp> // 包含 string_cast 头文件
class Cloth
{
public:
    const int mass_per_row = 36;
    const int mass_per_col = 36;
    const double mass_density = (double)mass_per_row / 14.0;
    const double structural_coef = 300.0;
    const double shear_coef = 50.0;
    const double flexion_coef = 100.0;
    const double damp_coef = 0.65;
    const glm::dvec3 gravity = glm::dvec3(0.0, -1.0, 0.0);
    const glm::vec3 cloth_pos = glm::vec3(-7.0, 18.0, -6.0);
    bool draw_texture = true;
    const double refine_angle = std::cos(glm::radians(135.0f));
    const int constraints_iterations = 6;
    const int refine_iterations = 3;
    const double visco_coef = 0.5f;                       // Viscosity coefficient
    const glm::dvec3 u_fluid = glm::dvec3(0.0, 0.0, 0.0); // Assume fluid = 0 with no wind

    std::vector<Mass *> masses;
    std::vector<Spring *> springs;
    std::vector<Mass *> faces;

    Cloth()
    {
        initialize_masses();
        link_springs();
        initialize_face();

        fixed_mass(get_mass(0, 0), glm::dvec3(0.8, 0.0, 0.0));
        fixed_mass(get_mass(mass_per_row - 1, 0), glm::dvec3(-0.8, 0.0, 0.0));
    }

    ~Cloth()
    {
        for (int i = 0; i < masses.size(); i++)
        {
            delete masses[i];
        }

        for (int i = 0; i < springs.size(); i++)
        {
            delete springs[i];
        }
        masses.clear();
        springs.clear();
        faces.clear();
    }

public:
    Mass *get_mass(int x, int y)
    {
        return masses[y * mass_per_row + x];
    }

    void fixed_mass(Mass *mass, glm::dvec3 offset)
    {
        mass->position += offset;
        mass->is_fixed = true;
    }

    void initialize_masses()
    {
        for (int i = 0; i < mass_per_row; i++)
        {
            for (int j = 0; j < mass_per_col; j++)
            {
                glm::dvec2 text_coord((double)j / (mass_per_row - 1), (double)i / (1 - mass_per_col));
                glm::dvec3 position((double)j / mass_density, 0, (double)i / mass_density);
                Mass *mass = new Mass(position, text_coord, false);
                masses.push_back(mass);
            }
        }
    }

    void link_springs()
    {
        for (int i = 0; i < mass_per_row; i++)
        {
            for (int j = 0; j < mass_per_col; j++)
            {
                Mass* mass = get_mass(i, j);
                // structural springs
                if (i < mass_per_row - 1)
                {
                    springs.push_back(new Spring(mass, get_mass(i + 1, j), structural_coef, Spring::STRUCTURAL));
                }
                if (j < mass_per_col - 1)
                {
                    springs.push_back(new Spring(mass, get_mass(i, j + 1), structural_coef, Spring::STRUCTURAL));
                }

                // shear springs
                if (i < mass_per_row - 1 && j < mass_per_col - 1)
                {
                    springs.push_back(new Spring(mass, get_mass(i + 1, j + 1), structural_coef, Spring::SHEAR));
                    springs.push_back(new Spring(get_mass(i + 1, j), get_mass(i, j + 1), structural_coef, Spring::SHEAR));
                }

                // flexion springs
                if (i < mass_per_row - 2)
                {
                    springs.push_back(new Spring(mass, get_mass(i + 2, j), flexion_coef, Spring::FLEXION));
                }
                if (j < mass_per_col - 2)
                {
                    springs.push_back(new Spring(mass, get_mass(i, j + 2), flexion_coef, Spring::FLEXION));
                }
            }
        }
    }

    void initialize_face()
    {
        for (int i = 0; i < mass_per_row - 1; i++)
        {
            for (int j = 0; j < mass_per_col - 1; j++)
            {
                // Left upper triangle
                faces.push_back(get_mass(i + 1, j));
                faces.push_back(get_mass(i, j));
                faces.push_back(get_mass(i, j + 1));

                // Right bottom triangle
                faces.push_back(get_mass(i + 1, j + 1));
                faces.push_back(get_mass(i + 1, j));
                faces.push_back(get_mass(i, j + 1));
            }
        }
    }

    void compute_forces()
    {
        for (auto &mass : this->masses)
        {
            mass->force = glm::dvec3(0.0);
        }

        for (auto &spring : this->springs)
        {
            glm::dvec3 spring_vec = spring->mass1->position - spring->mass2->position;
            double spring_length = glm::length(spring_vec);

            glm::dvec3 elastic_force = spring_vec * spring->spring_constant / spring_length * (spring_length - spring->rest_len);
            spring->mass1->force += -elastic_force;
            spring->mass2->force += elastic_force;
        }

        for (auto &mass : this->masses)
        {
            if (!mass->is_fixed)
            {
                // damping force
                mass->force += -mass->velocity * this->damp_coef;
                // gravity
                mass->force += gravity * mass->m;
                // velo
                glm::dvec3 relative_velocity = u_fluid - mass->velocity;
                double velocity_normal_component = glm::dot(mass->normal, relative_velocity);
                glm::dvec3 fluid_force = visco_coef * velocity_normal_component * mass->normal;
                mass->force += fluid_force;
            }
        }
    }

    void step(bool constraint, RigidType type, void *object, double delta_t)
    {
        compute_forces();

        for (auto &mass : this->masses)
        {
            if (!mass->is_fixed)
            {
                mass->last_position = mass->position;
                mass->velocity += mass->force / mass->m * delta_t;
                mass->position += mass->velocity * delta_t;
            }
            mass->force = glm::dvec3(0.0, 0.0, 0.0);
        }
        if(constraint){
            solve_constraints(0);
            update_velocity_after_constraints(delta_t);
        }
        collisionResponse(type, object);
    }

    /**
     * Runge Kutta
     */
    void rk4_step(bool constraint, RigidType type, void *object, double delta_t)
    {
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

        for (auto &mass : masses)
        {
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
        for (size_t i = 0; i < masses.size(); ++i)
        {
            if (!masses[i]->is_fixed)
            {
                k1_positions[i] = masses[i]->velocity * delta_t;
                k1_velocities[i] = masses[i]->force / masses[i]->m * delta_t;
            }
        }

        // k2
        for (size_t i = 0; i < masses.size(); ++i)
        {
            if (!masses[i]->is_fixed)
            {
                masses[i]->position = initial_positions[i] + 0.5 * k1_positions[i];
                masses[i]->velocity = initial_velocities[i] + 0.5 * k1_velocities[i];
            }
        }
        compute_forces();
        for (size_t i = 0; i < masses.size(); ++i)
        {
            if (!masses[i]->is_fixed)
            {
                k2_positions[i] = masses[i]->velocity * delta_t;
                k2_velocities[i] = masses[i]->force / masses[i]->m * delta_t;
            }
        }

        // k3
        for (size_t i = 0; i < masses.size(); ++i)
        {
            if (!masses[i]->is_fixed)
            {
                masses[i]->position = initial_positions[i] + 0.5 * k2_positions[i];
                masses[i]->velocity = initial_velocities[i] + 0.5 * k2_velocities[i];
            }
        }
        compute_forces();
        for (size_t i = 0; i < masses.size(); ++i)
        {
            if (!masses[i]->is_fixed)
            {
                k3_positions[i] = masses[i]->velocity * delta_t;
                k3_velocities[i] = masses[i]->force / masses[i]->m * delta_t;
            }
        }

        // k4
        for (size_t i = 0; i < masses.size(); ++i)
        {
            if (!masses[i]->is_fixed)
            {
                masses[i]->position = initial_positions[i] + k3_positions[i];
                masses[i]->velocity = initial_velocities[i] + k3_velocities[i];
            }
        }
        compute_forces();
        for (size_t i = 0; i < masses.size(); ++i)
        {
            if (!masses[i]->is_fixed)
            {
                k4_positions[i] = masses[i]->velocity * delta_t;
                k4_velocities[i] = masses[i]->force / masses[i]->m * delta_t;
            }
        }

        // Combine
        for (size_t i = 0; i < masses.size(); ++i)
        {
            if (!masses[i]->is_fixed)
            {
                masses[i]->position = initial_positions[i] + (k1_positions[i] + 2.0 * k2_positions[i] + 2.0 * k3_positions[i] + k4_positions[i]) / 6.0;
                masses[i]->velocity = initial_velocities[i] + (k1_velocities[i] + 2.0 * k2_velocities[i] + 2.0 * k3_velocities[i] + k4_velocities[i]) / 6.0;
            }
            masses[i]->force = glm::dvec3(0.0); // Reset force for the next timestep
        }

        if(constraint){
            solve_constraints(0);
            update_velocity_after_constraints(delta_t);
        }
        collisionResponse(type, object);
    }

    void explicit_verlet(bool constraint, RigidType type, void *object, double delta_t) 
    {
        compute_forces();

        for (auto &mass : this->masses)
        {
            if (!mass->is_fixed)
            {
                glm::dvec3 a = mass->force / mass->m + gravity * 10.0;
                auto temp = mass->position;
                mass->position += (1.0 - damp_coef) * (mass->position - mass->last_position) + a * delta_t * delta_t;
                mass->last_position = temp;
            }
            mass->force = glm::dvec3(0.0, 0.0, 0.0);
        }
        if(constraint){
            solve_constraints(0);
            update_velocity_after_constraints(delta_t);
        }
        collisionResponse(type, object);
    }


    void solve_constraints(int iterations)
    {
        for (int i = 0; i < this->constraints_iterations; i++)
        {
            bool normal = true;
            for (auto &spring : springs)
            {
                // skip the flexion spring(almost not limited in real cloth)
                if (spring->spring_type == Spring::FLEXION)
                {
                    continue;
                }
                double current_length = spring->get_length();
                if (current_length <= spring->max_len)
                {
                    continue;
                }
                if (spring->mass1->is_fixed && spring->mass2->is_fixed)
                {
                    continue;
                }

                glm::dvec3 direction = (spring->mass2->position - spring->mass1->position) / current_length;
                double delta = current_length - spring->max_len;
                double mass_sum = (spring->mass1->is_fixed ? 0.0 : 1.0) + (spring->mass2->is_fixed ? 0.0 : 1.0);
                double correction = delta / mass_sum;

                if (!spring->mass1->is_fixed)
                {
                    spring->mass1->position += direction * correction;
                    normal = false;
                }
                if (!spring->mass2->is_fixed)
                {
                    spring->mass2->position -= direction * correction;
                    normal = false;
                }
            }
            if (normal)
            {
                return;
            }
        }
    }

    void update_velocity_after_constraints(double delta_t)
    {
        for (auto &mass : masses)
        {
            mass->velocity = (mass->position - mass->last_position) / delta_t;
        }
    }

    void compute_normal()
    {
        for (int i = 0; i < faces.size() / 3; i++)
        {
            Mass *m1 = faces[3 * i + 0];
            Mass *m2 = faces[3 * i + 1];
            Mass *m3 = faces[3 * i + 2];

            glm::dvec3 normal = glm::cross(m2->position - m1->position, m3->position - m1->position);
            normal = glm::normalize(normal);
            m1->normal = normal;
            m2->normal = normal;
            m3->normal = normal;
        }
    }

    void add_force(glm::dvec3 f)
    {
        for (int i = 0; i < masses.size(); i++)
        {
            masses[i]->force += f;
        }
    }

    void reset()
    {
        // reset masses
        for (int i = 0; i < mass_per_row; i++)
        {
            for (int j = 0; j < mass_per_col; j++)
            {
                glm::dvec3 initial_position = glm::dvec3((double)i / mass_density, 0, (double)j / mass_density);
                Mass *mass = get_mass(i, j);
                mass->position = initial_position;
                mass->velocity = glm::dvec3(0.0, 0.0, 0.0);
                mass->force = glm::dvec3(0.0, 0.0, 0.0);
            }
        }

        // reconnect spring
        springs.clear();
        link_springs();
        // reset face
        initialize_face();

        // pin mass
        fixed_mass(get_mass(0, 0), glm::dvec3(1.0, 0.0, 0.0));
        fixed_mass(get_mass(mass_per_row - 1, 0), glm::dvec3(-1.0, 0.0, 0.0));

        // recompute normal
        compute_normal();
    }

    glm::vec3 getWorldPos(Mass *m)
    {
        return cloth_pos + glm::vec3(m->position);
    }

    void setWorldPos(Mass *m, glm::vec3 pos)
    {
        m->position = pos - cloth_pos;
    }

    void collisionResponse(RigidType type, void *object)
    {
        switch (type)
        {
            case RigidType::Empty:
                break;
            case RigidType::Ball:
                collisionResponse(static_cast<Ball *>(object));
                break;
            case RigidType::Cube:
                collisionResponse(static_cast<Cube *>(object));
                break;
        }
    }
    void collisionResponse(Ball *ball)
    {
        for (auto &mass : masses)
        {
            glm::vec3 distVec = getWorldPos(mass) - ball->center;
            float dist = glm::length(distVec);
            float penetration = ball->radius - dist;

            if (dist < ball->radius)
            {
                glm::vec3 normal = glm::normalize(distVec);
                int r = ball->radius;
                glm::vec3 contactPoint = ball->center + normal * (float)r;
                // glm::vec3 contactPoint = ball->center + normal *(float)1.2*(float)r;
                // Reposition the mass
                setWorldPos(mass, contactPoint);

                // Reflect velocity
                glm::vec3 incomingVelocity = mass->velocity;
                float velocityAlongNormal = glm::dot(incomingVelocity, normal);

                if (velocityAlongNormal < 0)
                {
                    glm::vec3 reflectedVelocity = incomingVelocity - 2 * velocityAlongNormal * normal;
                    mass->velocity = reflectedVelocity * (float)ball->friction;
                }
            }
        }
    }
    void collisionResponse(Cube *cube)//AABB
    {
        // Iterate through each mass in the cloth
        for (auto &mass : masses)
        {
            glm::vec3 worldPos = getWorldPos(mass);
            glm::vec3 dist = worldPos - cube->center;

            //half size of the cube
            double halfSize = cube->size / 2.0;

            //Check if the mass is inside the cube
            bool insideX = abs(dist.x) < halfSize;
            bool insideY = abs(dist.y) < halfSize;
            bool insideZ = abs(dist.z) < halfSize;

            if (insideX && insideY && insideZ)
            {
                //calculate penetration
                double penetrationX = halfSize - abs(dist.x);
                double penetrationY = halfSize - abs(dist.y);
                double penetrationZ = halfSize - abs(dist.z);

                //find nearest face
                if (penetrationX < penetrationY && penetrationX < penetrationZ)
                {
                    dist.x = (dist.x > 0) ? halfSize : -halfSize;
                }
                else if (penetrationY < penetrationX && penetrationY < penetrationZ)
                {
                    dist.y = (dist.y > 0) ? halfSize : -halfSize;
                }
                else
                {
                    dist.z = (dist.z > 0) ? halfSize : -halfSize;
                }
                //reposition the mass
                glm::vec3 newWorldPos = cube->center + dist;
                setWorldPos(mass, newWorldPos);

                //reflect the velocity
                glm::vec3 normal = glm::normalize(newWorldPos - worldPos);
                glm::vec3 incomingVelocity = mass->velocity;
                float velocityAlongNormal = glm::dot(incomingVelocity, normal);

                if (velocityAlongNormal < 0)
                {
                    glm::vec3 reflectedVelocity = incomingVelocity - 2 * velocityAlongNormal * normal;
                    mass->velocity = reflectedVelocity * cube->friction;
                }
            }
        }
    }

    // std::cout << "Cube Collision Detected: \n";
    // std::cout << "  WorldPos: " << glm::to_string(worldPos) << "\n";
    // std::cout << "  NearestSurface: " << glm::to_string(nearestSurface) << "\n";
    // std::cout << "  DistToSurface: " << glm::to_string(distToSurface) << "\n";
    // std::cout << "  CollisionNormal: " << glm::to_string(collisionNormal) << "\n";
    // std::cout << "  NewWorldPos: " << glm::to_string(newWorldPos) << "\n";
};
