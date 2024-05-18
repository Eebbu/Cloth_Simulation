#ifndef MASS_H
#define MASS_H

#include <glm/glm.hpp>
#include <map>
#include <vector>

class Spring;

class Mass {
public:
    double          m            = 1.0;
    bool            is_fixed     = false;
    glm::dvec2      tex_coord;
    glm::dvec3      normal;
    glm::dvec3	    position;
    glm::dvec3      last_position;
    glm::dvec3      velocity     = glm::dvec3(0, 0, 0);
    glm::dvec3	    acceleration = glm::dvec3(0, 0, 0);
    glm::dvec3      force        = glm::dvec3(0, 0, 0);
    
    enum SpringType{
        STRUCTURAL_ROW,       // -
        STRUCTURAL_COLUMN,    // |
        SHEAR_LEFT_TO_RIGHT,  // 
        SHEAR_RIGHT_TO_LEFT,  // /
        FLEXION_ROW,          // --
        FLEXION_COLUMN        // |
    };
    std::map<Mass::SpringType, std::vector<Spring*>> springs_map; // adaptive refinement

public:
    Mass(void) {}

	Mass(glm::dvec3 _pos, glm::dvec2 _tex_coord, bool _is_fixed)
        : position(_pos), last_position(_pos), tex_coord(_tex_coord), is_fixed(_is_fixed) {}

	~Mass() {}

    void link_springs(Mass::SpringType type, Spring* spring) {
        springs_map[type].push_back(spring);
    }
};

#endif // MASS_H