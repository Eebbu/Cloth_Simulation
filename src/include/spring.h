#ifndef SPRING_H
#define SPRING_H

#include "mass.h"
#include <glm/glm.hpp>
#include <vector>


class Spring {
public:
    Mass*  mass1;
    Mass*  mass2;
    double max_len;
	double rest_len;
    double spring_constant;
    
	Spring(Mass* m1, Mass* m2, double k): mass1(m1), mass2(m2), spring_constant(k) {
        rest_len = glm::length(mass2->position - mass1->position);
        max_len = rest_len * 1.5;
	}

    double get_length() {
        return glm::length(mass2->position - mass1->position);
    }
};

#endif // SPRING_H
