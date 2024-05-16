#pragma once

#include "mass.h"

using namespace std;

class Spring {
public:
    Mass*  mass1;
    Mass*  mass2;
	double rest_len;
    double spring_constant;
    
	Spring(Mass* m1, Mass* m2, double k): mass1(m1), mass2(m2), spring_constant(k) {
        rest_len = glm::length(mass2->position - mass1->position);
	}
};
