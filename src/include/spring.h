#pragma once

#include "points.h"

using namespace std;

class Spring {
public:
    Mass* mass1;
    Mass* mass2;
	double rest_len;
    double spring_constant;
    double dampCoef;
    
	Spring(Mass* m1, Mass* m2, double k) {
        mass1 = m1;
        mass2 = m2;
		
        rest_len = glm::length(mass2->position - mass1->position);
        spring_constant = k;
        dampCoef = 5.0;
	}
};
