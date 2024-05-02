#pragma once

#include "points.h"

using namespace std;

class Spring
{
public:
    Mass* mass1;
    Mass* mass2;
	double restLen;
    double hookCoef;
    double dampCoef;
    
	Spring(Mass* m1, Mass* m2, double k)
	{
        mass1 = m1;
        mass2 = m2;
		
        Vec3 currSp = mass2->position - mass1->position;
        restLen = currSp.length();
        hookCoef = k;
        dampCoef = 5.0;
	}

	void applyInternalForce(double timeStep) // Compute spring internal force
	{
        double currLen = Vec3::dist(mass1->position, mass2->position);
        Vec3 fDir1 = (mass2->position - mass1->position)/currLen;
        Vec3 diffV1 = mass2->velocity - mass1->velocity;
        Vec3 f1 = fDir1 * ((currLen-restLen)*hookCoef + Vec3::dot(diffV1, fDir1)*dampCoef);
        mass1->addForce(f1);
        mass2->addForce(f1.minus());
	}
};
