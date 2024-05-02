#pragma once

#include <vector>

#include "spring.h"
#include "rigid.h"

class Cloth {
public:
    const int massesDensity = 4;
    const int iterationFreq = 25;
    const double structuralCoef = 1000.0;
    const double shearCoef = 50.0;
    const double bendingCoef = 400.0;
    
    enum DrawModeEnum {
        DRAW_NODES,
        DRAW_LINES,
        DRAW_FACES
    };
    DrawModeEnum drawMode = DRAW_FACES;
    
    Vec3 clothPos;
    
    int width, height;
    int massesPerRow, massesPerCol;
    
    std::vector<Mass*> masses;
	std::vector<Spring*> springs;
	std::vector<Mass*> faces;
    
    Vec2 pin1;
    Vec2 pin2;
    
	Cloth(Vec3 pos, Vec2 size) {
        clothPos = pos;
        width = size.x;
        height = size.y;
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
    Mass* getMass(int x, int y) { 
        return masses[y * massesPerRow + x]; 
    }

    Vec3 computeFaceNormal(Mass* n1, Mass* n2, Mass* n3) {
        return Vec3::cross(n2->position - n1->position, n3->position - n1->position);
    }
    
    void pin(Vec2 index, Vec3 offset) {
        if (!(index.x < 0 || index.x >= massesPerRow || index.y < 0 || index.y >= massesPerCol)) {
            getMass(index.x, index.y)->position += offset;
            getMass(index.x, index.y)->isFixed = true;
        }
    }

    void unPin(Vec2 index) {
        if (!(index.x < 0 || index.x >= massesPerRow || index.y < 0 || index.y >= massesPerCol)) {
            getMass(index.x, index.y)->isFixed = false;
        }
    }
    
	void init() {
        massesPerRow = width * massesDensity;
        massesPerCol = height * massesDensity;
        
        pin1 = Vec2(0, 0);
        pin2 = Vec2(massesPerRow-1, 0);
        
        /** Add masses **/
        printf("Init cloth with %d masses\n", massesPerRow*massesPerCol);
        for (int i = 0; i < massesPerRow; i ++) {
            for (int j = 0; j < massesPerCol; j ++) {
                /** Create mass by position **/
                Mass* mass = new Mass(Vec3((double)j/massesDensity, -((double)i/massesDensity), 0));
                /** Set texture coordinates **/
                mass->texCoord.x = (double)j/(massesPerRow-1);
                mass->texCoord.y = (double)i/(1-massesPerCol);
                /** Add mass to cloth **/
                masses.push_back(mass);
                
            }
        }
        
        /** Add springs **/
        for (int i = 0; i < massesPerRow; i ++) {
            for (int j = 0; j < massesPerCol; j ++) {
                /** Structural **/
                if (i < massesPerRow-1) springs.push_back(new Spring(getMass(i, j), getMass(i+1, j), structuralCoef));
                if (j < massesPerCol-1) springs.push_back(new Spring(getMass(i, j), getMass(i, j+1), structuralCoef));
                /** Shear **/
                if (i < massesPerRow-1 && j < massesPerCol-1) {
                    springs.push_back(new Spring(getMass(i, j), getMass(i+1, j+1), shearCoef));
                    springs.push_back(new Spring(getMass(i+1, j), getMass(i, j+1), shearCoef));
                }
                /** Bending **/
                if (i < massesPerRow-2) springs.push_back(new Spring(getMass(i, j), getMass(i+2, j), bendingCoef));
                if (j < massesPerCol-2) springs.push_back(new Spring(getMass(i, j), getMass(i, j+2), bendingCoef));
            }
        }
        
        pin(pin1, Vec3(1.0, 0.0, 0.0));
        pin(pin2, Vec3(-1.0, 0.0, 0.0));
        
		/** Triangle faces **/
        for (int i = 0; i < massesPerRow-1; i ++) {
            for (int j = 0; j < massesPerCol-1; j ++) {
                // Left upper triangle
                faces.push_back(getMass(i+1, j));
                faces.push_back(getMass(i, j));
                faces.push_back(getMass(i, j+1));
                // Right bottom triangle
                faces.push_back(getMass(i+1, j+1));
                faces.push_back(getMass(i+1, j));
                faces.push_back(getMass(i, j+1));
            }
        }
	}
	
	void computeNormal() {
        /** Reset masses' normal **/
        Vec3 normal(0.0, 0.0, 0.0);
        for (int i = 0; i < masses.size(); i ++) {
            masses[i]->normal = normal;
        }
        /** Compute normal of each face **/
        for (int i = 0; i < faces.size()/3; i ++) { // 3 masses in each face
            Mass* n1 = faces[3*i+0];
            Mass* n2 = faces[3*i+1];
            Mass* n3 = faces[3*i+2];
            
            // Face normal
            normal = computeFaceNormal(n1, n2, n3);
            // Add all face normal
            n1->normal += normal;
            n2->normal += normal;
            n3->normal += normal;
        }
        
        for (int i = 0; i < masses.size(); i ++) {
            masses[i]->normal.normalize();
        }
	}
	
	void addForce(Vec3 f) {		 
		for (int i = 0; i < masses.size(); i++)
		{
			masses[i]->addForce(f);
		}
	}

	void computeForce(double timeStep, Vec3 gravity) {
        /** Masses **/
		for (int i = 0; i < masses.size(); i++)
		{
			masses[i]->addForce(gravity * masses[i]->m);
		}
		/** Springs **/
		for (int i = 0; i < springs.size(); i++)
		{
			springs[i]->applyInternalForce(timeStep);
		}
	}

	void integrate(double airFriction, double timeStep) {
        /** Mass* */
        for (int i = 0; i < masses.size(); i++) {
            masses[i]->integrate(timeStep);
        }
	}
	
    Vec3 getWorldPos(Mass* n) { 
        return clothPos + n->position; 
    }

    void setWorldPos(Mass* n, Vec3 pos) { 
        n->position = pos - clothPos; 
    }
    
	void collisionResponse(Ball* ball) {
        for (int i = 0; i < masses.size(); i++) {   
            /** Ball collision **/
            Vec3 distVec = getWorldPos(masses[i]) - ball->center;
            double distLen = distVec.length();
            double safeDist = ball->radius*1.05;
            if (distLen < safeDist) {
                distVec.normalize();
                setWorldPos(masses[i], distVec*safeDist+ball->center);
                masses[i]->velocity = masses[i]->velocity*ball->friction;
            }
        }
	}
};
