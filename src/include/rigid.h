#pragma once

#include <math.h>

#include <cmath>
#include <vector>

struct Vertex {
public:
    glm::dvec3 position;
    glm::dvec3 normal;
    
    Vertex() {}
    Vertex(glm::dvec3 pos) {
        position = pos;
    }
    ~Vertex() {}
};

class Sphere {
public:
    const int meridianNum = 24;
    const int parallelNum = 250;
    
    int radius;
    
    std::vector<Vertex*> vertexes;
    std::vector<Vertex*> faces;
    
    Sphere(int r, glm::dvec3 center) {
        radius = r;
        init();
    }

    ~Sphere() {
        for (int i = 0; i < vertexes.size(); i++) { delete vertexes[i]; }
        vertexes.clear();
        faces.clear();
    }
    
    Vertex* getTop() { 
        return vertexes[0]; 
    }

    Vertex* getVertex(int x, int y) {
        if (x < 0 || x >= parallelNum || y < 0 || y >= meridianNum) {
            printf("Vertex Index Out of Range.\n");
            exit(-1);
        } else {
            return vertexes[1+x*meridianNum+y];
        }
    }

    Vertex* getBottom() { 
        return vertexes[vertexes.size()-1]; 
    }
    
    glm::dvec3 computeFaceNormal(Vertex* v1, Vertex* v2, Vertex* v3) {
        return glm::cross(v2->position - v1->position, v3->position - v1->position);
    }
    
    void computeSphereNormal() {
        glm::dvec3 normal(0.0, 0.0, 0.0);
        for (int i = 0; i < vertexes.size(); i ++) {
            vertexes[i]->normal = normal;
        }
        
        // The normal of all faces of the first and last cycle should be calculated specially!
        for (int i = 0; i < faces.size()/3; i ++) {
            Vertex* v1 = faces[i*3+0];
            Vertex* v2 = faces[i*3+1];
            Vertex* v3 = faces[i*3+2];
            
            normal = computeFaceNormal(v1, v3, v2);
            v1->normal += normal;
            v2->normal += normal;
            v3->normal += normal;
        }
        
        for (int i = 0; i < vertexes.size(); i ++) {
            vertexes[i]->normal = glm::normalize(vertexes[i]->normal);
        }
    }
    
    void init() {
        /** Compute vertex position **/
        double cycleInterval = radius*2.0 / (parallelNum+1);
        double radianInterval = 2.0*M_PI/meridianNum;
        
        
        glm::dvec3 pos(0.0, radius, 0.0);
        vertexes.push_back(new Vertex(pos)); // Top vertex
        
        for (int i = 0; i < parallelNum; i ++) {
            pos.y -= cycleInterval;
            for (int j = 0; j < meridianNum; j ++) {
                double xzLen = radius * sqrt(1.0 - pow(pos.y/radius, 2));
                double xRadian = j * radianInterval;  // The length of projection line on X-Z pane
                
                pos.x = xzLen * sin(xRadian);
                pos.z = xzLen * cos(xRadian);
                vertexes.push_back(new Vertex(pos));
            }
        }
        pos = glm::dvec3(0.0, -radius, 0.0);
        vertexes.push_back(new Vertex(pos)); // Bottom vertex
        
        /** Slice faces **/
        // Top cycle
        for (int i = 0; i < meridianNum; i ++) {
            faces.push_back(getVertex(0, i));                               //   *   //
            faces.push_back(getTop());                                      //  / \  //
            faces.push_back(getVertex(0, (i+1)%meridianNum));               // *---* //
        }
        // Middle cycles
        for (int i = 0; i < parallelNum-1; i ++) {
            for (int j = 0; j < meridianNum; j ++) {
                faces.push_back(getVertex(i, j));                           //  *--* //
                faces.push_back(getVertex(i, (j+1)%meridianNum));           //  | /  //
                faces.push_back(getVertex(i+1, j));                         //  *    //
                
                faces.push_back(getVertex(i+1, (j+1)%meridianNum));         //     * //
                faces.push_back(getVertex(i+1, j));                         //   / | //
                faces.push_back(getVertex(i, (j+1)%meridianNum));           //  *--* //
            }
        }
        // Bottom cycle
        for (int i = 0; i < meridianNum; i ++) {
            faces.push_back(getBottom());                                   // *---* //
            faces.push_back(getVertex(parallelNum-1, i));                   //  \ /  //
            faces.push_back(getVertex(parallelNum-1, (i+1)%meridianNum));   //   *   //
        }
        
        /** Set normals **/
        computeSphereNormal();
    }
};

struct Ball {
    const glm::vec3       center = glm::vec3(0, 6, 0);
    const int        radius = 1;
    const glm::vec4  color = glm::vec4(0.6f, 0.5f, 0.8f, 1.0f);
    const double     friction = 0.8;
    
    Sphere* sphere;
    
    Ball() {
        sphere = new Sphere(radius, center);
    }
    ~Ball() {}
};
