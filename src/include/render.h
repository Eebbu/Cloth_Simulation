#pragma once

#include <iostream>

#include "cloth.h"
#include "rigid.h"
#include "program.h"
#include "stb_image.h"

struct Camera {
    const float speed        = 0.05f;
    const float frustumRatio = 1.0f;
    
    glm::vec3 pos   = glm::vec3(0.0f, 10.0f, 25.0f);
    glm::vec3 front = glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 up    = glm::vec3(0.0f, 1.0f, 0.0f);
    
    glm::mat4 uniProjMatrix;
    glm::mat4 uniViewMatrix;
    
    Camera() {
        /** Projection matrix : The frustum that camera observes **/
        uniProjMatrix = glm::mat4(1.0f);
        uniProjMatrix = glm::perspective(glm::radians(45.0f), frustumRatio, 0.1f, 50.0f);
        /** View Matrix : The camera **/
        uniViewMatrix = glm::mat4(1.0f);
    }
};
Camera cam;

struct Light {
    glm::vec3 pos   = glm::vec3(-5.0f, 4.0f, 12.0f);
    glm::vec3 color = glm::vec3(0.7f, 0.7f, 1.0f);
};
Light sun;

struct ClothRender {
    const Cloth* cloth;
    int massCount; // Number of all masses in faces
    
    glm::vec3 *vboPos; // Position
    glm::vec2 *vboTex; // Texture
    glm::vec3 *vboNor; // Normal

    GLuint programID;
    GLuint vaoID;
    GLuint vboIDs[3];
    GLuint texID;
    
    GLint aPtrPos;
    GLint aPtrTex;
    GLint aPtrNor;
    
    ClothRender(Cloth* cloth) {
        massCount = (int)(cloth->faces.size());
        if (massCount <= 0) {
            std::cout << "ERROR::ClothRender : No mass exists." << std::endl;
            exit(-1);
        }
        
        this->cloth = cloth;
        
        vboPos = new glm::vec3[massCount];
        vboTex = new glm::vec2[massCount];
        vboNor = new glm::vec3[massCount];
        for (int i = 0; i < massCount; i ++) {
            Mass* m = cloth->faces[i];
            vboPos[i] = glm::vec3(m->position);
            vboTex[i] = glm::vec2(m->tex_coord); // Texture coord will only be set here
            vboNor[i] = glm::vec3(m->normal);
        }
        
        /** Build render program **/
        Program program("../shaders/cloth.vs", "../shaders/cloth.fs");
        programID = program.ID;
        std::cout << "Cloth Program ID: " << programID << std::endl;

        // Generate ID of VAO and VBOs
        glGenVertexArrays(1, &vaoID);
        glGenBuffers(3, vboIDs);
        
        // Attribute pointers of VAO
        aPtrPos = 0;
        aPtrTex = 1;
        aPtrNor = 2;
        // Bind VAO
        glBindVertexArray(vaoID);
        
        // Position buffer
        glBindBuffer(GL_ARRAY_BUFFER, vboIDs[0]);
        glVertexAttribPointer(aPtrPos, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
        glBufferData(GL_ARRAY_BUFFER, massCount*sizeof(glm::vec3), vboPos, GL_DYNAMIC_DRAW);
        // Texture buffer
        glBindBuffer(GL_ARRAY_BUFFER, vboIDs[1]);
        glVertexAttribPointer(aPtrTex, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);
        glBufferData(GL_ARRAY_BUFFER, massCount*sizeof(glm::vec2), vboTex, GL_DYNAMIC_DRAW);
        // Normal buffer
        glBindBuffer(GL_ARRAY_BUFFER, vboIDs[2]);
        glVertexAttribPointer(aPtrNor, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
        glBufferData(GL_ARRAY_BUFFER, massCount*sizeof(glm::vec3), vboNor, GL_DYNAMIC_DRAW);
        
        // Enable it's attribute pointers since they were set well
        glEnableVertexAttribArray(aPtrPos);
        glEnableVertexAttribArray(aPtrTex);
        glEnableVertexAttribArray(aPtrNor);
        
        /** Load texture **/
        // Assign texture ID and gengeration
        glGenTextures(1, &texID);
        glBindTexture(GL_TEXTURE_2D, texID);
        // Set the texture wrapping parameters (for 2D tex: S, T)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        // Set texture filtering parameters (Minify, Magnify)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        /** Load image and configure texture **/
        stbi_set_flip_vertically_on_load(true);
        int texW, texH, colorChannels; // After loading the image, stb_image will fill them
        unsigned char *data = stbi_load("../textures/texture1.jpeg", &texW, &texH, &colorChannels, 0);
        if (data) {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texW, texH, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
            // Automatically generate all the required mipmaps for the currently bound texture.
            glGenerateMipmap(GL_TEXTURE_2D);
        } else {
            std::cout << "Failed to load texture" << std::endl;
        }
        // Always free image memory
        stbi_image_free(data);
        
        /** Set uniform **/
        glUseProgram(programID); // Active shader before set uniform
        // Set texture sampler
        glUniform1i(glGetUniformLocation(programID, "uniTex"), 0);
        
        /** Projection matrix : The frustum that camera observes **/
        // Since projection matrix rarely changes, set it outside the rendering loop for only onec time
        glUniformMatrix4fv(glGetUniformLocation(programID, "uniProjMatrix"), 1, GL_FALSE, &cam.uniProjMatrix[0][0]);
        
        /** Model Matrix : Put cloth into the world **/
        glm::mat4 uniModelMatrix = glm::mat4(1.0f);
        uniModelMatrix = glm::translate(uniModelMatrix, cloth->cloth_pos);
        glUniformMatrix4fv(glGetUniformLocation(programID, "uniModelMatrix"), 1, GL_FALSE, &uniModelMatrix[0][0]);
        
        /** Light **/
        glUniform3fv(glGetUniformLocation(programID, "uniLightPos"), 1, &(sun.pos[0]));
        glUniform3fv(glGetUniformLocation(programID, "uniLightColor"), 1, &(sun.color[0]));

        // Cleanup
        glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbined VBO
        glBindVertexArray(0); // Unbined VAO
    }
    
    void destroy() {
        delete [] vboPos;
        delete [] vboTex;
        delete [] vboNor;
        
        if (vaoID) {
            glDeleteVertexArrays(1, &vaoID);
            glDeleteBuffers(3, vboIDs);
            vaoID = 0;
        }
        
        if (programID) {
            glDeleteProgram(programID);
            programID = 0;
        }
    }
    
    void flush() {
        // Update all the positions of masses
        for (int i = 0; i < massCount; i ++) { // Tex coordinate dose not change
            Mass* m = cloth->faces[i];
            vboPos[i] = glm::vec3(m->position);
            vboNor[i] = glm::vec3(m->normal);
        }
        
        glUseProgram(programID);
        
        glBindVertexArray(vaoID);
        
        glBindBuffer(GL_ARRAY_BUFFER, vboIDs[0]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, massCount*sizeof(glm::vec3), vboPos);
        glBindBuffer(GL_ARRAY_BUFFER, vboIDs[1]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, massCount* sizeof(glm::vec2), vboTex);
        glBindBuffer(GL_ARRAY_BUFFER, vboIDs[2]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, massCount* sizeof(glm::vec3), vboNor);
        
        /** Bind texture **/
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texID);
        
        /** View Matrix : The camera **/
        cam.uniViewMatrix = glm::lookAt(cam.pos, cam.pos + cam.front, cam.up);
        glUniformMatrix4fv(glGetUniformLocation(programID, "uniViewMatrix"), 1, GL_FALSE, &cam.uniViewMatrix[0][0]);
        
        glEnable(GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        
        /** Draw **/
        if (cloth->draw_texture) {
            glDrawArrays(GL_TRIANGLES, 0, massCount);
        } else {
            glDrawArrays(GL_LINES, 0, massCount);
        }
        
        // End flushing
        glDisable(GL_BLEND);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        glUseProgram(0);
    }
};

struct SpringRender {
    std::vector<Spring*> springs;
    int springCount; // Number of masses in springs
    
    glm::vec4 uniSpringColor;
    
    glm::vec3 *vboPos; // Position
    glm::vec3 *vboNor; // Normal

    GLuint programID;
    GLuint vaoID;
    GLuint vboIDs[2];
    
    GLint aPtrPos;
    GLint aPtrNor;
    
    // Render any spring set, color and modelVector
    void init(std::vector<Spring*> s, glm::vec4 c, glm::vec3 modelVec) {
        springs = s;
        springCount = (int)(springs.size());
        if (springCount <= 0) {
            std::cout << "ERROR::SpringRender : No mass exists." << std::endl;
            exit(-1);
        }
        
        uniSpringColor = c;
        
        vboPos = new glm::vec3[springCount*2];
        vboNor = new glm::vec3[springCount*2];
        for (int i = 0; i < springCount; i ++) {
            Mass* mass1 = springs[i]->mass1;
            Mass* mass2 = springs[i]->mass2;
            vboPos[i*2] = glm::vec3(mass1->position);
            vboPos[i*2+1] = glm::vec3(mass2->position);
            vboNor[i*2] = glm::vec3(mass1->normal);
            vboNor[i*2+1] = glm::vec3(mass2->normal);
        }
        
        /** Build render program **/
        Program program("../shaders/spring.vs", "../shaders/spring.fs");
        programID = program.ID;
        std::cout << "Spring Program ID: " << programID << std::endl;

        // Generate ID of VAO and VBOs
        glGenVertexArrays(1, &vaoID);
        glGenBuffers(2, vboIDs);
        
        // Attribute pointers of VAO
        aPtrPos = 0;
        aPtrNor = 1;
        // Bind VAO
        glBindVertexArray(vaoID);
        
        // Position buffer
        glBindBuffer(GL_ARRAY_BUFFER, vboIDs[0]);
        glVertexAttribPointer(aPtrPos, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
        glBufferData(GL_ARRAY_BUFFER, springCount*2*sizeof(glm::vec3), vboPos, GL_DYNAMIC_DRAW);
        // Normal buffer
        glBindBuffer(GL_ARRAY_BUFFER, vboIDs[1]);
        glVertexAttribPointer(aPtrNor, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
        glBufferData(GL_ARRAY_BUFFER, springCount*2*sizeof(glm::vec3), vboNor, GL_DYNAMIC_DRAW);
        
        // Enable it's attribute pointers since they were set well
        glEnableVertexAttribArray(aPtrPos);
        glEnableVertexAttribArray(aPtrNor);
        
        /** Set uniform **/
        glUseProgram(programID); // Active shader before set uniform
        // Set color
        glUniform4fv(glGetUniformLocation(programID, "uniSpringColor"), 1, &uniSpringColor[0]);
        
        /** Projection matrix : The frustum that camera observes **/
        // Since projection matrix rarely changes, set it outside the rendering loop for only onec time
        glUniformMatrix4fv(glGetUniformLocation(programID, "uniProjMatrix"), 1, GL_FALSE, &cam.uniProjMatrix[0][0]);
        
        /** Model Matrix : Put rigid into the world **/
        glm::mat4 uniModelMatrix = glm::mat4(1.0f);
        uniModelMatrix = glm::translate(uniModelMatrix, modelVec);
        glUniformMatrix4fv(glGetUniformLocation(programID, "uniModelMatrix"), 1, GL_FALSE, &uniModelMatrix[0][0]);
        
        /** Light **/
        glUniform3fv(glGetUniformLocation(programID, "uniLightPos"), 1, &(sun.pos[0]));
        glUniform3fv(glGetUniformLocation(programID, "uniLightColor"), 1, &(sun.color[0]));

        // Cleanup
        glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbined VBO
        glBindVertexArray(0); // Unbined VAO
    }
    
    void destroy() {
        delete [] vboPos;
        delete [] vboNor;
        
        if (vaoID) {
            glDeleteVertexArrays(1, &vaoID);
            glDeleteBuffers(2, vboIDs);
            vaoID = 0;
        }
        if (programID) {
            glDeleteProgram(programID);
            programID = 0;
        }
    }
    
    void flush() {
        // Update all the positions of masses
        for (int i = 0; i < springCount; i ++) {
            Mass* mass1 = springs[i]->mass1;
            Mass* mass2 = springs[i]->mass2;
            vboPos[i*2] = glm::vec3(mass1->position);
            vboPos[i*2+1] = glm::vec3(mass2->position);
            vboNor[i*2] = glm::vec3(mass1->normal);
            vboNor[i*2+1] = glm::vec3(mass2->normal);
        }
        
        glUseProgram(programID);
        
        glBindVertexArray(vaoID);
        
        glBindBuffer(GL_ARRAY_BUFFER, vboIDs[0]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, springCount*2*sizeof(glm::vec3), vboPos);
        glBindBuffer(GL_ARRAY_BUFFER, vboIDs[1]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, springCount*2*sizeof(glm::vec3), vboNor);
        
        /** View Matrix : The camera **/
        cam.uniViewMatrix = glm::lookAt(cam.pos, cam.pos + cam.front, cam.up);
        glUniformMatrix4fv(glGetUniformLocation(programID, "uniViewMatrix"), 1, GL_FALSE, &cam.uniViewMatrix[0][0]);
        
        glEnable(GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        
        /** Draw **/
        glDrawArrays(GL_LINES, 0, springCount*2);
        
        // End flushing
        glDisable(GL_BLEND);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        glUseProgram(0);
    }
};

struct ClothSpringRender {
    Cloth *cloth;
    glm::vec4 defaultColor;
    SpringRender render;
    
    ClothSpringRender(Cloth* c) {
        cloth = c;
        defaultColor = glm::vec4(1.0, 1.0, 1.0, 1.0);
        render.init(cloth->springs, defaultColor, cloth->cloth_pos);
    }
    
    void flush() { render.flush(); }
};

struct RigidRender {
    std::vector<Vertex*> faces;
    int vertexCount; // Number of masses in faces
    
    glm::vec4 uniRigidColor;
    
    glm::vec3 *vboPos; // Position
    glm::vec3 *vboNor; // Normal

    GLuint programID;
    GLuint vaoID;
    GLuint vboIDs[2];
    
    GLint aPtrPos;
    GLint aPtrNor;
    
    // Render any rigid body only with it's faces, color and modelVector
    void init(std::vector<Vertex*> f, glm::vec4 c, glm::vec3 modelVec) {
        faces = f;
        vertexCount = (int)(faces.size());
        if (vertexCount <= 0) {
            std::cout << "ERROR::RigidRender : No vertex exists." << std::endl;
            exit(-1);
        }
        
        uniRigidColor = c;
        
        vboPos = new glm::vec3[vertexCount];
        vboNor = new glm::vec3[vertexCount];
        for (int i = 0; i < vertexCount; i ++) {
            Vertex* v = faces[i];
            vboPos[i] = glm::vec3(v->position);
            vboNor[i] = glm::vec3(v->normal);
        }
        
        /** Build render program **/
        Program program("../shaders/rigid.vs", "../shaders/rigid.fs");
        programID = program.ID;
        std::cout << "Rigid Program ID: " << programID << std::endl;

        // Generate ID of VAO and VBOs
        glGenVertexArrays(1, &vaoID);
        glGenBuffers(2, vboIDs);
        
        // Attribute pointers of VAO
        aPtrPos = 0;
        aPtrNor = 1;
        // Bind VAO
        glBindVertexArray(vaoID);
        
        // Position buffer
        glBindBuffer(GL_ARRAY_BUFFER, vboIDs[0]);
        glVertexAttribPointer(aPtrPos, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
        glBufferData(GL_ARRAY_BUFFER, vertexCount*sizeof(glm::vec3), vboPos, GL_DYNAMIC_DRAW);
        // Normal buffer
        glBindBuffer(GL_ARRAY_BUFFER, vboIDs[1]);
        glVertexAttribPointer(aPtrNor, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
        glBufferData(GL_ARRAY_BUFFER, vertexCount*sizeof(glm::vec3), vboNor, GL_DYNAMIC_DRAW);
        
        // Enable it's attribute pointers since they were set well
        glEnableVertexAttribArray(aPtrPos);
        glEnableVertexAttribArray(aPtrNor);
        
        /** Set uniform **/
        glUseProgram(programID); // Active shader before set uniform
        // Set color
        glUniform4fv(glGetUniformLocation(programID, "uniRigidColor"), 1, &uniRigidColor[0]);
        
        /** Projection matrix : The frustum that camera observes **/
        // Since projection matrix rarely changes, set it outside the rendering loop for only onec time
        glUniformMatrix4fv(glGetUniformLocation(programID, "uniProjMatrix"), 1, GL_FALSE, &cam.uniProjMatrix[0][0]);
        
        /** Model Matrix : Put rigid into the world **/
        glm::mat4 uniModelMatrix = glm::mat4(1.0f);
        uniModelMatrix = glm::translate(uniModelMatrix, modelVec);
        glUniformMatrix4fv(glGetUniformLocation(programID, "uniModelMatrix"), 1, GL_FALSE, &uniModelMatrix[0][0]);
        
        /** Light **/
        glUniform3fv(glGetUniformLocation(programID, "uniLightPos"), 1, &(sun.pos[0]));
        glUniform3fv(glGetUniformLocation(programID, "uniLightColor"), 1, &(sun.color[0]));

        // Cleanup
        glBindBuffer(GL_ARRAY_BUFFER, 0); // Unbined VBO
        glBindVertexArray(0); // Unbined VAO
    }
    
    void destroy() {
        delete [] vboPos;
        delete [] vboNor;
        
        if (vaoID) {
            glDeleteVertexArrays(1, &vaoID);
            glDeleteBuffers(2, vboIDs);
            vaoID = 0;
        }
        if (programID) {
            glDeleteProgram(programID);
            programID = 0;
        }
    }
    
    void flush() {
        glUseProgram(programID);
        
        glBindVertexArray(vaoID);
        
        glBindBuffer(GL_ARRAY_BUFFER, vboIDs[0]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, vertexCount*sizeof(glm::vec3), vboPos);
        glBindBuffer(GL_ARRAY_BUFFER, vboIDs[1]);
        glBufferSubData(GL_ARRAY_BUFFER, 0, vertexCount*sizeof(glm::vec3), vboNor);
        
        /** View Matrix : The camera **/
        cam.uniViewMatrix = glm::lookAt(cam.pos, cam.pos + cam.front, cam.up);
        glUniformMatrix4fv(glGetUniformLocation(programID, "uniViewMatrix"), 1, GL_FALSE, &cam.uniViewMatrix[0][0]);
        
        glEnable(GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        
        /** Draw **/
        glDrawArrays(GL_TRIANGLES, 0, vertexCount);
        
        // End flushing
        glDisable(GL_BLEND);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
        glUseProgram(0);
    }
};

struct BallRender {
    Ball* ball;
    RigidRender render;
    
    BallRender(Ball* b) {
        ball = b;
        render.init(ball->sphere->faces, ball->color, ball->center);
    }
    
    void flush() { render.flush(); }
};
struct CubeRender {
    Cube* cube;
    RigidRender render;
    
    CubeRender(Cube* c) {
        cube = c;
        render.init(cube->faces, cube->color, glm::vec3(0, 0, 0));
    }
    
    void flush() { render.flush(); }
};
struct RectangleRender {
    Rectangle* rectangle;
    RigidRender render;
    
    RectangleRender(Rectangle* rectangle) {
        rectangle = rectangle;
        render.init(rectangle->faces, rectangle->color, glm::vec3(0, 0, 0));
    }
    
    void flush() { render.flush(); }
};