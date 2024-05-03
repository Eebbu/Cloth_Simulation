#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <cmath>

#include "include/stb_image.h"
#include "include/cloth.h"
#include "include/rigid.h"
#include "include/program.h"
#include "include/display.h"

#define WIDTH 800
#define HEIGHT 800
#define AIR_FRICTION 0.02
#define TIME_STEP 0.01

/** Callback functions **/
void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void mouse_button_callback(GLFWwindow *window, int button, int action, int mods);
void cursor_pos_callback(GLFWwindow *window, double xpos, double ypos);

/** Global **/
// Wind
int windBlowing = 0;
int windForceScale = 15;
Vec3 windStartPos;
Vec3 windDir;
Vec3 wind;
Cloth cloth;

// Ball
Ball ball;
// Window and world
GLFWwindow *window;
Vec3 bgColor = Vec3(50.0/255, 50.0/255, 60.0/255);
Vec3 gravity(0.0, -9.8 / cloth.iteration_freq, 0.0);

int main(int argc, const char * argv[]) {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    // MacOS is forward compatible
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
    
    /** Create a GLFW window **/
    window = glfwCreateWindow(WIDTH, HEIGHT, "Cloth Simulation", NULL, NULL);
    if (window == NULL) {
        std::cout << "Failed to create GLFW window." << std::endl;
        glfwTerminate();
        return -1;
    }
    // Set the context of this window as the main context of current thread
    glfwMakeContextCurrent(window);
    
    // Initialize GLAD : this should be done before using any openGL function
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "Failed to initialize GLAD." << std::endl;
        glfwTerminate(); // This line isn't in the official source code, but I think that it should be added here.
        return -1;
    }
    
    /** Register callback functions **/
    // Callback functions should be registered after creating window and before initializing render loop
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_pos_callback);
    
    /** Renderers **/
    ClothRender clothRender(&cloth);
    ClothSpringRender clothSpringRender(&cloth);
    BallRender ballRender(&ball);
    
    Vec3 initForce(10.0, 40.0, 20.0);
    cloth.addForce(initForce);
    
    glEnable(GL_DEPTH_TEST);
    glPointSize(3);
    
    /** Redering loop **/
    while (!glfwWindowShouldClose(window)) {
        /** Set background clolor **/
        glClearColor(bgColor.x, bgColor.y, bgColor.z, 1.0); // Set color value (R,G,B,A) - Set Status
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        /** -------------------------------- Simulation & Rendering -------------------------------- **/
        for (int i = 0; i < cloth.iteration_freq; i ++) {
            cloth.computeForce(TIME_STEP, gravity);
            cloth.integrate(AIR_FRICTION, TIME_STEP);
            cloth.collisionResponse(&ball);
        }
        cloth.computeNormal();
        
        /** Display **/
        if (cloth.drawMode == Cloth::DRAW_LINES) {
            clothSpringRender.flush();
        } else {
            clothRender.flush();
        }
        ballRender.flush();
        
        /** -------------------------------- Simulation & Rendering -------------------------------- **/
        
        glfwSwapBuffers(window);
        glfwPollEvents(); // Update the status of window
    }

    glfwTerminate();
    
    return 0;
}

void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
    glViewport(0, 0, width, height);
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        windBlowing = 1;
        // Set start point of wind direction
        windStartPos.setZeroVec();
        glfwGetCursorPos(window, &windStartPos.x, &windStartPos.y);
        windStartPos.y = -windStartPos.y; // Reverse y since the screen local in the fourth quadrant
    }
    if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
        windBlowing = 0;
        windDir.setZeroVec();
    }
}

void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos) {
    /** Wind **/
    if (windBlowing) {
        windDir = Vec3(xpos, -ypos, 0) - windStartPos;
        windDir.normalize();
        wind = windDir * windForceScale;
        cloth.addForce(wind);
    }
}
