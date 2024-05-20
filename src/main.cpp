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
#include "include/render.h"
#include <thread>

#define WIDTH 800
#define HEIGHT 800
#define AIR_FRICTION 0.02
#define TIME_STEP 0.01

using namespace std;
/** Callback functions **/
void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void mouse_button_callback(GLFWwindow *window, int button, int action, int mods);
void cursor_pos_callback(GLFWwindow *window, double xpos, double ypos);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);


/** Global **/
// Wind
int windBlowing = 0;
double windForceScale = 15.0;
glm::dvec3 windStartPos;
glm::dvec3 windDir;
glm::dvec3 wind;
Cloth cloth;
//show constraint
bool constraint = true;


// Ball&Cube&rectangle
Ball ball;
Cube cube;
Rectangle rectangle;
RigidType currentRigidType = RigidType::Empty;  // Default to Emptu
//Boolean show ball or cube
bool showBall = false;
bool showCube = false;
bool showRect = false;
void *obj;

// Window and world
GLFWwindow *window;
glm::dvec3 bgColor = glm::dvec3(0.0/255, 0.0/255, 0.0/255);


int main(int argc, const char * argv[]) {
    string method = argc > 1 ? argv[1] : "Euler";  //default method is Euler
    cout << method << endl;
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
    glfwSetKeyCallback(window, key_callback);

    /** Renderers **/
    ClothRender clothRender(&cloth);
    ClothSpringRender clothSpringRender(&cloth);
    BallRender ballRender(&ball);
    CubeRender cubeRender(&cube);
    RectangleRender rectRender(&rectangle);
    // Vec3 initForce(10.0, 40.0, 20.0);
    // cloth.addForce(initForce);
    
    glEnable(GL_DEPTH_TEST);
    glPointSize(3);
    
    /** Redering loop **/
    while (!glfwWindowShouldClose(window)) {
        /** Set background clolor **/
        glClearColor(bgColor.x, bgColor.y, bgColor.z, 1.0); // Set color value (R,G,B,A) - Set Status
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        
        /** -------------------------------- Simulation & Rendering -------------------------------- **/
        if(currentRigidType == RigidType::Ball){
            obj = static_cast<void*>(&ball);
        }else if(currentRigidType == RigidType::Cube){
            obj = static_cast<void*>(&cube);
        }else if(currentRigidType == RigidType::Rectangle){
            obj = static_cast<void*>(&rectangle);
        }else{
            obj = nullptr;
        }
        
        
        for (int i = 0; i < 25; i ++) {
            if (method == "RK") {
                cloth.rk4_step(constraint, currentRigidType, obj, TIME_STEP);
            } else if (method == "VERLET") {
                cloth.explicit_verlet(constraint, currentRigidType, obj, TIME_STEP);
            } else {
                cloth.step(constraint, currentRigidType, obj, TIME_STEP);
            }
        }
        cloth.compute_normal();

        /** Display **/
        if (cloth.draw_texture) {
            clothRender.flush();
        } else {
            clothSpringRender.flush();
        }
        //control ball & cube rendering
        if (showCube) {
            cubeRender.flush();
        }
        
        if (showBall) {
            ballRender.flush();
        }
        
        if(showRect){
            rectRender.flush();
        }
        
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
        windStartPos = glm::dvec3(0, 0, 0);
        glfwGetCursorPos(window, &windStartPos.x, &windStartPos.y);
        windStartPos.y = -windStartPos.y; // Reverse y since the screen local in the fourth quadrant
    }
    if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
        windBlowing = 0;
        windDir = glm::dvec3(0, 0, 0);
    }
}

void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos) {
    /** Wind **/
    if (windBlowing) {
        windDir = glm::dvec3(xpos, -ypos, 0) - windStartPos;
        windDir = glm::normalize(windDir);
        wind = windDir * windForceScale;
        cloth.add_force(wind);
    }
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    //reset the simulation when press R
    if (key == GLFW_KEY_R && action == GLFW_PRESS) {

        cloth.reset();
        cout << "----------Simulation reset-----------" << endl;
    }

    //add texture when press TS
    if (key == GLFW_KEY_T && action == GLFW_PRESS) {
        cloth.draw_texture = (1 - cloth.draw_texture);
    }

    //show cube when press C
    if (key == GLFW_KEY_C && action == GLFW_PRESS) {
        showCube = (1 - showCube);
        showBall = false;
        showRect = false;
        if(currentRigidType == RigidType::Cube){
            currentRigidType = RigidType::Empty;
            cout << "----------Hide Cube-----------" << endl;
        }else{
            currentRigidType = RigidType::Cube;
            cout << "----------Show Cube-----------" << endl;
        }
    }
    //show rectangle when press E
    if (key == GLFW_KEY_E && action == GLFW_PRESS) {
        showCube = false;
        showBall = false;
        showRect = (1 - showRect);
        if(currentRigidType == RigidType::Rectangle){
            currentRigidType = RigidType::Empty;
            cout << "----------Hide Rectangle-----------" << endl;
        }else{
            currentRigidType = RigidType::Rectangle;
            cout << "----------Show Rectangle-----------" << endl;
        }
    }

    //show ball when press B
    if (key == GLFW_KEY_B && action == GLFW_PRESS) {
        showBall = (1 - showBall);
        showCube = false;
        showRect = false;
        if(currentRigidType == RigidType::Ball){
            currentRigidType = RigidType::Empty;
            cout << "----------Hide Ball-----------" << endl;
        }else{
            currentRigidType = RigidType::Ball;
            cout << "----------Show Ball-----------" << endl;
        }
    }

    //add constraint when press A
    if (key == GLFW_KEY_A && action == GLFW_PRESS) {
        constraint = (1 - constraint);
        cout << "----------Add constraint-----------" << endl;
    }

    //close windoow when press Esc
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GL_TRUE);
    }


}
