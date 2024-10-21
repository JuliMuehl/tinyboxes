#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "Renderer.hpp"
#include "TinyBoxes.hpp"
#include <iostream>

#include <chrono>

bool stepWorld = false;

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS){
        stepWorld = !stepWorld;
    }    
}

int main(int argc,char** argv){
    float theta = 0;
    World w = World();

    std::vector<Vector3f> cv = {{-1,-1,1},{-1,1,1},{1,1,1},{1,-1,1},{-1,-1,-1},{-1,1,-1},{1,1,-1},{1,-1,-1}};

    std::shared_ptr<ConvexCollider> collider = std::make_shared<ConvexPolyhedron>(cv);
    std::shared_ptr<ConvexCollider> collider_sphere = std::make_shared<SphereCollider>(1);

    constexpr float BOX_INVERSE_MASS = 1.0; 
    
    BodyId b11 = w.AddBody({Vector3f(0,1.0,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), BOX_INVERSE_MASS * Matrix3f::Identity(),BOX_INVERSE_MASS,collider});
    BodyId b12 = w.AddBody({Vector3f(0,3.1,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), BOX_INVERSE_MASS * Matrix3f::Identity(),BOX_INVERSE_MASS,collider});
    BodyId b13 = w.AddBody({Vector3f(0,5.2,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), BOX_INVERSE_MASS * Matrix3f::Identity(),BOX_INVERSE_MASS,collider});
    BodyId b14 = w.AddBody({Vector3f(0,7.3,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), BOX_INVERSE_MASS * Matrix3f::Identity(),BOX_INVERSE_MASS,collider});

    BodyId b21 = w.AddBody({Vector3f(-2.1,1.0,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), BOX_INVERSE_MASS * Matrix3f::Identity(),BOX_INVERSE_MASS,collider});
    BodyId b22 = w.AddBody({Vector3f(-2.1,3.1,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), BOX_INVERSE_MASS * Matrix3f::Identity(),BOX_INVERSE_MASS,collider});
    BodyId b23 = w.AddBody({Vector3f(-2.1,5.2,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), BOX_INVERSE_MASS * Matrix3f::Identity(),BOX_INVERSE_MASS,collider});
    BodyId b24 = w.AddBody({Vector3f(-2.1,7.3,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), BOX_INVERSE_MASS * Matrix3f::Identity(),BOX_INVERSE_MASS,collider});

    BodyId b31 = w.AddBody({Vector3f(-4.1,1.0,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), BOX_INVERSE_MASS * Matrix3f::Identity(),BOX_INVERSE_MASS,collider});
    BodyId b32 = w.AddBody({Vector3f(-4.1,3.1,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), BOX_INVERSE_MASS * Matrix3f::Identity(),BOX_INVERSE_MASS,collider});
    BodyId b33 = w.AddBody({Vector3f(-4.1,5.2,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), BOX_INVERSE_MASS * Matrix3f::Identity(),BOX_INVERSE_MASS,collider});
    BodyId b34 = w.AddBody({Vector3f(-4.1,7.3,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), BOX_INVERSE_MASS * Matrix3f::Identity(),BOX_INVERSE_MASS,collider});

    BodyId ball1 = w.AddBody({Vector3f(3.0,8,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), Matrix3f::Identity(),0.0,collider_sphere});
    BodyId ball2 = w.AddBody({Vector3f(3.0,11,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(50,0,0),Vector3f(0,0,0), .5 * Matrix3f::Identity(),0.5,collider});

    w.AddJoint(DistanceJoint(ball1,ball2,3));

    if(!glfwInit()){
        std::cout << "Error initializing GLFW" << std::endl;
        return 1;
    }
    auto* window = glfwCreateWindow(640,480,"Tinyboxes Visualization",nullptr,nullptr);
    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, keyCallback);
    if(glewInit()){
        std::cout << "Error initializing GLEW" << std::endl;
        return 1;
    }
    std::vector<BodyId> bodies = {b11,b12,b13,b14,b21,b22,b23,b24,b31,b32,b33,b34,ball1,ball2};
    std::shared_ptr<Mesh> sphereMesh = Mesh::CreateSphere(1.0);
    std::shared_ptr<Mesh> cubeMesh = Mesh::CreateCube(1.0,1.0,1.0);
    std::vector<std::shared_ptr<Mesh>> meshes = {};
    Material redPlastic = Material(Vector3f(0.0f,0.0f,0.0f),Vector3f(0.5f,0.0f,0.0f),Vector3f(0.7f,0.6f,0.6f ),Vector3f(0.0,0.0,0.0));
    Material blackPlastic = Material(Vector3f(0.0f,0.0f,0.0f),Vector3f(0.1f,0.1f,0.1f),Vector3f(0.5f,0.5f,0.5f ),Vector3f(0.0,0.0,0.0));
    Material turquoise = Material(Vector3f(0.1f, 0.18725f, 0.1745f),Vector3f(0.396f, 0.74151f, 0.69102f),Vector3f(0.297254f, 0.30829f, 0.306678f),Vector3f(0.0,0.0,0.0));
    Material polishedSilver = Material(Vector3f(0.19225f, 0.19225f, 0.19225f),Vector3f(0.50754f, 0.50754f, 0.50754f),Vector3f(0.508273f, 0.508273f, 0.508273f),Vector3f(0.0,0.0,0.0));;
    Material obsidian = Material(Vector3f(0.05375f, 0.05f, 0.06625f),Vector3f(0.18275f, 0.17f, 0.22525f),Vector3f(0.332741f, 0.328634f, 0.346435f),Vector3f(0.0,0.0,0.0));;
    Material gold = Material(Vector3f(0.24725f, 0.1995f, 0.0745f),Vector3f(0.75164f, 0.60648f, 0.22648f),Vector3f(0.628281f, 0.555802f, 0.366065f),Vector3f(0.1,0.1,0.1));
    std::vector<Material> materials = {};
    for(int i = 0;i<12;i++){
        meshes.push_back(cubeMesh);
        materials.push_back(polishedSilver);
    }
    for(int i = 0;i<2;i++){
        meshes.push_back(sphereMesh);
        materials.push_back(blackPlastic);
    }
    std::vector<Matrix4f> poses = {};
    for(int i = 0;i<bodies.size();i++){
        poses.push_back(w.BodyTransform(bodies[i]));
    }

    auto renderer = Renderer();
    
    glClearColor(0.0,0.0,0.0,1.0);
    glEnable(GL_DEPTH_TEST);

    CameraController cam = CameraController(45.0,640.0/480.0,0.5,1000.0);
    cam.eyePosition = Vector3f(0.0,-10.0,-10.0);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    bool processInput = true;

    while(!glfwWindowShouldClose(window)){
        if(stepWorld)
            w.step(0.01);
        Vector3f pos = w.GetPosition(ball2);
        
        for(int i = 0;i<bodies.size();i++){
            poses[i] = w.BodyTransform(bodies[i]);
        }
        renderer.Render(cam,meshes,materials,poses);
        double xpos,ypos;
        glfwGetCursorPos(window, &xpos, &ypos);
        
        int width,height;
        glfwGetWindowSize(window,&width,&height);
        cam.ResizeCallback(width,height);
        glViewport(0,0,width,height);
        if(processInput){
            cam.CursorCallback(xpos/width-0.5,ypos/height-0.5);
            
            cam.KeyCallback(window);
            if(glfwGetKey(window,GLFW_KEY_ESCAPE) == GLFW_PRESS){
                std::cout << "ESCAPE PRESSED" << std::endl;
                processInput = false;
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            }
        }
        if(!processInput && glfwGetMouseButton(window,GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS){
            processInput = true;
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        }
        glfwSwapBuffers(window);
        glfwPollEvents();
        auto end_time = std::chrono::system_clock::now();
        auto duration = (end_time-start_time);
        double fduration = std::chrono::duration<double>(duration).count();
        std::cout << "Frame Per Second: " <<   1.0/fduration << "fps" << std::endl;
    }
}
