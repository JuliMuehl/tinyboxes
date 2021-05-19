## TinyBoxes 
A small rigid body dynamics engine written in c++.
The engine currently features collision detection between convex shapes using the GJK and EPA algorithms aswell as collision resolution using  Projected Gauss Seidel Iteration (a.k.a. Sequential Impulses). The engine also implements a simple distance constraint and additional joints (hinge, ball and socket joint) will be added soon.

TinyBoxes has no external dependencies despite OpenGL and GLFW which are only required for the demos.

## Demos
<img src = "https://i.postimg.cc/d3tPpnZC/Wrecking-Ball-Screenshot.gif" width=300 height=300>
<img src = "https://i.postimg.cc/XJgkTwTS/Driving-Screenshot.gif" width=300 height=300>


## Usage

```c++
#include "TinyBoxes.hpp"

int main(){
    //Create World
    World w = World(); 
    //Create a Collider for the bodies
    std::shared_ptr<ConvexCollider> collider_sphere = std::make_shared<SphereCollider>(1); 

    //Create a body and add it to the world
    uint64_t body1 = w.AddBody({Vector3f(0,1,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), Matrix3f::Identity(),0.0,collider_sphere});
    //Create a second body and add it to the world
    uint64_t body2 = w.AddBody({Vector3f(0,4,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), Matrix3f::Identity(),1.0,collider_sphere});
    //Creates a third body and add it to the world
    uint64_t body3 = w.AddBody({Vector3f(0,10,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), Matrix3f::Identity(),1.0,collider_sphere});
    //Create a joint and add it to the world
    w.AddJoint(DistanceJoint(body1,body2,3.0)); 
    while(true){
        w.step(.01f); // Make an integration step of size .01
        //Get the position and orientation of the bodies as 4x4 Matrices
        Matrix4f mat1 = w.BodyTransform(body1);
        Matrix4f mat2 = w.BodyTransform(body2);
        Matrix4f mat3 = w.BodyTransform(body3);
        //Use your draw code with the positions and orientations of the bodies (this part is up to you)
        yourDrawCode(mat1);
        yourDrawCode(mat2);
        yourDrawCode(mat3);
    }
}

``` 

Have a look at the demos/ directory for additional examples of how the engine can be used.

## Build Instructions
You can use the following command to build the demos (requires g++, glfw and OpenGl to be installed on your system)

```bash 
g++ -lGL -lglfw -lm -I include src/*.cpp demos/<insert_demo_name>.cpp -o <insert_demo_name>
``` 


