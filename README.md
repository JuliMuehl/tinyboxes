## TinyBoxes 
A small rigid body dynamics engine written in c++.
The engine currently features collsiion detection between convex shapes using the GJK and EPA algorithms aswell as collision resolution using Sequential Impulses. The engine also implements a simple distance constraint and additional joints (hinge, ball and socket joint) will be added soon.

There are no external dependencies despite Opengl and GLFW which are only required for the demos.

## Demos:
<img src = "https://i.postimg.cc/d3tPpnZC/Wrecking-Ball-Screenshot.gif" width=300 height=300>
<img src = "https://i.postimg.cc/XJgkTwTS/Driving-Screenshot.gif" width=300 height=300>

##Usage

```c++
#include "tinyboxes.hpp"

int main(){
    World w = World(); // Creates World
    std::shared_ptr<ConvexCollider> collider_sphere = std::make_shared<SphereCollider>(1); // create a Collider for the body

    uint64_t body1 = w.AddBody({Vector3f(0,1,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), Matrix3f::Identity(),0.0,collider_sphere});//Creates a body and adds it to the world
    uint64_t body2 = w.AddBody({Vector3f(0,4,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(1,0,0), Matrix3f::Identity(),1.0,collider_sphere});//Creates a second body and adds it to the world
    w.AddJoint(DistanceJoint(body1,body2,3.0)); // Creates a joint and adds it to the world
    while(true){
        w.step(.01f); // Make an integration step of size .01
        //Get the position and orientation of the bodies as 4x4 Matrices
        Matrix4f mat1 = w.BodyTransform(body1);
        Matrix4f mat2 = w.BodyTransform(body2);
        //Draw the matrices (this part is up to you)
        yourDrawCode(mat1);
        yourDrawCode(mat2);
    }
}

``` 

## Build Instructions
In order to build the demos use the following command (requires g++, glfw, OpenGl to be installed on your system)

```bash 
g++ -lGL -lglfw -lm -I include src/*.cpp demos/<insert_demo_name>.cpp -o <insert_demo_name_binary>
``` 


