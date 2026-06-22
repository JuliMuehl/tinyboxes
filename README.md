



## TinyBoxes 
A small 3D rigid body dynamics engine written in C++.
The engine currently features collision detection between convex shapes using the GJK and EPA algorithms as well as collision resolution using  Projected Gauss Seidel Iteration (a.k.a. Sequential Impulses). The engine also implements a simple distance constraint and additional joints (hinge, ball and socket joint) may be added in the future.

TinyBoxes has no external dependencies despite OpenGL/GLEW and GLFW which are only required for the demos.

## Demos

[WreckingBallDemo.mp4](https://github.com/user-attachments/assets/d8fbcfd1-8e86-410c-90b9-182506b7003c)

## Usage

```c++
#include "TinyBoxes.hpp"

int main(){
    //Create World
    World w = World(); 
    //Create a Collider for the bodies
    std::shared_ptr<ConvexCollider> collider_sphere = std::make_shared<SphereCollider>(1); 
    //Create a body and add it to the world
    BodyId body1 = w.AddBody({Vector3f(0,4,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), Matrix3f::Identity(),0.0,collider_sphere});
    //Create a second body and add it to the world
    BodyId body2 = w.AddBody({Vector3f(0,7,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), Matrix3f::Identity(),1.0,collider_sphere});
    //Create a third body and add it to the world
    BodyId body3 = w.AddBody({Vector3f(0,10,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), Matrix3f::Identity(),1.0,collider_sphere});
    //Create a joint and add it to the world
    w.AddJoint(DistanceJoint(body1,body2,3.0)); 
    while(true){
        // Make a (euler) integration step with step size .01
        w.step(.01f); 
        //Get the positions and orientations of the bodies as 4x4 Matrices
        Matrix4f mat1 = w.BodyTransform(body1);
        Matrix4f mat2 = w.BodyTransform(body2);
        Matrix4f mat3 = w.BodyTransform(body3);
        //Use your draw code with the positions and orientations of the bodies (this part is up to you)
        YourDrawCode(mat1);
        YourDrawCode(mat2);
        YourDrawCode(mat3);
    }
}

``` 

Have a look at the demos/ directory for additional examples of how the engine can be used.

## Build Instructions
You can use the following command to build the project (including the demos requires cmake, glfw, GLEW and OpenGL to be installed on your system)

```bash 
cmake -B build
make -C build
```
You can run the demos using e.g.
```bash
build/demos/WreckingBallDemo
```
Press Space to pause/unpause the demo simulations.
Use arrow keys and mouse to navigate the camera.
