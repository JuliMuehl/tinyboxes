#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <thread>
#include <chrono>

#include "TinyBoxes.hpp"

static void drawFace(){
    glBegin(GL_TRIANGLES);
        glVertex3f(1,-1,1);
        glVertex3f(-1,-1,1);
        glVertex3f(1,1,1);

        glVertex3f(-1,-1,1);
        glVertex3f(1,1,1);
        glVertex3f(-1,1,1);
    glEnd();
}

static void drawSphere(float radius){
    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
    glColor3f(0,.8,.7);
    auto x = [](float u,float v){return cosf(u) * cosf(v);};
    auto y = [](float u,float v){return cosf(u) * sinf(v);};
    auto z = [](float u,float v){return sinf(u);};

    float n = 16.0f;
    glPushMatrix();
        glScalef(radius,radius,radius);
        glBegin(GL_TRIANGLES);
        for(int i = 0;i<n;i++){
            for(int j = 0;j<n;j++){
                float u = i/n * 2*M_PI;
                float u_ = (i+1)/n * 2*M_PI;
                float v = j/n * 2*M_PI;
                float v_ = (j+1)/n * 2*M_PI;
                glVertex3f(x(u,v),y(u,v),z(u,v));
                glVertex3f(x(u_,v_),y(u_,v_),z(u_,v_));
                glVertex3f(x(u_,v),y(u_,v),z(u_,v));

                glVertex3f(x(u,v),y(u,v),z(u,v));
                glVertex3f(x(u_,v_),y(u_,v_),z(u_,v_));
                glVertex3f(x(u,v_),y(u,v_),z(u,v_));
            }
        }
        glEnd();
    glPopMatrix();
    glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
    glColor3f(.0,.0,.0);
    glPushMatrix();
        glScalef(radius,radius,radius);
        glBegin(GL_TRIANGLES);
        for(int i = 0;i<n;i++){
            for(int j = 0;j<n;j++){
                float u = i/n * 2*M_PI;
                float u_ = (i+1)/n * 2*M_PI;
                float v = j/n * 2*M_PI;
                float v_ = (j+1)/n * 2*M_PI;
                glVertex3f(x(u,v),y(u,v),z(u,v));
                glVertex3f(x(u_,v_),y(u_,v_),z(u_,v_));
                glVertex3f(x(u_,v),y(u_,v),z(u_,v));

                glVertex3f(x(u,v),y(u,v),z(u,v));
                glVertex3f(x(u_,v_),y(u_,v_),z(u_,v_));
                glVertex3f(x(u,v_),y(u,v_),z(u,v_));
            }
        }
        glEnd();
    glPopMatrix();
}

static void drawCube(float scale=1){
    glColor3f(0,.8f,.7f);
    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
    glPushMatrix();
        glScalef(scale,scale,scale);
        glPushMatrix();
            glRotatef(0,1,0,0);
            drawFace();
        glPopMatrix();
        glPushMatrix();
            glRotatef(90,1,0,0);
            drawFace();
        glPopMatrix();
        glPushMatrix();
            glRotatef(180,1,0,0);
            drawFace();
        glPopMatrix();
        glPushMatrix();
            glRotatef(270,1,0,0);
            drawFace();
        glPopMatrix();
        glPushMatrix();
            glRotatef(90,0,1,0);
            drawFace();
        glPopMatrix();
        glPushMatrix();
            glRotatef(270,0,1,0);
            drawFace();
        glPopMatrix();
    glPopMatrix();
    glColor3f(0,0,0);
    glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
    glPushMatrix();
        glScalef(scale,scale,scale);
        glPushMatrix();
            glRotatef(0,1,0,0);
            drawFace();
        glPopMatrix();
        glPushMatrix();
            glRotatef(90,1,0,0);
            drawFace();
        glPopMatrix();
        glPushMatrix();
            glRotatef(180,1,0,0);
            drawFace();
        glPopMatrix();
        glPushMatrix();
            drawFace();
        glPopMatrix();
        glPushMatrix();
            glRotatef(90,0,1,0);
            drawFace();
        glPopMatrix();
        glPushMatrix();
            glRotatef(270,0,1,0);
            drawFace();
        glPopMatrix();
    glPopMatrix();
}


int main(){
    glfwInit();
    GLFWwindow* window = glfwCreateWindow(800,800,"Boxes",NULL,NULL);
    glfwMakeContextCurrent(window);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-1,1,-1,1,1,100);
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_MODELVIEW);
    glClearColor(0,0,0,1);
    float theta = 0;
    World w = World();

    std::vector<Vector3f> cv = {{-1,-1,1},{-1,1,1},{1,1,1},{1,-1,1},{-1,-1,-1},{-1,1,-1},{1,1,-1},{1,-1,-1}};
    std::shared_ptr<ConvexCollider> collider = std::make_shared<ConvexPolyhedron>(cv);
    std::shared_ptr<ConvexCollider> collider_sphere = std::make_shared<SphereCollider>(1);
    constexpr float BOX_INVERSE_MASS = 1.0; 

    BodyId b1 = w.AddBody({Vector3f(0,10.0,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(500,0,0),Vector3f(0,0,0), BOX_INVERSE_MASS * Matrix3f::Identity(),BOX_INVERSE_MASS,collider});
    BodyId b2 = w.AddBody({Vector3f(0.0,0.0,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0),1*Matrix3f::Identity(),0,collider});

    w.AddJoint(BallAndSocketJoint(b1,Vector3f(0,-5,0),b2,Vector3f(0,5,0)));

    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);

    while(!glfwWindowShouldClose(window)){
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        w.step(.005f);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        glColor3f(1,1,1);
        glPushMatrix();
            glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
            glTranslatef(0,0,-30);
            glRotatef(40,1,0,0);
            glPushMatrix();
                glBegin(GL_QUADS);
                    for(int i = -100;i<=100;i++){
                        for(int j = -100;j<=100;j++){
                            glVertex3f(i + 1,0, j + 1);
                            glVertex3f(i - 1,0, j + 1);
                            glVertex3f(i - 1,0, j - 1);
                            glVertex3f(i + 1,0, j - 1);
                        }
                    }
                glEnd();
            glPopMatrix();
            glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);

            glPushMatrix();
            {
                Matrix4f mat = w.BodyTransform(b1);
                glMultMatrixf(&mat.a[0][0]);
                drawCube(1);
            }
            glPopMatrix();
            glPushMatrix();
            {
                Matrix4f mat = w.BodyTransform(b2);
                glMultMatrixf(&mat.a[0][0]);
                drawCube(1);
            }
            glPopMatrix();

        glPopMatrix();
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}
