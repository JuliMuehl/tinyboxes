#include <GL/gl.h>
#include <GLFW/glfw3.h>

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

#include <thread>
#include <chrono>

int main(int argc,char** argv){
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
    
    uint64_t b1 = w.AddBody({Vector3f(0,1,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), Matrix3f::Identity(),1.0,collider});
    uint64_t b2 = w.AddBody({Vector3f(0,3.1,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), Matrix3f::Identity(),1.0,collider});
    uint64_t b3 = w.AddBody({Vector3f(0,5.2,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), Matrix3f::Identity(),1.0,collider});
    uint64_t b4 = w.AddBody({Vector3f(0,7.3,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), Matrix3f::Identity(),1.0,collider});

    uint64_t b21 = w.AddBody({Vector3f(-2.1,1,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), Matrix3f::Identity(),1.0,collider});
    uint64_t b22 = w.AddBody({Vector3f(-2.1,3.1,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), Matrix3f::Identity(),1.0,collider});
    uint64_t b23 = w.AddBody({Vector3f(-2.1,5.2,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), Matrix3f::Identity(),1.0,collider});
    uint64_t b24 = w.AddBody({Vector3f(-2.1,7.3,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), Matrix3f::Identity(),1.0,collider});

    uint64_t b31 = w.AddBody({Vector3f(-4.1,1,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), Matrix3f::Identity(),1.0,collider});
    uint64_t b32 = w.AddBody({Vector3f(-4.1,3.1,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), Matrix3f::Identity(),1.0,collider});
    uint64_t b33 = w.AddBody({Vector3f(-4.1,5.2,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), Matrix3f::Identity(),1.0,collider});
    uint64_t b34 = w.AddBody({Vector3f(-4.1,7.3,0),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(0,0,0), Matrix3f::Identity(),1.0,collider});

    uint64_t b5 = w.AddBody({Vector3f(-2.0,1,-32),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(100,0,0), Matrix3f::Identity(),1.0,collider_sphere});
    uint64_t b6 = w.AddBody({Vector3f(-2.0,1,-35),Quaternionf(1,Vector3f(0,0,0)),Vector3f(0,0,0),Vector3f(100,0,0), Matrix3f::Identity(),1.0,collider_sphere});

    w.AddJoint(DistanceJoint(b5,b6,3.0));
    
    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);

    while(!glfwWindowShouldClose(window)){
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        w.step(.01f);
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

            glPushMatrix();
            {
                Matrix4f mat = w.BodyTransform(b3);
                glMultMatrixf(&mat.a[0][0]);
                drawCube(1);
            }
            glPopMatrix();

            glPushMatrix();
            {
                Matrix4f mat = w.BodyTransform(b4);
                glMultMatrixf(&mat.a[0][0]);
                drawCube(1);
            }
            glPopMatrix();

            glPushMatrix();
            {
                Matrix4f mat = w.BodyTransform(b21);
                glMultMatrixf(&mat.a[0][0]);
                drawCube(1);
            }
            glPopMatrix();

            glPushMatrix();
            {
                Matrix4f mat = w.BodyTransform(b22);
                glMultMatrixf(&mat.a[0][0]);
                drawCube(1);
            }
            glPopMatrix();

            glPushMatrix();
            {
                Matrix4f mat = w.BodyTransform(b23);
                glMultMatrixf(&mat.a[0][0]);
                drawCube(1);
            }
            glPopMatrix();

            glPushMatrix();
            {
                Matrix4f mat = w.BodyTransform(b24);
                glMultMatrixf(&mat.a[0][0]);
                drawCube(1);
            }
            glPopMatrix();

            glPushMatrix();
            {
                Matrix4f mat = w.BodyTransform(b31);
                glMultMatrixf(&mat.a[0][0]);
                drawCube(1);
            }
            glPopMatrix();

            glPushMatrix();
            {
                Matrix4f mat = w.BodyTransform(b32);
                glMultMatrixf(&mat.a[0][0]);
                drawCube(1);
            }
            glPopMatrix();

            glPushMatrix();
            {
                Matrix4f mat = w.BodyTransform(b33);
                glMultMatrixf(&mat.a[0][0]);
                drawCube(1);
            }
            glPopMatrix();

            glPushMatrix();
            {
                Matrix4f mat = w.BodyTransform(b34);
                glMultMatrixf(&mat.a[0][0]);
                drawCube(1);
            }
            glPopMatrix();



            glPushMatrix();
            {
                Matrix4f mat = w.BodyTransform(b5);
                glMultMatrixf(&mat.a[0][0]);
                drawSphere(1);
            }
            glPopMatrix();

            glPushMatrix();
            {
                Matrix4f mat = w.BodyTransform(b6);
                glMultMatrixf(&mat.a[0][0]);
                drawSphere(1);
            }
            glPopMatrix();


        glPopMatrix();
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}
