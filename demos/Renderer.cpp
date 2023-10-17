#include "Math.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include <memory>
#include <cmath>

#define GLSL_VERSION "#version 330\n"
#define SHADER_STRING(x) GLSL_VERSION #x

GLint compileProgram(const char* vertexShaderSource,const char* fragmentShaderSource){
    GLint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader,1,&vertexShaderSource,nullptr);
    glCompileShader(vertexShader);
    GLint status;
    glGetShaderiv(vertexShader,GL_COMPILE_STATUS,&status);
    if(status != GL_TRUE){
        int length;
        glGetShaderiv(vertexShader,GL_INFO_LOG_LENGTH,&length);
        char* infoLog = new char[length];
        glGetShaderInfoLog(vertexShader,length,nullptr,infoLog);
        std::cout << "Error compiling vertex shader:" << std::endl 
            << infoLog << std::endl;
        delete[] infoLog;
    }
    GLint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader,1,&fragmentShaderSource,nullptr);
    glCompileShader(fragmentShader);
    glGetShaderiv(fragmentShader,GL_COMPILE_STATUS,&status);
    if(status != GL_TRUE){
        int length;
        glGetShaderiv(fragmentShader,GL_INFO_LOG_LENGTH,&length);
        char* infoLog = new char[length];
        glGetShaderInfoLog(fragmentShader,length,nullptr,infoLog);
        std::cout << "Error compiling fragment shader:" << std::endl 
            << infoLog << std::endl;
        delete[] infoLog;
    }
    GLint program = glCreateProgram();
    glAttachShader(program,vertexShader);
    glAttachShader(program,fragmentShader);
    glLinkProgram(program);
    glGetProgramiv(program,GL_LINK_STATUS,&status);
    if(status != GL_TRUE){
        int length;
        glGetProgramiv(fragmentShader,GL_INFO_LOG_LENGTH,&length);
        char* infoLog = new char[length];
        glGetProgramInfoLog(program,length,nullptr,infoLog);
        std::cout << "Error linking program:" << std::endl 
            << infoLog << std::endl;
        delete[] infoLog;
    }
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    return program;
}

struct Material{
    Vector3f ambient;
    Vector3f diffuse;
    Vector3f specular;
    Vector3f emmision;
};

struct Mesh{
    Material mat;
    GLuint vertexBuffer;
    GLuint normalBuffer;
    GLuint faceBuffer;
    const size_t numElements;
    
    Mesh(const std::vector<Vector3f>&vertices,
         const std::vector<Vector3f>& normals,
         const std::vector<std::tuple<GLuint,GLuint,GLuint>>& faces,
         const Material& mat):mat(mat),numElements(faces.size() * 3){
        glGenBuffers(1,&vertexBuffer);
        glBindBuffer(GL_ARRAY_BUFFER,vertexBuffer);
        glBufferData(GL_ARRAY_BUFFER,sizeof(vertices[0]) * vertices.size(),vertices.data(),GL_STATIC_DRAW);
        glGenBuffers(1,&normalBuffer);
        glBindBuffer(GL_ARRAY_BUFFER,normalBuffer);
        glBufferData(GL_ARRAY_BUFFER,sizeof(normals[0]) * normals.size(),normals.data(),GL_STATIC_DRAW);
        glGenBuffers(1,&faceBuffer);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,faceBuffer);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,sizeof(faces[0]) * faces.size(),faces.data(),GL_STATIC_DRAW);
    }

    static std::shared_ptr<Mesh> CreateCube(float a,float b,float c,const Material& mat){
        std::vector<Vector3f> vertices = {
            {-a,-b,-c},
            {-a, b,-c},
            {-a, b, c},
            {-a,-b, c},

            { a,-b,-c},
            { a, b,-c},
            { a, b, c},
            { a,-b, c},

            {-a,-b,-c},
            { a,-b,-c},
            { a,-b, c},
            {-a,-b, c},

            {-a, b,-c},
            { a, b,-c},
            { a, b, c},
            {-a, b, c},

            {-a,-b,-c},
            { a,-b,-c},
            { a, b,-c},
            {-a, b,-c},

            {-a,-b, c},
            { a,-b, c},
            { a, b, c},
            {-a, b, c},
        };

       std::vector<Vector3f> normals = {
            {-1, 0, 0},
            {-1, 0, 0},
            {-1, 0, 0},
            {-1, 0, 0},
            { 1, 0, 0},
            { 1, 0, 0},
            { 1, 0, 0},
            { 1, 0, 0},

            { 0,-1, 0},
            { 0,-1, 0},
            { 0,-1, 0},
            { 0,-1, 0},
            { 0, 1, 0},
            { 0, 1, 0},
            { 0, 1, 0},
            { 0, 1, 0},

            { 0, 0,-1},
            { 0, 0,-1},
            { 0, 0,-1},
            { 0, 0,-1},
            { 0, 0, 1},
            { 0, 0, 1},
            { 0, 0, 1},
            { 0, 0, 1},
        };
        std::vector<std::tuple<GLuint,GLuint,GLuint>> faces = {
            {0,1,2},
            {0,3,2},
            {4,5,6},
            {4,7,6},
            {8,9,10},
            {8,11,10},
            {12,13,14},
            {12,15,14},
            {16,17,18},
            {16,19,18},
            {20,21,22},
            {20,23,22}
        };
        return std::make_shared<Mesh>(vertices,normals,faces,mat);
    }

    ~Mesh(){
        glDeleteBuffers(1,&vertexBuffer);
        glDeleteBuffers(1,&normalBuffer);
        glDeleteBuffers(1,&faceBuffer);
    }
};

struct CameraController{
    Matrix4f GetViewMatrix(){
        return viewMatrix;
    }
    Matrix4f viewMatrix = Matrix4f::Identity();
    Matrix4f projectionMatrix;
    Vector3f eyePosition = Vector3f();
    CameraController(float fov=45.0,float near=0.5,float far=1000.0){
        projectionMatrix.a[0][0] = atanf(fov/2.0);
        projectionMatrix.a[1][1] = atan(fov/2.0);
        projectionMatrix.a[2][2] = -((far+near)/(far-near));
        projectionMatrix.a[2][3] = -((2*(near*far))/(far-near));
        projectionMatrix.a[3][2] = -1;
    }
    void CursorCallback(double xpos,double ypos){
        Matrix3f xrotmat = Matrix4f::Identity();
        Matrix3f yrotmat = Matrix4f::Identity();
        float pitch = -0.01 * ypos;
        xrotmat.a[1][1] = cosf(pitch);
        xrotmat.a[1][2] = sinf(pitch);
        xrotmat.a[2][1] = -sinf(pitch);
        xrotmat.a[2][2] = cosf(pitch);
        float yaw = -0.01 * xpos;
        yrotmat.a[0][0] = cosf(yaw);
        yrotmat.a[0][2] = sinf(yaw);
        yrotmat.a[2][0] = -sinf(yaw);
        yrotmat.a[2][2] = cosf(yaw);
        Matrix3f newRot = xrotmat * yrotmat;
        for(int i = 0;i<3;i++){
            for(int j = 0;j<3;j++){
                viewMatrix.a[i][j] = newRot.a[i][j];
            }
        }
        Vector3f lastCol = Matrix3f(viewMatrix) * eyePosition;
        viewMatrix.a[0][3] = lastCol.x;
        viewMatrix.a[1][3] = lastCol.y;
        viewMatrix.a[2][3] = lastCol.z;
        viewMatrix.a[3][3] = 1.0;
    }
    void KeyCallback(GLFWwindow* window){
        Vector3f forward = Vector3f(viewMatrix.a[0][2],viewMatrix.a[1][2],viewMatrix.a[2][2]);
        
        Vector3f right  = Vector3f(viewMatrix.a[0][0],viewMatrix.a[1][0],viewMatrix.a[2][0]);
        float speed = 0.05;
        if(glfwGetKey(window,GLFW_KEY_W) == GLFW_PRESS){
            eyePosition +=  speed * forward;
        }
        if(glfwGetKey(window,GLFW_KEY_S) == GLFW_PRESS){
            eyePosition += -speed * forward;
        }
        if(glfwGetKey(window,GLFW_KEY_D) == GLFW_PRESS){
            eyePosition += -speed * right;
        }
        if(glfwGetKey(window,GLFW_KEY_A) == GLFW_PRESS){
            eyePosition +=  speed * right;
        }
        Vector3f lastCol = Matrix3f(viewMatrix) * eyePosition;
        viewMatrix.a[0][3] = lastCol.x;
        viewMatrix.a[1][3] = lastCol.y;
        viewMatrix.a[2][3] = lastCol.z;
        viewMatrix.a[3][3] = 1.0;
    }
};

/*

const char* vertexShaderSource = SHADER_STRING(
    in vec2 inUv;
    out vec4 fragCol;
    void main(){
        fragCol = vec4(inUv,0.0,1.0);
        gl_Position = vec4(inUv * 2.0 - 1.0,0.0,1.0);
    }
);

const char* fragmentShaderSource = SHADER_STRING(
    in vec4 fragCol;
    out vec4 outCol;
    void main(){
        outCol = fragCol;
    }
);

*/

const char* vertexShaderSource = SHADER_STRING(
        in vec3 inPosition;
        in vec3 inNormal;
        out vec3 fragColor;

        vec3 lightDir = normalize(vec3(0.0,1.0,1.0));
        mat4 model = mat4(1.0);
        mat4 cam = mat4(1.0);
        uniform mat4 uView;
        uniform mat4 uProjection;

        uniform float time;

        void main(){
            float c = cos(3.141569 * time);
            float s = sin(3.141569 * time);
            model[1][1] = c;
            model[1][2] = -s;
            model[2][1] = s;
            model[2][2] = c;
            vec4 hpos = model * vec4(inPosition,1.0);
            vec3 normal= mat3(model) * inNormal;
            fragColor = (0.3 + max(dot(lightDir,normal),0.0)) * vec3(1.0);
            gl_Position = uProjection*uView*hpos;
        }
);

const char* fragmentShaderSource = SHADER_STRING(
        in vec3 fragColor;
        out vec4 outColor;
        void main(){
            outColor = vec4(fragColor,1.0);
        }
);

int main(){
    if(!glfwInit()){
        std::cout << "Error initializing GLFW" << std::endl;
    }
    auto* window = glfwCreateWindow(640,480,"Tinyboxes Visualization",nullptr,nullptr);
    glViewport(0,0,640,480);
    glfwMakeContextCurrent(window);
    if(glewInit()){
        std::cout << "Error initializing GLEW" << std::endl;
    }

    GLint program = compileProgram(vertexShaderSource,fragmentShaderSource);
    glUseProgram(program);

    /*
    float uvData[] = {
        0.0,0.0,
        0.0,1.0,
        1.0,1.0,
        0.0,0.0,
        1.0,0.0,
        1.0,1.0
    };
    GLuint uvBuffer; 
    glGenBuffers(1,&uvBuffer);
    glBindBuffer(GL_ARRAY_BUFFER,uvBuffer);
    glBufferData(GL_ARRAY_BUFFER,sizeof(uvData),&uvData[0],GL_STATIC_DRAW);

    GLint inUvLocation = glGetAttribLocation(program,"inUv");
    glEnableVertexAttribArray(inUvLocation);
    glVertexAttribPointer(inUvLocation,2,GL_FLOAT,GL_FALSE,0,nullptr);
    */


    std::shared_ptr<Mesh> mesh = Mesh::CreateCube(1.0,1.0,1.0,Material());
    GLint inPositionLocation = glGetAttribLocation(program,"inPosition");
    GLint inNormalLocation = glGetAttribLocation(program,"inNormal");
    GLint uViewLocation = glGetUniformLocation(program,"uView");
    GLint uProjectionLocation = glGetUniformLocation(program,"uProjection");

    std::cout << "inPositionLocation=" << inPositionLocation << std::endl;
    std::cout << "inNormalLocation=" << inNormalLocation << std::endl;
    std::cout << "uViewLocation=" << uViewLocation << std::endl;

    glBindBuffer(GL_ARRAY_BUFFER,mesh->vertexBuffer);
    glEnableVertexAttribArray(inPositionLocation);
    glVertexAttribPointer(inPositionLocation,3,GL_FLOAT,GL_FALSE,0,nullptr);

    glBindBuffer(GL_ARRAY_BUFFER,mesh->normalBuffer);
    glEnableVertexAttribArray(inNormalLocation);
    glVertexAttribPointer(inNormalLocation,3,GL_FLOAT,GL_FALSE,0,nullptr);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,mesh->faceBuffer);
    glClearColor(0.0,0.0,0.0,1.0);

    glEnable(GL_DEPTH_TEST);

    CameraController cam = CameraController();

    float t = 0.0;
    GLint timeLocation = glGetUniformLocation(program,"time");
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    bool processInput = true;
    while(!glfwWindowShouldClose(window)){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glUniform1f(timeLocation,t+=0.000);
        glUniformMatrix4fv(uViewLocation,1,GL_TRUE,&cam.viewMatrix.a[0][0]);
        glUniformMatrix4fv(uProjectionLocation,1,GL_TRUE,&cam.projectionMatrix.a[0][0]);
        glDrawElements(GL_TRIANGLES,mesh->numElements,GL_UNSIGNED_INT,nullptr);
        double xpos,ypos;
        glfwGetCursorPos(window, &xpos, &ypos);
        if(processInput){
            cam.CursorCallback(xpos,ypos);
            cam.KeyCallback(window);
            if(glfwGetKey(window,GLFW_KEY_ESCAPE) == GLFW_PRESS){
                processInput = false;
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            }
        }
        if(!processInput && glfwGetMouseButton(window,GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS){
            processInput = true;
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        }
        //glDrawArrays(GL_TRIANGLES,0,6);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}

