#ifndef RENDERER_HPP
#define RENDERER_HPP
#include "Math.hpp"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include <memory>
#include <cmath>
#include <chrono>

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
        glGetProgramiv(program,GL_INFO_LOG_LENGTH,&length);
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
    Vector3f emission;
    Material(){}
    Material(Vector3f ambient,Vector3f diffuse,Vector3f specular,Vector3f emission):ambient(ambient),diffuse(diffuse),specular(specular),emission(emission){}
};

struct Mesh{
    GLuint vertexBuffer;
    GLuint normalBuffer;
    GLuint faceBuffer;
    const size_t numElements;
    
    Mesh(const std::vector<Vector3f>&vertices,
         const std::vector<Vector3f>& normals,
         const std::vector<std::tuple<GLuint,GLuint,GLuint>>& faces):numElements(faces.size() * 3){
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

    static std::shared_ptr<Mesh> CreateCube(float a,float b,float c){
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
        return std::make_shared<Mesh>(vertices,normals,faces);
    }

    static std::shared_ptr<Mesh> CreateSphere(float r,int gridSize=64){
        std::vector<Vector3f> vertices;
        std::vector<Vector3f> normals;
        std::vector<std::tuple<GLuint,GLuint,GLuint>> faces;
        auto parameterization = [r](float u,float v){
            float cu = cosf(u*M_PI),cv=cosf(v*M_PI * 2.0),su=sinf(u*M_PI),sv=sinf(v*M_PI * 2.0);
            return r*Vector3f(su*cv,su*sv,cu);
        };
        int faceCount = 0;
        for(int i = 0;i<gridSize;i++){
            for(int j = 0;j<gridSize;j++){
                float u = 1.0*1.0*i/gridSize,v = 1.0*1.0*j/gridSize;
                float u_ = u + 1.0/gridSize,v_ = v + 1.0/gridSize;
                std::cout << "(" << u << " " << v << ")" << "\t" << "(" << u_ << " " << v_ << ")" << std::endl; 
                Vector3f v1 = parameterization(u,v),v2 = parameterization(u_,v),v3=parameterization(u_,v_),v4=parameterization(u,v_);
                Vector3f n1 = Cross(v1-v2,v1-v3), n2 = Cross(v1-v4,v1-v3);
                n1 *= ((int)!std::signbit(Dot(n1,v1)) * 2 - 1) / n1.Norm();n2 *= ((int)!std::signbit(Dot(n2,v1)) * 2 - 1) / n2.Norm();
                vertices.push_back(v1);vertices.push_back(v2);vertices.push_back(v3);
                normals.push_back(n1);normals.push_back(n1);normals.push_back(n1);
                faces.push_back({3*(2*faceCount + 0) + 0,3*(2*faceCount + 0) + 1,3*(2*faceCount + 0) + 2});
                vertices.push_back(v1);vertices.push_back(v4);vertices.push_back(v3);
                normals.push_back(n2);normals.push_back(n2);normals.push_back(n2);
                faces.push_back({3*(2*faceCount + 1) + 0,3*(2*faceCount + 1) + 1,3*(2*faceCount + 1) + 2});
                faceCount++;
            }
        }
        std::cout << vertices.size() << " " << normals.size() << std::endl;
        return std::make_shared<Mesh>(vertices,normals,faces);
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
    float pitch=0.0,yaw=0.0;
    float fov,aspect,near,far;
    CameraController(float fov=M_PI/4,float aspect=1.0,float near=0.5,float far=1000.0):fov(fov),aspect(aspect),near(near),far(far){
        UpdateProjectionMatrix();
    }
    void UpdateProjectionMatrix(){
        projectionMatrix.a[0][0] = atanf(fov/2.0) / aspect;
        projectionMatrix.a[1][1] = atanf(fov/2.0);
        projectionMatrix.a[2][2] = -((far+near)/(far-near));
        projectionMatrix.a[2][3] = -((2*(near*far))/(far-near));
        projectionMatrix.a[3][2] = -1;
    }
    void ResizeCallback(double width,double height){
        aspect = width/height;
        UpdateProjectionMatrix();
    }
    void CursorCallback(double xpos,double ypos){
        pitch = M_PI* ypos;
        //pitch = fmod(pitch,M_PI)
        if(pitch < -M_PI/2) pitch = -M_PI/2;
        if(pitch > M_PI/2) pitch = M_PI/2;
        yaw = fmod(2*M_PI*xpos,2*M_PI);

        Matrix3f xrotmat = Matrix3f::Identity();
        Matrix3f yrotmat = Matrix3f::Identity();

        xrotmat.a[1][1] = cosf(pitch);
        xrotmat.a[1][2] = -sinf(pitch);
        xrotmat.a[2][1] = sinf(pitch);
        xrotmat.a[2][2] = cosf(pitch);
        
        yrotmat.a[0][0] = cosf(yaw);
        yrotmat.a[0][2] = sinf(yaw);
        yrotmat.a[2][0] = -sinf(yaw);
        yrotmat.a[2][2] = cosf(yaw);
        
        Matrix3f rotMat = xrotmat * yrotmat;
        for(int i = 0;i<3;i++)for(int j = 0;j<3;j++) viewMatrix.a[i][j] = rotMat.a[i][j];
    }
    void KeyCallback(GLFWwindow* window){
        Vector3f forward = Vector3f(viewMatrix.a[2][0],viewMatrix.a[2][1],viewMatrix.a[2][2]);
        
        Vector3f right  = Vector3f(viewMatrix.a[0][0],viewMatrix.a[0][1],viewMatrix.a[0][2]);
        float speed = 0.25;
        if(glfwGetKey(window,GLFW_KEY_W) == GLFW_PRESS){
            eyePosition +=  +speed * forward;
        }
        if(glfwGetKey(window,GLFW_KEY_S) == GLFW_PRESS){
            eyePosition += -speed * forward;
        }
        if(glfwGetKey(window,GLFW_KEY_A) == GLFW_PRESS){
            eyePosition += +speed * right;
        }
        if(glfwGetKey(window,GLFW_KEY_D) == GLFW_PRESS){
            eyePosition += -speed * right;
        }
        
        Vector4f lastCol = viewMatrix.Transform(Vector4f(eyePosition.x,eyePosition.y,eyePosition.z,0.0));
        viewMatrix.a[0][3] = lastCol.x;
        viewMatrix.a[1][3] = lastCol.y;
        viewMatrix.a[2][3] = lastCol.z;
        viewMatrix.a[3][3] = 1.0;
    }
};



struct Renderer{
    GLuint uvBuffer;
    GLint atmosphere_program;
    GLint atmosphere_inUvLocation;
    GLint atmosphere_uProjectionLocation;
    GLint atmosphere_uViewLocation;
    GLint atmosphere_uSunDirectionLocation;
    
    GLint floor_program;
    GLint floor_inUvLocation;
    GLint floor_uProjectionLocation;
    GLint floor_uViewLocation;
    
    GLint mesh_program;
    GLint mesh_inPositionLocation;
    GLint mesh_inNormalLocation;
    GLint mesh_uModelLocation;
    GLint mesh_uViewLocation;
    GLint mesh_uProjectionLocation;
    GLint mesh_uAmbientLocation;
    GLint mesh_uDiffuseLocation;
    GLint mesh_uSpecularLocation;
    GLint mesh_uEmissionLocation;
    GLint mesh_uSunDirectionLocation;

    Vector3f sunDirection = Vector3f(0.0,1.0,3.0).Normalize();

    static constexpr const char* atmosphere_vertexShaderSource= SHADER_STRING(
        in vec2 inUv;
        out vec2 fragUv;
        void main(){
            fragUv = inUv;
            gl_Position = vec4(2.0 * inUv - 1.0,0.0,1.0);
        }
    );


    static constexpr const char* atmosphere_fragmentShaderSource = SHADER_STRING(
        in vec2 fragUv;
        out vec4 fragColor;
        uniform mat4 uView;
        uniform mat4 uProjection;
        uniform vec3 uSunDirection;

        vec3 sun_color(vec3 direction){
            vec3 sun_direction = uSunDirection;
            sun_direction.x *= -1.0;
            float dp = max(dot(sun_direction,direction),0.0);
            float intensity = pow(dp,100);
            return vec3(intensity);
        }

        vec3 sky_color(vec3 direction){
            float h = 1.0-max(direction.y,0.0);
            return vec3(h*h,h,1.0*h+0.2);
        }

        vec3 raycast_color(vec2 uv){
            vec3 ray = (inverse(uProjection*uView)*vec4(2.0 * uv - 1.0,1.0,1.0)).xyz;
            ray = normalize(ray);
            ray.x *= -1.0;
            return sky_color(ray) + sun_color(ray);
        }

        void main(){
            fragColor = vec4(raycast_color(fragUv),1.0);
        }
    );

    static constexpr const char* floor_vertexShaderSource = SHADER_STRING(
        in vec2 inUv;
        out vec2 fragXZ;
        uniform mat4 uProjection;
        uniform mat4 uView;
        float width = 1000;
        float height = 1000;
        void main(){
            vec2 dims = vec2(width,height);
            vec3 position = vec3(0.0);
            position.xz = inUv * dims - dims*0.5;
            fragXZ = position.xz;
            gl_Position = uProjection * uView * vec4(position,1.0);
        }
    );

    static constexpr const char* floor_fragmentShaderSource = SHADER_STRING(
        in vec2 fragXZ;
        out vec4 outCol;

        void main(){
            vec2 xz = round(fragXZ);
            if(mod(xz.x,2) == mod(xz.y,2)){
                outCol = vec4(vec3(0.8),1.0);
            }else{
                outCol = vec4(vec3(0.2),1.0);
            }
        }
    );


    static constexpr const char* mesh_vertexShaderSource = SHADER_STRING(
            in vec3 inPosition;
            in vec3 inNormal;
            out vec3 fragNormal;
            out vec3 fragPosition;

            mat4 model = mat4(1.0);
            mat4 cam = mat4(1.0);
            uniform mat4 uView;
            uniform mat4 uProjection;
            uniform mat4 uModel;

            void main(){
                fragNormal = mat3(uModel) * inNormal;
                vec4 modelPosition = uModel*vec4(inPosition,1.0);
                vec3 eyePosition = inverse(uView)[3].xyz;

                fragPosition = eyePosition - modelPosition.xyz;
                gl_Position = uProjection*uView*modelPosition;
            }
    );

    static constexpr const char* mesh_fragmentShaderSource = SHADER_STRING(
            in vec3 fragNormal;
            in vec3 fragPosition;
            out vec4 outColor;
            uniform vec3 uAmbient;
            uniform vec3 uDiffuse;
            uniform vec3 uSpecular;
            uniform vec3 uEmission;
            uniform vec3 uSunDirection;
            float lightIntenisity = 0.8;
            vec3 lightDir = normalize(uSunDirection);
            void main(){
                float specularCoeff = pow(max(-dot(reflect(lightDir,fragNormal),normalize(fragPosition)),0.0),6.0);
                float diffuseCoeff = max(dot(lightDir,fragNormal),0.0);
                vec3 fragColor = uEmission + lightIntenisity * (uAmbient + uDiffuse * diffuseCoeff + uSpecular * specularCoeff);
                outColor = vec4(fragColor,0.0);
            }
    );

    Renderer(){
        float uvData[] = {
            0.0,0.0,
            0.0,1.0,
            1.0,1.0,
            0.0,0.0,
            1.0,0.0,
            1.0,1.0
        };
        glGenBuffers(1,&uvBuffer);
        glBindBuffer(GL_ARRAY_BUFFER,uvBuffer);
        glBufferData(GL_ARRAY_BUFFER,sizeof(uvData),&uvData[0],GL_STATIC_DRAW);

        atmosphere_program = compileProgram(atmosphere_vertexShaderSource,atmosphere_fragmentShaderSource);
        atmosphere_inUvLocation = glGetAttribLocation(atmosphere_program,"inUv");
        atmosphere_uProjectionLocation = glGetUniformLocation(atmosphere_program,"uProjection");
        atmosphere_uViewLocation = glGetUniformLocation(atmosphere_program,"uView");
        atmosphere_uSunDirectionLocation = glGetUniformLocation(atmosphere_program,"uSunDirection");

        floor_program = compileProgram(floor_vertexShaderSource,floor_fragmentShaderSource);
        floor_inUvLocation = glGetAttribLocation(floor_program,"inUv");
        
        floor_uProjectionLocation = glGetUniformLocation(floor_program,"uProjection");
        floor_uViewLocation = glGetUniformLocation(floor_program,"uView");

        mesh_program = compileProgram(mesh_vertexShaderSource,mesh_fragmentShaderSource);
        mesh_inPositionLocation = glGetAttribLocation(mesh_program,"inPosition");
        mesh_inNormalLocation = glGetAttribLocation(mesh_program,"inNormal");
        mesh_uModelLocation = glGetUniformLocation(mesh_program,"uModel");
        mesh_uViewLocation = glGetUniformLocation(mesh_program,"uView");
        mesh_uProjectionLocation = glGetUniformLocation(mesh_program,"uProjection");
        mesh_uAmbientLocation = glGetUniformLocation(mesh_program,"uAmbient");
        mesh_uDiffuseLocation = glGetUniformLocation(mesh_program,"uDiffuse");
        mesh_uSpecularLocation = glGetUniformLocation(mesh_program,"uSpecular");
        mesh_uEmissionLocation = glGetUniformLocation(mesh_program,"uEmission");
        mesh_uSunDirectionLocation = glGetUniformLocation(mesh_program,"uSunDirection");
        
    }

    void Render(const CameraController& cam,const std::vector<std::shared_ptr<Mesh>>& meshes,const std::vector<Material>& materials,const std::vector<Matrix4f>& poses){
      
        //Render Atmosphere
        glUseProgram(atmosphere_program);
        glBindBuffer(GL_ARRAY_BUFFER,uvBuffer);
        glEnableVertexAttribArray(atmosphere_inUvLocation);
        glVertexAttribPointer(atmosphere_inUvLocation,2,GL_FLOAT,GL_FALSE,0,nullptr);
        glUniformMatrix4fv(atmosphere_uProjectionLocation,1,GL_TRUE,&cam.projectionMatrix.a[0][0]);
        glUniformMatrix4fv(atmosphere_uViewLocation,1,GL_TRUE,&cam.viewMatrix.a[0][0]);
        glUniform3f(atmosphere_uSunDirectionLocation,sunDirection.x,sunDirection.y,sunDirection.z);
        glDrawArrays(GL_TRIANGLES,0,6);
        //Atmosphere is infinitely far away from the camera.
        //We just clear the depth buffer such that everything else is drawn on top.
        glClear(GL_DEPTH_BUFFER_BIT);
      

        //Render Ground Plane (Floor)
        glUseProgram(floor_program);
        glBindBuffer(GL_ARRAY_BUFFER,uvBuffer);
        glEnableVertexAttribArray(floor_inUvLocation);
        glVertexAttribPointer(floor_inUvLocation,2,GL_FLOAT,GL_FALSE,0,nullptr);
        glUniformMatrix4fv(floor_uProjectionLocation,1,GL_TRUE,&cam.projectionMatrix.a[0][0]);
        glUniformMatrix4fv(floor_uViewLocation,1,GL_TRUE,&cam.viewMatrix.a[0][0]);
        glDrawArrays(GL_TRIANGLES,0,6);       

        //Render Meshes
        glUseProgram(mesh_program);
        
        for(int i = 0;i<meshes.size();i++){
            const auto& mesh = meshes[i];
            const auto& pose = poses[i];
            const auto& mat = materials[i];
            
            glUniformMatrix4fv(mesh_uViewLocation,1,GL_TRUE,&cam.viewMatrix.a[0][0]);
            glUniformMatrix4fv(mesh_uProjectionLocation,1,GL_TRUE,&cam.projectionMatrix.a[0][0]);
            glUniformMatrix4fv(mesh_uModelLocation,1,GL_TRUE,&pose.a[0][0]);
            glUniform3f(mesh_uAmbientLocation,mat.ambient.x,mat.ambient.y,mat.ambient.z);
            glUniform3f(mesh_uDiffuseLocation,mat.diffuse.x,mat.diffuse.y,mat.diffuse.z);
            glUniform3f(mesh_uSpecularLocation,mat.specular.x,mat.specular.y,mat.specular.z);
            glUniform3f(mesh_uEmissionLocation,mat.emission.x,mat.emission.y,mat.emission.z);
            glUniform3f(mesh_uSunDirectionLocation,sunDirection.x,sunDirection.y,sunDirection.z);

            glBindBuffer(GL_ARRAY_BUFFER,mesh->vertexBuffer);
            glEnableVertexAttribArray(mesh_inPositionLocation);
            glVertexAttribPointer(mesh_inPositionLocation,3,GL_FLOAT,GL_FALSE,0,nullptr);

            glBindBuffer(GL_ARRAY_BUFFER,mesh->normalBuffer);
            glEnableVertexAttribArray(mesh_inNormalLocation);
            glVertexAttribPointer(mesh_inNormalLocation,3,GL_FLOAT,GL_FALSE,0,nullptr);

            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,mesh->faceBuffer);
            glDrawElements(GL_TRIANGLES,mesh->numElements,GL_UNSIGNED_INT,nullptr);
        }

    }
};
/*
int main(){
    if(!glfwInit()){
        std::cout << "Error initializing GLFW" << std::endl;
    }
    auto* window = glfwCreateWindow(640,480,"Tinyboxes Visualization",nullptr,nullptr);
    glfwMakeContextCurrent(window);
    if(glewInit()){
        std::cout << "Error initializing GLEW" << std::endl;
    }

    std::shared_ptr<Mesh> mesh = Mesh::CreateSphere(1.0);
    std::vector<std::shared_ptr<Mesh>> meshes = {mesh};
    std::vector<Matrix4f> poses = {Matrix4f::Identity()};
    std::vector<Material> materials = {Material(Vector3f(0.0f,0.0f,0.0f),Vector3f(0.5f,0.0f,0.0f),Vector3f(0.7f,0.6f,0.6f ),Vector3f(0.0,0.0,0.0))};
    poses[0].a[1][3] = 1.0;
    auto renderer = Renderer();
    
    glClearColor(0.0,0.0,0.0,1.0);
    glEnable(GL_DEPTH_TEST);

    CameraController cam = CameraController(45.0,640.0/480.0,0.5,1000.0);

    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    bool processInput = true;
    while(!glfwWindowShouldClose(window)){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
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
    }
}
*/
#endif
