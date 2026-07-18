#ifndef RENDERER_HPP
#define RENDERER_HPP
#include "Math.hpp"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <ostream>
#include <stdexcept>
#include <vector>
#include <tuple>
#include <memory>
#include <cmath>

#define GLSL_VERSION "#version 330\n"
#define SHADER_STRING(x) GLSL_VERSION #x

static inline GLint compileProgram(const char* vertexShaderSource,const char* fragmentShaderSource){
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
        throw std::runtime_error("Error compiling vertex shader:\n" +  std::string(infoLog) + "\n");
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

        throw std::runtime_error("Error compiling fragment shader:\n" +  std::string(infoLog) + "\n");
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

        throw std::runtime_error("Error linking program:\n" +  std::string(infoLog) + "\n");

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
                n1 *= ((int)!std::signbit(Dot(n1,v1)) * 2 - 1) / n1.Norm();
                n2 *= ((int)!std::signbit(Dot(n2,v1)) * 2 - 1) / n2.Norm();
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

class CameraController{
public:
    CameraController(float fov=M_PI/4,float aspect=1.0,float near=0.5,float far=1000.0):fov(fov),aspect(aspect),near(near),far(far){
        UpdateProjectionMatrix();
    }
    const Matrix4f& GetViewMatrix() const {
        return viewMatrix;
    }
    const Matrix4f& GetProjectionMatrix() const {
        return projectionMatrix;
    }
    void SetPosition(const Vector3f& eyePosition){
        this->eyePosition = eyePosition;
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
private:
    Matrix4f viewMatrix = Matrix4f::Identity();
    Matrix4f projectionMatrix;
    Vector3f eyePosition = Vector3f();
    float pitch=0.0,yaw=0.0;
    float fov,aspect,near,far;
};

class Renderer{
public:
    Renderer(){
        constexpr float uvData[] = {
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
        atmosphere_inUVLocation = glGetAttribLocation(atmosphere_program,"inUV");
        atmosphere_uProjectionLocation = glGetUniformLocation(atmosphere_program,"uProjection");
        atmosphere_uViewLocation = glGetUniformLocation(atmosphere_program,"uView");
        atmosphere_uSunDirectionLocation = glGetUniformLocation(atmosphere_program,"uSunDirection");

        floor_program = compileProgram(floor_vertexShaderSource,floor_fragmentShaderSource);
        floor_inUVLocation = glGetAttribLocation(floor_program,"inUV");
        
        floor_uProjectionLocation = glGetUniformLocation(floor_program,"uProjection");
        floor_uViewLocation = glGetUniformLocation(floor_program,"uView");
        floor_uSMViewProjection = glGetUniformLocation(floor_program, "uSMViewProjection");

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
        mesh_uSMViewProjection = glGetUniformLocation(mesh_program, "uSMViewProjection");

        glGenTextures(1, &shadowmap_depthTexture);
        glBindTexture(GL_TEXTURE_2D, shadowmap_depthTexture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, shadowmap_width, shadowmap_height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        // Set border color to maximum depth (prevents sampling artifacts outside bounds)
        float borderColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
        glGenTextures(1, &shadowmap_colorTexture);
        glBindTexture(GL_TEXTURE_2D, shadowmap_colorTexture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, shadowmap_width, shadowmap_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        glGenFramebuffers(1, &shadowmap_fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, shadowmap_fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadowmap_depthTexture, 0);
        
        glBindFramebuffer(GL_FRAMEBUFFER, shadowmap_fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, shadowmap_colorTexture, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    void Render(const Matrix4f& view, 
                const Matrix4f& projection,
                const std::vector<std::shared_ptr<Mesh>>& meshes,
                const std::vector<Material>& materials,
                const std::vector<Matrix4f>& poses,
                const Matrix4f& smViewProjection){
        //Render Atmosphere
        glUseProgram(atmosphere_program);
        glBindBuffer(GL_ARRAY_BUFFER,uvBuffer);
        glEnableVertexAttribArray(atmosphere_inUVLocation);
        glVertexAttribPointer(atmosphere_inUVLocation,2,GL_FLOAT,GL_FALSE,0,nullptr);
        glUniformMatrix4fv(atmosphere_uProjectionLocation,1,GL_TRUE,&projection.a[0][0]);
        glUniformMatrix4fv(atmosphere_uViewLocation,1,GL_TRUE,&view.a[0][0]);
        glUniform3f(atmosphere_uSunDirectionLocation,sunDirection.x,sunDirection.y,sunDirection.z);
        glDrawArrays(GL_TRIANGLES,0,6);
        //Atmosphere is infinitely far away from the camera.
        //We just clear the depth buffer such that everything else is drawn on top.
        glClear(GL_DEPTH_BUFFER_BIT);
      
        
        //Render Ground Plane (Floor)
        glUseProgram(floor_program);
        glBindBuffer(GL_ARRAY_BUFFER,uvBuffer);
        glEnableVertexAttribArray(floor_inUVLocation);
        glVertexAttribPointer(floor_inUVLocation,2,GL_FLOAT,GL_FALSE,0,nullptr);
        glUniformMatrix4fv(floor_uProjectionLocation,1,GL_TRUE, &projection.a[0][0]);
        glUniformMatrix4fv(floor_uViewLocation,1,GL_TRUE, &view.a[0][0]);
        glUniformMatrix4fv(floor_uSMViewProjection, 1, GL_TRUE, &smViewProjection.a[0][0]);
        glDrawArrays(GL_TRIANGLES,0,6);       

        //Render Meshes
        glUseProgram(mesh_program);
        glUniformMatrix4fv(mesh_uSMViewProjection, 1, GL_TRUE, &smViewProjection.a[0][0]);
        for(int i = 0;i<meshes.size();i++){
            const auto& mesh = meshes[i];
            const auto& pose = poses[i];
            const auto& mat = materials[i];
            
            glUniformMatrix4fv(mesh_uViewLocation,1,GL_TRUE,&view.a[0][0]);
            glUniformMatrix4fv(mesh_uProjectionLocation,1,GL_TRUE,&projection.a[0][0]);
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

    void Render(const CameraController& cam,
                const std::vector<std::shared_ptr<Mesh>>& meshes,
                const std::vector<Material>& materials,
                const std::vector<Matrix4f>& poses){
        Matrix3f viewRot = LookAtOrigin(sunDirection, Vector3f(0.0, 1.0, 0.0));
        Matrix4f viewMatrix = Matrix4f(viewRot, Vector3f(0,0,0));
        Matrix4f projectionMatrix = Matrix4f::Identity();
        projectionMatrix.a[0][0] /= 25.0;
        projectionMatrix.a[1][1] /= 25.0;
        projectionMatrix.a[2][2] /= 25.0;
        Matrix4f smPV = projectionMatrix*viewMatrix;
        int viewport[4];
        glGetIntegerv(GL_VIEWPORT, &viewport[0]);
        glViewport(0, 0, shadowmap_width, shadowmap_height);
        glBindFramebuffer(GL_FRAMEBUFFER, shadowmap_fbo);
        Render(viewMatrix, projectionMatrix, meshes, materials, poses, smPV);

        glBindTexture(GL_TEXTURE_2D, shadowmap_depthTexture);
        glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        Render(cam.GetViewMatrix(), cam.GetProjectionMatrix(), meshes, materials, poses, smPV);
    }

private:
    GLuint uvBuffer;
    GLint atmosphere_program;
    GLint atmosphere_inUVLocation;
    GLint atmosphere_uProjectionLocation;
    GLint atmosphere_uViewLocation;
    GLint atmosphere_uSunDirectionLocation;
    
    GLint floor_program;
    GLint floor_inUVLocation;
    GLint floor_uProjectionLocation;
    GLint floor_uViewLocation;
    GLint floor_uSMViewProjection;
    
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
    GLint mesh_uSMViewProjection;

    GLuint shadowmap_depthTexture;
    GLuint shadowmap_colorTexture;
    GLuint shadowmap_fbo;
    constexpr static GLuint shadowmap_width=1024, shadowmap_height=1024;


    Vector3f sunDirection = Vector3f(0.0,1.0,1.5).Normalize();

    static Matrix3f LookAtOrigin(Vector3f from, Vector3f up){
        Vector3f v2 = -from.Normalize();
        Vector3f v0 = Cross(v2, up).Normalize();
        Vector3f v1 = -Cross(v2, v0).Normalize();
        return Matrix3f(v0, v1, v2);
    }

    static constexpr const char* atmosphere_vertexShaderSource = SHADER_STRING(
        in vec2 inUV;
        out vec2 fragUV;
        void main(){
            fragUV = inUV;
            gl_Position = vec4(2.0 * inUV - 1.0,0.0,1.0);
        }
    );

    static constexpr const char* atmosphere_fragmentShaderSource = SHADER_STRING(
        in vec2 fragUV;
        out vec4 fragColor;
        uniform mat4 uView;
        uniform mat4 uProjection;
        uniform vec3 uSunDirection;

        vec3 sun_color(vec3 direction){
            vec3 sun_direction = uSunDirection;
            sun_direction.x *= -1.0;
            float dp = max(dot(sun_direction,direction),0.0);
            float intensity = pow(dp,200);
            return vec3(intensity);
        }

        vec3 sky_color(vec3 direction){
            float h = smoothstep(0.0, 1.4,  1.4-max(direction.y,0.0)) / 1.2;
            return vec3(h*h,h,1.0*h+0.2);
        }

        vec3 raycast_color(vec2 uv){
            vec3 ray = (inverse(uProjection*uView)*vec4(2.0 * uv - 1.0,1.0,1.0)).xyz;
            ray = normalize(ray);
            ray.x *= -1.0;
            return sky_color(ray) + sun_color(ray);
        }

        void main(){
            fragColor = vec4(raycast_color(fragUV),1.0);
        }
    );

    static constexpr const char* floor_vertexShaderSource = SHADER_STRING(
        in vec2 inUV;
        out vec3 fragXYZ;
        uniform mat4 uProjection;
        uniform mat4 uView;
        float width = 50;
        float height = 50;
        void main(){
            vec2 dims = vec2(width,height);
            vec3 position = vec3(0.0);
            position.xz = inUV * dims - dims*0.5;
            fragXYZ = position;
            gl_Position = uProjection * uView * vec4(position,1.0);
        }
    );

    static constexpr const char* floor_fragmentShaderSource = SHADER_STRING(
        in vec3 fragXYZ;
        out vec4 outColor;

        uniform mat4 uSMViewProjection;
        uniform sampler2D SMDepthSampler;

        void main(){
            vec2 xz = round(fragXYZ/2.5).xz;
            if(mod(xz.x,2) == mod(xz.y,2)){
                outColor = vec4(vec3(0.8),1.0);
            }else{
                outColor = vec4(vec3(0.2),1.0);
            }

            vec4 smPosition = uSMViewProjection * vec4(fragXYZ, 1.0);
            vec3 pos = smPosition.xyz / smPosition.w * 0.5 + 0.5;
            vec2 uv = pos.xy;
            vec2 texelSize = 1.0 / textureSize(SMDepthSampler, 0);
            float shadowTotal = 0.0;
            float bias = 0.005;

            for(int x = -2; x <= 2; ++x) {
                for(int y = -2; y <= 2; ++y) {
                    vec2 offset = vec2(x, y) * texelSize;
                    float closestDepth = texture(SMDepthSampler, uv + offset).r; 
                    shadowTotal += (closestDepth + bias < pos.z) ? 0.2 : 1.0;
                }
            }
            outColor.xyz *= (shadowTotal / 25.0);
        }
    );


    static constexpr const char* mesh_vertexShaderSource = SHADER_STRING(
        in vec3 inPosition;
        in vec3 inNormal;
        out vec3 fragNormal;
        out vec3 fragPosition;
        out vec4 smPosition;

        mat4 model = mat4(1.0);
        mat4 cam = mat4(1.0);
        uniform mat4 uView;
        uniform mat4 uProjection;
        uniform mat4 uModel;

        uniform mat4 uSMViewProjection;

        void main(){
            fragNormal = mat3(uModel) * inNormal;
            vec4 modelPosition = uModel*vec4(inPosition,1.0);
            vec3 eyePosition = inverse(uView)[3].xyz;

            fragPosition = eyePosition - modelPosition.xyz;

            smPosition = uSMViewProjection * vec4(modelPosition.xyz, 1.0);
            gl_Position = uProjection*uView*modelPosition;
        }
    );

    static constexpr const char* mesh_fragmentShaderSource = SHADER_STRING(
        in vec3 fragNormal;
        in vec3 fragPosition;
        in vec4 smPosition;
        out vec4 outColor;
        uniform vec3 uAmbient;
        uniform vec3 uDiffuse;
        uniform vec3 uSpecular;
        uniform vec3 uEmission;
        uniform vec3 uSunDirection;

        uniform sampler2D SMDepthSampler;
        float lightIntenisity = 0.8;
        vec3 lightDir = normalize(uSunDirection);
        void main(){
            float specularCoeff = pow(max(-dot(reflect(lightDir,fragNormal),normalize(fragPosition)),0.0),6.0);
            float diffuseCoeff = max(dot(lightDir,fragNormal),0.0);
            vec3 fragColor = uEmission + lightIntenisity * (uAmbient + uDiffuse * diffuseCoeff + uSpecular * specularCoeff);
            
            outColor = vec4(fragColor,0.0);
            vec3 pos = smPosition.xyz * 0.5 + 0.5;
            vec2 uv = pos.xy;
            float bias = max(0.05 * (1.0 - dot(fragNormal, uSunDirection)), 0.005);
            vec2 texelSize = 1.0 / textureSize(SMDepthSampler, 0);
            float shadowTotal = 0.0;
            for(int x = -2; x <= 2; ++x) {
                for(int y = -2; y <= 2; ++y) {
                    vec2 offset = vec2(x, y) * texelSize;
                    float closestDepth = texture(SMDepthSampler, uv + offset).r; 
                    shadowTotal += (closestDepth + bias < pos.z) ? 0.2 : 1.0;
                }
            }
            outColor.xyz *= (shadowTotal / 25.0);
        }
    );
};

#endif
