#ifndef TINYBOXES_GUARD
#define TINYBOXES_GUARD
#include <vector>
#include <memory>
#include <map>
#include <stdint.h>
#include <list>

#include "Math.hpp"
#include "Collision.hpp"

struct RigidBody{
    Vector3f x;
    Quaternionf theta;
    Vector3f v;
    Vector3f omega;
    Matrix3f inverseInertia;
    float inverseMass;
    std::shared_ptr<ConvexCollider> collider;
    inline Vector3f GetVelocityAtPoint(Vector3f point) const{
        Vector3f r = x - point;
        return Cross(omega,r) + v;
    }
    inline Vector3f PointToLocal(Vector3f p) const {
        return theta.Conjugate().Rotate(p-x);
    }
    inline Vector3f PointToGlobal(Vector3f r) const {
        return theta.Rotate(r) + x;
    }
    inline Matrix3f GetTransformedInverseInertia() const{
        return Matrix3f(theta) * inverseInertia * Matrix3f(theta.Conjugate());
    }
};

class PlaneConstraint{
public:
    inline PlaneConstraint(Vector3f normal,float d): 
    PlaneConstraint(normal.Normalize(), ComputeTangentFrame(normal), d){}
    
    void FindViolations(const std::vector<RigidBody>& bodies);
    inline bool ApplyImpulses(std::vector<RigidBody>& bodies,float dt){
        bool converged = true;
        for(auto& entry:contacts){
            auto& id = entry.first;
            auto& contact = entry.second;
            converged = converged & ApplyImpulse(bodies[id],contact,dt);
        }
        return converged;
    }
private:
    inline PlaneConstraint(const Vector3f& normal, const std::pair<Vector3f, Vector3f>& uv, float d):
    normal(normal), u1(uv.first), u2(uv.second), d(d){}

    static constexpr float BETA = 0.01;
    static constexpr float MU = 0.50;
    static constexpr float TOLLERANCE = 1e-5;
    static constexpr float CONTACT_POINT_TOLLERANCE = 5e-3;
    static constexpr float CONTACT_CONSTRAINT_THRESHOLD = 1e-6;
    static constexpr float CONVERGENCE_TRESHOLD = 10.0 / 100.0;
    const Vector3f normal,u1,u2;
    float d;
    std::vector<std::pair<size_t,Contact>> contacts;
    bool ApplyImpulse(RigidBody& body,Contact& contact,float dt);
};

class CollisionConstraint{
public:
    void FindViolations(const std::vector<RigidBody>& bodies);
    inline bool ApplyImpulses(std::vector<RigidBody>& bodies,float dt){
        bool converged = true;
        for(auto& entry:violations){
            size_t i = entry.first.first,j = entry.first.second;
            
            auto& contacts = entry.second;
            for(auto& contact:contacts){
                converged = converged & ApplyImpulse(bodies[i],bodies[j],contact,dt);
            }
        }
        return converged;
    }
private:
    static constexpr float BETA = 0.01;
    static constexpr float MU = 0.50;
    static constexpr float TOLLERANCE = 1e-5;
    static constexpr float CONTACT_POINT_TOLLERANCE = 5e-3;
    static constexpr float CONTACT_CONSTRAINT_THRESHOLD = 1e-6;
    static constexpr float CONVERGENCE_TRESHOLD = 10.0 / 100.0;
    std::map<std::pair<size_t,size_t>,std::list<Contact>> violations;
    std::map<std::pair<size_t,size_t>,int> last_contact;
    bool ApplyImpulse(RigidBody& b1,RigidBody& b2,Contact& contact,float dt);
};

class Joint:std::enable_shared_from_this<Joint>{
    public:
    virtual bool ApplyImpulses(std::vector<RigidBody>& bodies,float dt) = 0;
};

class World{
public:
    using BodyId = uint64_t;
    World(Vector3f g = Vector3f(0,-9.81,0)):g(g){}
    inline BodyId AddBody(RigidBody body){
        bodies.push_back(body);
        return bodies.size() - 1;
    }
    inline Matrix4f BodyTransform(BodyId bodyId) const {
        const RigidBody& body = bodies[bodyId];
        return Matrix4f(Matrix3f(body.theta),body.x);
    }
    inline Vector3f GetPosition(BodyId bodyId) const{
        return bodies[bodyId].x;
    }
    inline Quaternionf GetOrientation(BodyId bodyId) const{
        return bodies[bodyId].theta;
    }
    inline Vector3f GetVelocity(BodyId bodyId) const{
        return bodies[bodyId].v;
    }
    inline Vector3f GetAngularVelocity(BodyId bodyId) const{
        return bodies[bodyId].omega;
    }
    inline float GetInverseMass(BodyId bodyId) const{
        return bodies[bodyId].inverseMass;
    }
    inline Matrix3f GetTransformedInverseInertia(BodyId bodyId) const{
        return bodies[bodyId].GetTransformedInverseInertia(); 
    }
    inline Vector3f PointToLocal(BodyId bodyId,Vector3f p) const{
        return bodies[bodyId].PointToLocal(p);
    }
    inline Vector3f PointToGlobal(BodyId bodyId,Vector3f r) const{
        return bodies[bodyId].PointToGlobal(r);
    }
    inline void AddJoint(std::shared_ptr<Joint> joint){
        distanceJoints.push_back(joint);
    }
    void step(float dt);
private:
    static constexpr int GAUSS_SEIDEL_ITERATIONS = 512;
    std::vector<RigidBody> bodies;
    std::vector<std::shared_ptr<Joint>> distanceJoints;
    Vector3f g;
    PlaneConstraint plane_c = PlaneConstraint(Vector3f(0,1,0),0);
    CollisionConstraint collision_c;
};

class DistanceJoint:public Joint{
public:
    DistanceJoint(uint64_t bodyId1,uint64_t bodyId2,float r):bodyId1(bodyId1),bodyId2(bodyId2),r(r){}
    virtual bool ApplyImpulses(std::vector<RigidBody>& bodies,float dt){
        RigidBody &b1 = bodies[bodyId1],&b2 = bodies[bodyId2];
        Vector3f J = b1.x - b2.x;
        float C = .5 * J.NormSquared() - r*r;
        float K = (b1.inverseMass + b2.inverseMass ) * Dot(J,J);
        float last_lambda = lambda;
        lambda = (Dot(b1.v - b2.v,J) + BETA*C/dt)/ K;
        if(K >= TOLLERANCE){
            b1.v +=-b1.inverseMass * lambda * J;
            b2.v += b2.inverseMass * lambda * J;
        }
        return std::fabs(lambda - last_lambda)/lambda < CONVERGENCE_TRESHOLD;
    }
private:
    float lambda;
    const uint64_t bodyId1,bodyId2;
    const float r;
    static constexpr float BETA = .01;
    static constexpr float TOLLERANCE = 1e-8;
    static constexpr float CONVERGENCE_TRESHOLD = 10.0 / 100.0;
};

class BodyRef{
    friend class World;    
    public:
    using BodyId=World::BodyId;
    BodyRef(World& w, BodyId bid):world(w), id(bid) {}
    inline Matrix4f BodyTransform(){
        return world.BodyTransform(id);
    }
    inline Vector3f GetPosition(){
        return world.GetPosition(id);
    }
    inline Quaternionf GetOrientation(){
        return world.GetOrientation(id);
    }
    inline Vector3f GetVelocity(){
        return world.GetVelocity(id);
    }
    inline Vector3f GetAngularVelocity(){
        return world.GetAngularVelocity(id);
    }
    inline float GetInverseMass() const {
        return world.GetInverseMass(id);
    }
    inline Matrix3f GetTransformedInverseInertia() const{
        return world.GetTransformedInverseInertia(id);
    }
    inline Vector3f PointToLocal(Vector3f p){
        return world.PointToLocal(id,p); 
    }
    inline Vector3f PointToGlobal(Vector3f r){
        return world.PointToGlobal(id,r);
    }
    inline BodyId GetId(){
        return id;
    }

    private:
    World& world;
    BodyId id;
};

#endif
