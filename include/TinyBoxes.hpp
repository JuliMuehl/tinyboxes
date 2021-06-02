#include <vector>
#include <algorithm>
#include <memory>
#include <map>

#include "Math.hpp"
#include "Contact.hpp"
#include "Collision.hpp"

struct RigidBody{
    Vector3f x;
    Quaternionf θ;
    Vector3f v;
    Vector3f ω;
    Matrix3f inverseInertia;
    float inverseMass;
    std::shared_ptr<ConvexCollider> collider;
    inline Vector3f GetVelocityAtPoint(Vector3f point) const{
        Vector3f r = x - point;
        return Cross(ω,r) + v;
    }
    inline Matrix3f GetTransformedInverseInertia() const{
        return Matrix3f(θ) * inverseInertia * Matrix3f(θ.Conjugate());
    }
};

struct PlaneConstraint{
    private:
    static constexpr float BETA = .2;
    static constexpr float MU = .5;
    static constexpr float TOLLERANCE = 1e-5;
    Vector3f normal,u1,u2;
    float d;
    public:
    std::vector<std::pair<size_t,Contact>> contacts;
    inline PlaneConstraint(Vector3f normal,float d):normal(normal.Normalize()),d(d){
        if(normal.x != 0) u1 = Cross(normal,Vector3f(0,1,0)).Normalize();
        else u1 = Cross(normal,Vector3f(1,0,0)).Normalize();
        u2 = Cross(u1,normal).Normalize();
    }
    void FindViolations(const std::vector<RigidBody>& bodies);
    inline void ApplyImpulses(std::vector<RigidBody>& bodies,float dt){
        for(auto& entry:contacts){
            auto& id = entry.first;
            auto& contact = entry.second;
            ApplyImpulse(bodies[id],contact,dt);
        }
    }

    private:
    void ApplyImpulse(RigidBody& body,Contact contact,float dt);
};

struct CollisionConstraint{
    private:
    static constexpr float BETA = .2;
    static constexpr float MU = 0.5;
    static constexpr float TOLLERANCE = 1e-4;
    std::map<std::pair<size_t,size_t>,std::vector<Contact>> violations;
    std::map<std::pair<size_t,size_t>,int> last_contact;
    static constexpr float RESTING_VELOCITY = 0.1f;
    static constexpr float CONTACT_TOLLERANCE = .01;
    static constexpr float CONTACT_DEPTH_TOLLERANCE = .01;
    static constexpr unsigned int CONTACT_EXPIRATION = 4;
    public:
    void FindViolations(const std::vector<RigidBody>& bodies);
    inline void ApplyImpulses(std::vector<RigidBody>& bodies,float dt){
        for(auto& entry:violations){
            size_t i = entry.first.first,j = entry.first.second;
            
            auto& contacts = entry.second;
            for(auto& contact:contacts){
                ApplyImpulse(bodies[i],bodies[j],contact,dt);
            }
        }
    }
    private:
    void ApplyImpulse(RigidBody& b1,RigidBody& b2,const Contact& contact,float dt);
};

struct DistanceJoint{
    private:
    const uint64_t bodyId1,bodyId2;
    const float r;
    static constexpr float BETA = .01;
    static constexpr float TOLLERANCE = 1e-8;
    public:
    DistanceJoint(uint64_t bodyId1,uint64_t bodyId2,float r):bodyId1(bodyId1),bodyId2(bodyId2),r(r){}
    inline void ApplyImpulses(std::vector<RigidBody>& bodies,float dt){
        RigidBody &b1 = bodies[bodyId1],&b2 = bodies[bodyId2];
        Vector3f J = b1.x - b2.x;
        float C = .5 * J.NormSquared() - r*r;
        float K = (b1.inverseMass + b2.inverseMass ) * Dot(J,J);
        float lambda = (Dot(b1.v - b2.v,J) + BETA*C/dt)/ K;
        if(K >= TOLLERANCE){
            b1.v +=-b1.inverseMass * lambda * J;
            b2.v += b2.inverseMass * lambda * J;
        }
    }
};

using BodyId = uint64_t;

struct World{
    private:
    static constexpr int GAUSS_SEIDEL_ITERATIONS = 100;
    std::vector<RigidBody> bodies;
    std::vector<DistanceJoint> distanceJoints;
    Vector3f g;
    PlaneConstraint plane_c = PlaneConstraint(Vector3f(0,1,0),0);
    CollisionConstraint collision_c;
    public:
    World(Vector3f g = Vector3f(0,-9.81,0)):g(g){}
    inline BodyId AddBody(RigidBody body){
        bodies.push_back(body);
        return bodies.size() - 1;
    }
    inline Matrix4f BodyTransform(BodyId bodyId){
        const RigidBody& body = bodies[bodyId];
        return Matrix4f(Matrix3f(body.θ),body.x);
    }
    inline Vector3f GetPosition(BodyId bodyId){
        return bodies[bodyId].x;
    }
    inline Quaternionf GetOrientation(BodyId bodyId){
        return bodies[bodyId].θ;
    }
    inline Vector3f GetVelocity(BodyId bodyId){
        return bodies[bodyId].v;
    }
    inline Vector3f GetAngularVelocity(BodyId bodyId){
        return bodies[bodyId].ω;
    }
    inline void AddJoint(DistanceJoint joint){
        distanceJoints.push_back(joint);
    }
    void step(float dt);
};

