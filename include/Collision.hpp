#ifndef COLLISION_HPP
#define COLLISION_HPP
#include <vector>
#include <limits>
#include <memory>


#include "Math.hpp"
#include "Contact.hpp"

struct SupportPoint {
    Vector3f a;
    Vector3f b;
    Vector3f x;
    SupportPoint(){}
    SupportPoint(const Vector3f& a):a(a),x(a){}
    SupportPoint(const Vector3f& a, const Vector3f& b, const Vector3f& x):a(a), b(b), x(x) {}
    SupportPoint(const Vector3f& a,const Vector3f& b):a(a),b(b),x(a-b) {}
    inline SupportPoint operator+(const SupportPoint& p) const noexcept {
        return {a + p.a,b + p.b,x + p.x};
    }
};

inline SupportPoint operator*(float s,const SupportPoint& p) noexcept {
    return {s * p.a,s * p.b,s * p.x};
}

enum class ColliderType{Sphere,Polyhedron};

struct ConvexCollider{
    virtual Vector3f Support(const Vector3f&) const noexcept = 0;
    virtual float GetBoundingRadius() const noexcept = 0;
    virtual ColliderType GetType() const noexcept = 0;
};

struct SphereCollider:public ConvexCollider{
    float radius;
    SphereCollider(float radius):radius(radius){}
    Vector3f Support(const Vector3f& direction) const noexcept override{
        return (radius/direction.Norm())*direction;
    }
    float GetBoundingRadius() const noexcept override{
        return radius;
    }
    ColliderType GetType() const noexcept override{
        return ColliderType::Sphere;
    }
};

struct ConvexPolyhedron:public ConvexCollider{
    std::vector<Vector3f> vertices;
    float radius;
    ConvexPolyhedron(std::vector<Vector3f> vertices):vertices(vertices){
        radius = vertices[0].Norm();
        for(auto& vertex:vertices){
            radius = std::max(radius,vertex.Norm());
        }
    }
    Vector3f Support(const Vector3f& direction) const noexcept override{
        float max_projection = std::numeric_limits<float>::min();
        Vector3f max_vertex = vertices[0];
        for(const auto& v:vertices){
            float projection = Dot(direction,v);
            if(projection >= max_projection) {
                max_projection = projection;
                max_vertex = v;
            }
        }
        return max_vertex;
    }
    float GetBoundingRadius() const noexcept override{
        return radius;
    }
    ColliderType GetType() const noexcept override{
        return ColliderType::Polyhedron;
    }
};


struct AffineTransformCollider:public ConvexCollider{
    Quaternionf orientation;
    Vector3f position;
    const ConvexCollider& collider;
    AffineTransformCollider(const Vector3f& position,const Quaternionf& orientation,const ConvexCollider&  collider):position(position),orientation(orientation),collider(collider){}
    Vector3f Support(const Vector3f& direction) const noexcept override{
        return orientation.Rotate(collider.Support(orientation.Conjugate().Rotate(direction))) + position;
    }
    float GetBoundingRadius() const noexcept override{
        return collider.GetBoundingRadius();
    }
    ColliderType GetType() const noexcept override{
        return collider.GetType();
    }
};


bool gjk(std::vector<SupportPoint>& simplex,const ConvexCollider& c1,const ConvexCollider& c2,Vector3f v = Vector3f(1,0,0)) noexcept;

Contact EPA(const std::vector<SupportPoint>& simplex,const ConvexCollider& c1,const ConvexCollider& c2) noexcept;
    
#endif
