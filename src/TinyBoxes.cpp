#include "TinyBoxes.hpp"
#include <cstdio>
#include <iostream>

template<typename T>
static T Clamp(const T& x,const T& lower,const T& upper){
    if(x >= upper)
        return upper; 
    if(x <= lower)
        return lower;
    return x;
}

void PlaneConstraint::FindViolations(const std::vector<RigidBody>& bodies){
    contacts.clear();
    for(size_t i = 0;i<bodies.size();i++){
        const RigidBody& b = bodies[i];
        Vector3f support = b.x + b.theta.Rotate(b.collider->Support(b.theta.Conjugate().Rotate(-normal)));
        if(Dot(support,normal) <= d){
            if(b.collider->GetType() == ColliderType::Polyhedron){
                std::shared_ptr<ConvexPolyhedron> c = std::dynamic_pointer_cast<ConvexPolyhedron> (b.collider);
                for(Vector3f v:c->vertices){
                    v = b.theta.Rotate(v) + b.x;
                    float seperation = Dot(v,normal);
                    if(seperation < 0){
                        Contact contact;
                        contact.point = v;
                        contact.depth = -seperation;
                        contact.normal = normal;
                        contact.u1 = u1;
                        contact.u2 = u2;
                        contacts.push_back(std::make_pair(i,contact));
                    }
                }
            }else{
                Contact contact;
                contact.point = support;
                contact.depth = Dot(support,normal);
                contact.normal = normal;
                contact.u1 = u1;
                contact.u2 = u2;
                contacts.push_back(std::make_pair(i,contact));
            }
        }
    }
}

void PlaneConstraint::ApplyImpulse(RigidBody& body,Contact contact,float dt){
    Vector3f r = contact.point - body.x;
    auto invI = body.GetTransformedInverseInertia();

    //Non Penetration
    Vector3f J_v_n = normal;
    Vector3f J_omega_n = Cross(normal,r);
    float C = Dot(body.x + r,normal) - d;
    float K_n = Dot(J_omega_n,invI.Transform(J_omega_n)) + body.inverseMass;
    float b_n = C * BETA / dt;
    Vector3f vTotal = body.v + Cross(r,body.omega);
    float lambda_n = (-Dot(normal,vTotal) - b_n) / K_n;
    lambda_n = std::max(lambda_n,0.0f);
    //Friction
    Vector3f J_v_u1 = u1;
    Vector3f J_v_u2 = u2;
    Vector3f J_omega_u1 = Cross(r,u1);
    Vector3f J_omega_u2 = Cross(r,u2);

    float K_u1 = Dot(J_omega_u1,invI.Transform(J_omega_u1)) + body.inverseMass;
    float K_u2 = Dot(J_omega_u2,invI.Transform(J_omega_u2)) + body.inverseMass;
    float lambda_u1 = -(Dot(J_omega_u1,body.omega) + Dot(body.v,J_v_u1))/K_u1;
    float lambda_u2 = -(Dot(J_omega_u2,body.omega) + Dot(body.v,J_v_u2))/K_u1;
    lambda_u1 = Clamp(lambda_u1,-MU*std::fabs(lambda_n),MU*std::fabs(lambda_n));
    lambda_u2 = Clamp(lambda_u2,-MU*std::fabs(lambda_n),MU*std::fabs(lambda_n));
    if(K_n >= TOLLERANCE && K_u1 >= TOLLERANCE && K_u2 >= TOLLERANCE){
        body.v += body.inverseMass * (lambda_n * J_v_n + lambda_u1*J_v_u1 + lambda_u2*J_v_u2);
        body.omega += invI.Transform(lambda_n * J_omega_n + lambda_u1*J_omega_u1 + lambda_u2*J_omega_u2);
    }
}


void CollisionConstraint::FindViolations(const std::vector<RigidBody>& bodies){
    constexpr float CONTACT_POINT_TOLLERANCE = 1e-2;
    for(size_t i = 0;i<bodies.size()-1;i++){
        for(size_t j = i+1;j<bodies.size();j++){
            bool collisionFound = false;
            auto key = std::make_pair(i,j);
            if((bodies[i].x - bodies[j].x).Norm() <= bodies[i].collider->GetBoundingRadius() + bodies[j].collider->GetBoundingRadius()){
                std::vector<SupportPoint> simplex;
                const RigidBody& b1 = bodies[i];
                const RigidBody& b2 = bodies[j];
                auto c1 = AffineTransformCollider(b1.x,b1.theta,*b1.collider);
                auto c2 = AffineTransformCollider(b2.x,b2.theta,*b2.collider);
                
                if(gjk(simplex,c1,c2)){
                    //last_contact[key] = 0;
                    collisionFound = true;
                    auto contact = EPA(simplex,c1,c2);
                    if(std::isnan(contact.depth)) continue;
                    contact.r1 = b1.PointToLocal(contact.point);
                    contact.r2 = b2.PointToLocal(contact.point);
                    auto& contacts = violations[key];
                    if(b1.collider->GetType() != ColliderType::Polyhedron || b2.collider->GetType() != ColliderType::Polyhedron) {
                        contacts.clear();
                        contacts.push_back(contact);
                        //last_contact[key] = 0;
                        continue;
                    }
                    //bool isResting = (b1.GetVelocityAtPoint(contact.point) - b2.GetVelocityAtPoint(contact.point)).NormSquared() <= RESTING_VELOCITY;
                    //if(!isResting){
                        //contacts.clear();
                    //}
                    bool replacedContact = false;
                    
                    for(Contact& c:contacts){
                        float dot = Dot(c.normal,contact.normal);
                        //(1 - dot * dot) <= CONTACT_NORMAL_TOLLERANCE && 
                        //fabs(contact.depth - c.depth) <= CONTACT_DEPTH_TOLLERANCE 
                        if((c.point - contact.point).NormSquared() <= CONTACT_POINT_TOLLERANCE ){
                            c = contact;
                            replacedContact = true;
                            break;
                        }
                    }

                    //contacts.erase(std::remove_if(contacts.begin(),contacts.end(),[&contact,b1,b2](auto c){return c.age>=CONTACT_EXPIRATION;}),contacts.end());
                    //for(Contact& c:contacts){
                        //c.age++;
                    //}
                    
                    if(!replacedContact) contacts.push_back(contact);
                }
            }
            //if(last_contact[key]++ >= 5) violations[key].clear();
        }
    }
    for(auto& entry : violations){
        auto i = entry.first.first,j=entry.first.second;
        auto& b1 = bodies[i],&b2=bodies[j];
        auto& contacts = entry.second;
        for(auto it = contacts.begin();it != contacts.end();){
            const Contact& c = *it;
            if((b1.PointToGlobal(c.r1) - b2.PointToGlobal(c.r2)).NormSquared() > 0.25*CONTACT_POINT_TOLLERANCE){
                it = contacts.erase(it);
            }else{
                it++;
            }
        }
    }
}

void CollisionConstraint::ApplyImpulse(RigidBody& b1,RigidBody& b2,const Contact& contact,float dt){
    auto invI1 = b1.GetTransformedInverseInertia();
    auto invI2 = b2.GetTransformedInverseInertia();

    Vector3f r2 = contact.point - b2.x;
    Vector3f r1 = contact.point - b1.x;
    float C = contact.depth;

    Vector3f J_v_n = contact.normal;
    Vector3f J_omega1_n = Cross(contact.normal,r1);
    Vector3f J_omega2_n = Cross(contact.normal,r2);
    float K_n = b1.inverseMass + b2.inverseMass +  Dot(J_omega1_n,invI1.Transform(J_omega1_n)) + Dot(J_omega2_n,invI2.Transform(J_omega2_n));
    
    Vector3f v1 = b1.v + Cross(r1,b1.omega);
    Vector3f v2 = b2.v + Cross(r2,b2.omega);
    Vector3f vrel = v1 - v2;
    //float lambda_n = (BETA * C / dt + Dot(contact.normal,vrel)) / K_n;
    float lambda_n = (BETA * C + Dot(contact.normal,vrel)) / K_n;
    lambda_n = std::max(lambda_n,0.0f);

    Vector3f J_v_u1 = contact.u1;
    Vector3f J_omega1_u1 = -Cross(r1,contact.u1);
    Vector3f J_omega2_u1 = -Cross(r2,contact.u1);
    float K_u1 = b1.inverseMass + b2.inverseMass + Dot(J_omega1_u1,invI1.Transform(J_omega1_u1)) + Dot(J_omega2_u1,invI2.Transform(J_omega2_u1));
    float lambda_u1 = (Dot(J_v_u1,b1.v-b2.v) + Dot(J_omega1_u1,b1.omega) - Dot(J_omega2_u1,b2.omega))/K_u1;
    lambda_u1 = Clamp(lambda_u1,-MU*std::fabs(lambda_n),MU*std::fabs(lambda_n));

    Vector3f J_v_u2 = contact.u2;
    Vector3f J_omega1_u2 = Cross(contact.u2,r1);
    Vector3f J_omega2_u2 = Cross(contact.u2,r2);
    float K_u2 = b1.inverseMass + b2.inverseMass + Dot(J_omega1_u2,invI1.Transform(J_omega1_u2)) + Dot(J_omega2_u2,invI2.Transform(J_omega2_u2));
    float lambda_u2 = (Dot(J_v_u2,b1.v-b2.v) + Dot(J_omega1_u2,b1.omega) -Dot(J_omega2_u2,b2.omega))/K_u2;
    lambda_u2 = Clamp(lambda_u2,-MU*std::fabs(lambda_n),MU*std::fabs(lambda_n));
    if(K_u1 >= TOLLERANCE && K_u2 >= TOLLERANCE && K_n >= TOLLERANCE){
        b1.v += -b1.inverseMass*(lambda_n * J_v_n + lambda_u1*J_v_u1 + lambda_u2*J_v_u2);
        b2.v += b2.inverseMass*(lambda_n * J_v_n + lambda_u1*J_v_u1 + lambda_u2*J_v_u2);
        b1.omega += -invI1.Transform(lambda_n * J_omega1_n + lambda_u1 * J_omega1_u1 + lambda_u2 * J_omega1_u2);
        b2.omega += invI2.Transform(lambda_n * J_omega2_n + lambda_u1 * J_omega2_u1 + lambda_u2 * J_omega2_u2);
    }
}

void World::step(float dt){
    for(auto& body:bodies){
        if(body.inverseMass > 0) body.v += dt*g;
    }
    plane_c.FindViolations(bodies);
    collision_c.FindViolations(bodies);
    for(int i = 0;i<GAUSS_SEIDEL_ITERATIONS;i++){
        plane_c.ApplyImpulses(bodies,dt);
        collision_c.ApplyImpulses(bodies,dt);
        for(auto& joint:distanceJoints){
            joint.ApplyImpulses(bodies,dt);
        }
    }
    for(auto& body:bodies){
        body.x += dt*body.v;
        body.theta += dt*(body.theta*body.omega);
        body.theta *= 1/body.theta.Norm();
    }
}
