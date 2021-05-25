#include "TinyBoxes.hpp"

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
        Vector3f support = b.x + b.θ.Rotate(b.collider->Support(b.θ.Conjugate().Rotate(-normal)));
        if(Dot(support,normal) <= d){
            if(b.collider->GetType() == ColliderType::Polyhedron){
                std::shared_ptr<ConvexPolyhedron> c = std::dynamic_pointer_cast<ConvexPolyhedron> (b.collider);
                for(Vector3f v:c->vertices){
                    v = b.θ.Rotate(v) + b.x;
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
    Vector3f J_ω_n = Cross(normal,r);
    float C = Dot(body.x + r,normal) - d;
    float K_n = Dot(J_ω_n,invI.Transform(J_ω_n)) + body.inverseMass;
    float b_n = C * BETA / dt;
    Vector3f vTotal = body.v + Cross(r,body.ω);
    float lambda_n = (-Dot(normal,vTotal) - b_n) / K_n;
    lambda_n = std::max(lambda_n,0.0f);
    //Friction
    Vector3f J_v_u1 = u1;
    Vector3f J_v_u2 = u2;
    Vector3f J_ω_u1 = Cross(r,u1);
    Vector3f J_ω_u2 = Cross(r,u2);

    float K_u1 = Dot(J_ω_u1,invI.Transform(J_ω_u1)) + body.inverseMass;
    float K_u2 = Dot(J_ω_u2,invI.Transform(J_ω_u2)) + body.inverseMass;
    float lambda_u1 = -(Dot(J_ω_u1,body.ω) + Dot(body.v,J_v_u1))/K_u1;
    float lambda_u2 = -(Dot(J_ω_u2,body.ω) + Dot(body.v,J_v_u2))/K_u1;
    lambda_u1 = Clamp(lambda_u1,-MU*std::fabs(lambda_n),MU*std::fabs(lambda_n));
    lambda_u2 = Clamp(lambda_u2,-MU*std::fabs(lambda_n),MU*std::fabs(lambda_n));
    if(K_n >= TOLLERANCE && K_u1 >= TOLLERANCE && K_u2 >= TOLLERANCE){
        body.v += body.inverseMass * (lambda_n * J_v_n + lambda_u1*J_v_u1 + lambda_u2*J_v_u2);
        body.ω += invI.Transform(lambda_n * J_ω_n + lambda_u1*J_ω_u1 + lambda_u2*J_ω_u2);
    }
}


void CollisionConstraint::FindViolations(const std::vector<RigidBody>& bodies){
    for(size_t i = 0;i<bodies.size()-1;i++){
        for(size_t j = i+1;j<bodies.size();j++){
            bool collisionFound = false;
            auto key = std::make_pair(i,j);
            if((bodies[i].x - bodies[j].x).Norm() <= bodies[i].collider->GetBoundingRadius() + bodies[j].collider->GetBoundingRadius()){
                std::vector<SupportPoint> simplex;
                auto c1 = AffineTransformCollider(bodies[i].x,bodies[i].θ,*bodies[i].collider);
                auto c2 = AffineTransformCollider(bodies[j].x,bodies[j].θ,*bodies[j].collider);
                if(gjk(simplex,c1,c2)){
                    last_contact[key] = 0;
                    collisionFound = true;
                    auto contact = EPA(simplex,c1,c2);
                    if(std::isnan(contact.depth)) continue;
                    std::vector<Contact>& contacts = violations[key];
                    if(bodies[i].collider->GetType() != ColliderType::Polyhedron || bodies[j].collider->GetType() != ColliderType::Polyhedron) {
                        contacts.clear();
                        contacts.push_back(contact);
                        last_contact[key] = 0;
                        continue;
                    }
                    bool isResting = (bodies[i].GetVelocityAtPoint(contact.point) - bodies[j].GetVelocityAtPoint(contact.point)).NormSquared() <= RESTING_VELOCITY;
                    if(!isResting){
                        contacts.clear();
                    }
                    bool replacedContact = false;
                    
                    for(Contact& c:contacts){
                        if((c.normal,contact.normal).NormSquared() <= CONTACT_TOLLERANCE && (c.point - contact.point).NormSquared() <= CONTACT_TOLLERANCE){
                            c = contact;
                            replacedContact = true;
                            break;
                        }
                    }
                    const RigidBody& b1 = bodies[i];
                    const RigidBody& b2 = bodies[j];
                    contacts.erase(std::remove_if(contacts.begin(),contacts.end(),[&contact,b1,b2](auto c){return c.age>=CONTACT_EXPIRATION || fabs(contact.depth - c.depth) >= CONTACT_DEPTH_TOLLERANCE ;}),contacts.end());
                    for(Contact& c:contacts){
                        c.age++;
                    }
                    if(!replacedContact) contacts.push_back(contact);
                }
            }
            if(last_contact[key]++ >= 5) violations[key].clear();
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
    Vector3f J_ω1_n = Cross(contact.normal,r1);
    Vector3f J_ω2_n = Cross(contact.normal,r2);
    float K_n = b1.inverseMass + b2.inverseMass +  Dot(J_ω1_n,invI1.Transform(J_ω1_n)) + Dot(J_ω2_n,invI2.Transform(J_ω2_n));
    
    Vector3f v1 = b1.v + Cross(r1,b1.ω);
    Vector3f v2 = b2.v + Cross(r2,b2.ω);
    Vector3f vrel = v1 - v2;
    float lambda_n = (BETA * C / dt + Dot(contact.normal,vrel)) / K_n;
    lambda_n = std::max(lambda_n,0.0f);

    Vector3f J_v_u1 = contact.u1;
    Vector3f J_ω1_u1 = -Cross(r1,contact.u1);
    Vector3f J_ω2_u1 = -Cross(r2,contact.u1);
    float K_u1 = b1.inverseMass + b2.inverseMass + Dot(J_ω1_u1,invI1.Transform(J_ω1_u1)) + Dot(J_ω2_u1,invI2.Transform(J_ω2_u1));
    float lambda_u1 = (Dot(J_v_u1,b1.v-b2.v) + Dot(J_ω1_u1,b1.ω) - Dot(J_ω2_u1,b2.ω))/K_u1;
    lambda_u1 = Clamp(lambda_u1,-MU*std::fabs(lambda_n),MU*std::fabs(lambda_n));

    Vector3f J_v_u2 = contact.u2;
    Vector3f J_ω1_u2 = Cross(contact.u2,r1);
    Vector3f J_ω2_u2 = Cross(contact.u2,r2);
    float K_u2 = b1.inverseMass + b2.inverseMass + Dot(J_ω1_u2,invI1.Transform(J_ω1_u2)) + Dot(J_ω2_u2,invI2.Transform(J_ω2_u2));
    float lambda_u2 = (Dot(J_v_u2,b1.v-b2.v) + Dot(J_ω1_u2,b1.ω) -Dot(J_ω2_u2,b2.ω))/K_u2;
    lambda_u2 = Clamp(lambda_u2,-MU*std::fabs(lambda_n),MU*std::fabs(lambda_n));
    if(K_u1 >= TOLLERANCE && K_u2 >= TOLLERANCE && K_n >= TOLLERANCE){
        b1.v += -b1.inverseMass*(lambda_n * J_v_n + lambda_u1*J_v_u1 + lambda_u2*J_v_u2);
        b2.v += b2.inverseMass*(lambda_n * J_v_n + lambda_u1*J_v_u1 + lambda_u2*J_v_u2);
        b1.ω += -invI1.Transform(lambda_n * J_ω1_n + lambda_u1 * J_ω1_u1 + lambda_u2 * J_ω1_u2);
        b2.ω += invI2.Transform(lambda_n * J_ω2_n + lambda_u1 * J_ω2_u1 + lambda_u2 * J_ω2_u2);
    }
}

void World::step(float dt){
    for(auto& body:bodies){
        if(body.inverseMass > 0) body.v += dt*g;
        body.x += dt*body.v;
        body.θ += dt*(body.θ*body.ω);
        body.θ *= 1/body.θ.Norm();
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
}

