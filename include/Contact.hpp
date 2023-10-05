#ifndef CONTACT_HPP
#define CONTACT_HPP
#include <list>
struct Contact{
    float depth;
    Vector3f normal,u1,u2;
    Vector3f point;
    Vector3f r1,r2;
    unsigned int age = 0;
};
#endif
