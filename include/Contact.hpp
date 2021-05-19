#ifndef CONTACT_HPP
#define CONTACT_HPP
struct Contact{
    float depth;
    Vector3f normal,u1,u2;
    Vector3f point;
    unsigned int age = 0;
};
#endif
