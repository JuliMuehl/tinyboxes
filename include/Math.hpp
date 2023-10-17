#ifndef MATH_HPP
#define MATH_HPP

#include <cmath>

struct Vector4f;

struct Vector3f{
    float x,y,z;
    inline Vector3f():x(0),y(0),z(0){}
    inline Vector3f(float x,float y,float z):x(x),y(y),z(z){}
    inline Vector3f(const Vector4f& v);
    inline Vector3f& Normalize(){
        float n = Norm();
        this->x /= n;
        this->y /= n;
        this->z /= n;
        return *this;
    }
    
    float NormSquared() const {
        return x*x+y*y+z*z;
    }
    float Norm() const {
        return std::sqrt(NormSquared());
    }
};

struct Vector4f{
    float x,y,z,w;
    inline Vector4f():x(0),y(0),z(0),w(1){}
    inline Vector4f(const Vector3f& v,float w):x(v.x),y(v.y),z(v.z),w(w){}
    inline Vector4f(const Vector3f& v):Vector4f(v,1){}
    inline Vector4f(float x,float y,float z,float w):x(x),y(y),z(z),w(w){}
};

inline Vector3f::Vector3f(const Vector4f& v):x(v.x),y(v.y),z(v.z){}

inline Vector3f Cross(Vector3f v,Vector3f w){
    return Vector3f(
                v.y*w.z-w.y*v.z,
                v.z*w.x-w.z*v.x,
                v.x*w.y - w.x*v.y
            );
}

inline float Dot(Vector3f v,Vector3f w){
    return v.x*w.x + v.y*w.y + v.z*w.z;
}
inline float Dot(Vector4f v,Vector4f w){
    return v.x*w.x + v.y*w.y + v.z*w.z;
}
inline Vector3f& operator+= (Vector3f& v,Vector3f w){
    v.x += w.x;
    v.y += w.y;
    v.z += w.z;
    return v;
}
inline Vector4f& operator+= (Vector4f& v,Vector4f w){
    v.x += w.x;
    v.y += w.y;
    v.z += w.z;
    v.w += w.w;
    return v;
}
inline Vector3f operator+ (Vector3f v,Vector3f w){
    return Vector3f(v.x+w.x,v.y+w.y,v.z+w.z);
}
inline Vector3f operator- (Vector3f v,Vector3f w){
    return Vector3f(v.x-w.x,v.y-w.y,v.z-w.z);
}
inline Vector4f operator+ (Vector4f v,Vector4f w){
    return Vector4f(v.x+w.x,v.y+w.y,v.z+w.z,v.w + w.w);
}
inline Vector3f operator* (float s,Vector3f v){
    return Vector3f(s*v.x,s*v.y,s*v.z);
}
inline Vector4f operator* (float s,Vector4f v){
    return Vector4f(s*v.x,s*v.y,s*v.z,s*v.w);
}
inline Vector3f& operator*= (Vector3f& v,float s){
    v = s*v;
    return v;
}
inline Vector4f& operator*= (Vector4f& v,float s){
    v = s*v;
    return v;
}
inline Vector3f operator -(Vector3f v){
    return Vector3f(-v.x,-v.y,-v.z);
}
inline bool operator==(Vector3f v,Vector3f w){
    return v.x == w.x && v.y == w.y && v.z == w.z;
}

struct Quaternionf{
    float s;
    Vector3f v;
    inline Quaternionf(Vector3f v):s(0),v(v){}
    inline Quaternionf(float s,Vector3f v):s(s),v(v){}
    inline float NormSquared() const {
        return s*s + v.NormSquared();
    }
    inline float Norm() const {
        return std::sqrt(NormSquared());
    }
    inline Quaternionf Conjugate() const {
        return Quaternionf(s,(-1)*v);
    }
    inline Vector3f Rotate(const Vector3f& v) const; 
};

inline Quaternionf operator+(const Quaternionf& p,const Quaternionf& q){
    return Quaternionf(p.s+q.s,p.v+q.v);
}

inline Quaternionf operator*(const Quaternionf& p,const Quaternionf& q){
    return Quaternionf(p.s*q.s - Dot(p.v,q.v),p.s*q.v + q.s*p.v + Cross(p.v,q.v));
}

inline Quaternionf operator*(float s,const Quaternionf& p){
    return Quaternionf(s*p.s,s*p.v);
}

inline Vector3f Quaternionf::Rotate(const Vector3f& v ) const {
    return ((*this).Conjugate() * Quaternionf(v) * (*this)).v;
}

inline Quaternionf& operator+= (Quaternionf& p,Quaternionf q){
    p.s += q.s;
    p.v += q.v;
    return p;
}

inline Quaternionf& operator*= (Quaternionf& p,float s){
    p.s *= s;
    p.v *= s;
    return p;
}

struct Matrix4f;

struct Matrix3f{
    float a[3][3];
     Matrix3f(){
        for(int i = 0;i<3;i++){
            for(int j = 0;j<3;j++){
                a[i][j] = 0;
            }
        }
    }
    Matrix3f(Quaternionf p){
        float s = 1/p.Norm();
        s *= s;
        a[0][0] = 1-2*s*(p.v.y*p.v.y + p.v.z*p.v.z);
        a[1][1] = 1-2*s*(p.v.z*p.v.z + p.v.x*p.v.x);
        a[2][2] = 1-2*s*(p.v.x*p.v.x + p.v.y*p.v.y);

        a[0][1] = 2*s*(p.v.x*p.v.y - p.s*p.v.z);
        a[1][0] = 2*s*(p.v.x*p.v.y + p.s*p.v.z);

        a[0][2] = 2*(p.v.x*p.v.z + p.s*p.v.y);
        a[2][0] = 2*(p.v.x*p.v.z - p.s*p.v.y);
        
        a[1][2] = 2*s*(p.v.y*p.v.z - p.s*p.v.x);
        a[2][1] = 2*s*(p.v.y*p.v.z + p.s*p.v.x);
    }
    Matrix3f(const Matrix4f& m);
    Vector3f Transform(Vector3f v){
        return Vector3f(
                    a[0][0] * v.x + a[0][1] * v.y + a[0][2] * v.z,
                    a[1][0] * v.x + a[1][1] * v.y + a[1][2] * v.z,
                    a[2][0] * v.x + a[2][1] * v.y + a[2][2] * v.z);
    }

    inline Matrix3f Transpose(){
        Matrix3f res;
        for(int i = 0;i<3;i++){
            for(int j = 0;j<3;j++){
                res.a[i][j] = a[j][i];
            }
        }
        return res;
    }
    
    static inline Matrix3f Identity(){
        Matrix3f result;
        for(int i = 0;i<3;i++){
            result.a[i][i] = 1;
        }
        return result;
    }
};

inline Matrix3f operator*(const Matrix3f& a,const Matrix3f& b){
    Matrix3f res;
    for(int i = 0;i<3;i++)
        for(int j = 0;j<3;j++)
            for(int k = 0;k<3;k++)
                res.a[i][j] += a.a[i][k] * b.a[k][j];
    return res;
}

/*
FIXME:
Defining this operator creates an ambiguity between Quaternionf * Vector3f and Matrix3f * Vector3f
since Quaternionf can be implicitly constructed from Matrix3f.
inline Vector3f operator*(const Matrix3f& a,const Vector3f& v){
    return Vector3f(
                    a.a[0][0] * v.x + a.a[0][1] * v.y + a.a[0][2] * v.z,
                    a.a[1][0] * v.x + a.a[1][1] * v.y + a.a[1][2] * v.z,
                    a.a[2][0] * v.x + a.a[2][1] * v.y + a.a[2][2] * v.z);
}
*/

inline Matrix3f operator*(const float s,const Matrix3f& a){
    Matrix3f res;
    for(int i = 0;i<3;i++)
        for(int j = 0;j<3;j++)
            res.a[i][j] = s * a.a[i][j];
    return res;
}


struct Matrix4f{
    float a[4][4];
    Matrix4f(){
        for(int i = 0;i<4;i++)
            for(int j =0;j<4;j++)
                a[i][j] = 0;
    }
    Matrix4f(Matrix3f m3,Vector3f v){
        for(int i = 0;i<3;i++){
            for(int j = 0;j<3;j++){
                a[i][j] = m3.a[i][j];
            }
        }
        for(int i = 0;i<4;i++){
            a[i][3] = 0;
        }
        a[3][0] = v.x;
        a[3][1] = v.y;
        a[3][2] = v.z;
        a[3][3] = 1;
    }
    static Matrix4f Identity(){
        Matrix4f m;
        for(int i = 0;i<4;i++){
            m.a[i][i] = 1;
        }
        return m;
    }

    inline Vector3f Translation() const{
        return Vector3f(a[3][0],a[3][1],a[3][2]);
    }
};

inline Matrix4f operator*(const Matrix4f& a,const Matrix4f& b){
    Matrix4f res;
    for(int i = 0;i<4;i++)
        for(int j = 0;j<4;j++)
            for(int k = 0;k<4;k++)
                res.a[i][j] += a.a[i][k] * b.a[k][j];
    return res;
}

inline Matrix3f::Matrix3f(const Matrix4f& m){
    for(int i = 0;i<3;i++)
        for(int j = 0;j<3;j++)
            this->a[i][j] = m.a[i][j];
}

#endif
