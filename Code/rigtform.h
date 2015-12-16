#ifndef RIGTFORM_H
#define RIGTFORM_H

#include <iostream>
#include <cassert>

#include "matrix4.h"
#include "quat.h"
using namespace std;

class RigTForm {
  Cvec3 t_; // translation component
  Quat r_;  // rotation component represented as a quaternion

public:
  RigTForm() : t_(0) {
    assert(norm2(Quat(1,0,0,0) - r_) < CS175_EPS2);
  }

  RigTForm(const Cvec3& t, const Quat& r) : t_(t), r_(r)  {}
  explicit RigTForm(const Cvec3& t) : t_(t), r_()                  {}    // only set translation part (rotation is identity)
  explicit RigTForm(const Quat& r) : t_(0), r_(r)                  {}    // only set rotation part (translation is 0)

  Cvec3 getTranslation() const {
    return t_;
  }

  Quat getRotation() const {
    return r_;
  }

  RigTForm& setTranslation(const Cvec3& t) {
    t_ = t;
    return *this;
  }

  RigTForm& setRotation(const Quat& r) {
    r_ = r;
    return *this;
  }

  Cvec4 operator * (const Cvec4& a) const {
    return Cvec4(t_, 0.0) * a[3] + r_ * a;
  }

  RigTForm operator * (const RigTForm& a) const {
    return RigTForm(t_ + Cvec3(r_ * Cvec4(a.t_, 0)), r_*a.r_);
  }
};

inline RigTForm inv(const RigTForm& tform) {
  Quat invRot = inv(tform.getRotation());
  return RigTForm(Cvec3(invRot * Cvec4(-tform.getTranslation(), 1)), invRot);
}

inline RigTForm transFact(const RigTForm& tform) {
  return RigTForm(tform.getTranslation());
}

inline RigTForm linFact(const RigTForm& tform) {
  return RigTForm(tform.getRotation());
}

inline Matrix4 rigTFormToMatrix(const RigTForm& tform) {
  Matrix4 m = quatToMatrix(tform.getRotation());
  const Cvec3 t = tform.getTranslation();
  for (int i = 0; i < 3; ++i) {
    m(i, 3) = t(i);
  }
  return m;
}

inline RigTForm lerp(const RigTForm& tform0, const RigTForm& tform1, 
                     const RigTForm& tform2, const RigTForm& tform3, double t, int i) {
  t += i; 

  // lerp
  Cvec3 d_i = (tform2.getTranslation() - tform0.getTranslation()) * 1.0/6 + tform1.getTranslation(); 
  Cvec3 e_i = (tform3.getTranslation() - tform1.getTranslation()) * -1.0/6 + tform2.getTranslation(); 
  Cvec3 f   = tform1.getTranslation()*(1-t+i) + d_i*(t-i); 
  Cvec3 g   = d_i*(1-t+i) + e_i*(t-i); 
  Cvec3 h   = e_i*(1-t+i) + tform2.getTranslation()*(t-i); 
  Cvec3 m   = f*(1-t+i) + g*(t-i); 
  Cvec3 n   = g*(1-t+i) + h*(t-i); 
  Cvec3 c_t = m*(1-t+i) + n*(t-i); 

  // slerp
  Quat d_i_rotation = pow(shortRotation(tform2.getRotation() * inv(tform0.getRotation())),1.0/6) * tform1.getRotation(); 
  Quat e_i_rotation = pow(shortRotation(tform3.getRotation() * inv(tform1.getRotation())),-1.0/6) * tform2.getRotation(); 
  Quat f_rotation = pow(tform1.getRotation(),(1-t+i)) * pow(d_i_rotation,(t-i)); 
  Quat g_rotation = pow(d_i_rotation,(1-t+i)) * pow(e_i_rotation,(t-i)); 
  Quat h_rotation = pow(e_i_rotation,(1-t+i)) * pow(tform2.getRotation(),(t-i)); 
  Quat m_rotation = pow(f_rotation,(1-t+i)) * pow(g_rotation,(t-i)); 
  Quat n_rotation = pow(g_rotation,(1-t+i)) * pow(h_rotation,(t-i)); 
  Quat c_rotation = pow(m_rotation,(1-t+i)) * pow(n_rotation,(t-i)); 

  return RigTForm(c_t, c_rotation);
}

#endif
