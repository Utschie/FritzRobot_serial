//
// Created by jsy on 29.10.23.
//

#ifndef SRC_EKF_H
#define SRC_EKF_H
#include "Eigen/Dense"
#include "Eigen/Geometry"
using namespace Eigen;

class EKF
{
public:

    EKF();
    EKF(float _q0, float _q1, float _q2,float _q3);
    //float dt;//时间差
    Vector4f state_vector;
    Vector4f quat_est;//4*1
    void predict(float wx, float wy, float wz,float dt);
    void update();
    void setP(Matrix4f P_);
    void setQ(Matrix4f Q_);
    void setR(Matrix3f R_);
    void setz(float z1,float z2, float z3);

private:
    Matrix4f P;//协方差矩阵6*6
    Matrix4f Q;
    Matrix3f R;
    //Matrix4f P_;

    Vector3f z;//用于保存来自加速度的的归一化的重力加速度值


};

#endif //SRC_EKF_H
