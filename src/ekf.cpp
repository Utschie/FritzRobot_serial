#include "ekf.h"
#include<iostream>

using namespace Eigen;
using namespace std;
EKF::EKF()
{//default parameters
    state_vector << 1.,
            0.,
            0.,
            0.;//the firs 4 elements is in quaternion form
    P = Matrix4f::Zero();
    Q = 0.0001*(Matrix4f::Identity());
    R = 0.00026780980988405645*(Matrix3f::Identity(3,3));
    //dt = 0.005;
}

EKF::EKF(float _q0, float _q1, float _q2,float _q3)
{
    state_vector << _q0,
            _q1,
            _q2,
            _q3;
    P = Matrix4f::Zero();
    Q = 0.0001*(Matrix4f::Identity());
    R = 0.00026780980988405645*(Matrix3f::Identity(3,3));
    z << 0.,
         0.,
         1.;

}

void EKF::setP(Matrix4f P_)
{
    P = P_;
}


void EKF::setQ(Matrix4f Q_)
{
    Q = Q_;
}

void EKF::setR(Matrix3f R_)
{
    R = R_;
}

void EKF::setz(float z1,float z2, float z3)
{
    z << z1,
            z2,
            z3;
    if (z.norm()!=1.0)
    {
        z.normalize();
    }

}


void EKF::predict(float wx, float wy, float wz,float dt)
{
    Matrix4f omega;
    omega << 0., -wx, -wy, -wz,
            wx,  0.,  wz, -wy,
            wy, -wz,  0.,  wx,
            wz,  wy, -wx,  0.;
    Matrix4f F;
    F=Matrix4f::Identity()+0.5*dt*omega;
    quat_est=F*state_vector;
    quat_est.normalize();

    //Matrix4f P_;

    P=F*P*F.transpose()+Q;

}

void EKF::update() {
    float q0, q1, q2, q3;

    q0 = quat_est(0);
    q1 = quat_est(1);
    q2 = quat_est(2);
    q3 = quat_est(3);

    Vector3f h;
    h << 2 * q1 * q3 - 2 * q0 * q2,
         2 * q0 * q1 + 2 * q2 * q3,
         q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    MatrixXf H(3, 4);
    H << -2 * q2, 2 * q3, -2 * q0, 2 * q1,
          2 * q1, 2 * q0, 2 * q3, 2 * q2,
          2 * q0, -2 * q1, -2 * q2, 2 * q3;

    MatrixXf H_T(4, 3);
    H_T = H.transpose();

    Vector3f residual;
    residual = z - h;

    MatrixXf K(4, 3);
    K = P * H_T * ((H * P * H_T + R).inverse());

    Vector4f correction;
    correction = K * residual;
    state_vector = quat_est + correction;//按理说应该是用四元数乘法修正state_vector,但是实际使用中加法反而效果更稳定；
    state_vector.normalize();
    P=P-K*(H*P*H.transpose())*K.transpose();
}