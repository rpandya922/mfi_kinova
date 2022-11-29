#pragma once

#include "common.hpp"

namespace rc
{
    typedef Eigen::Matrix<float, 6, 1> Vector6f;
    typedef Eigen::Matrix<float, 7, 1> Vector7f;

    void PrintMat(const Eigen::Matrix4f& mat);

    template<typename T>
    Eigen::Matrix<T, Eigen::Dynamic, 1> ToEigen(std::vector<T> data);

    void PoseMatToTransEulerRad(const Eigen::Matrix4f& mat, float& x, float& y, float& z, float& rx, float& ry, float& rz);
    void PoseMatToTransEulerDeg(const Eigen::Matrix4f& mat, float& x, float& y, float& z, float& rx, float& ry, float& rz);
    void PoseMatToTransEulerRad(const Eigen::Matrix4f& mat, Vector6f& p);
    void PoseMatToTransEulerDeg(const Eigen::Matrix4f& mat, Vector6f& p);
    float PoseMatToTransAngRad(const Eigen::Matrix4f& mat, Vector6f& p);
    void PoseMatToTransAngDeg(const Eigen::Matrix4f& mat, Vector6f& p);
    void PoseMatToTransQuat(const Eigen::Matrix4f& mat, Vector7f& p);

    Eigen::Matrix3f as_symm_matrix(const Eigen::Vector3f& vec);
    Eigen::Matrix4f twist_transform(const Eigen::Vector3f& w, const Eigen::Vector3f& q, 
                         float theta);
    Eigen::Matrix<float, 6, 6> adjoint_transform(const Eigen::Matrix4f& mat);
    
    void NormalizeAnglesNegPI_PI(Eigen::VectorXf& vec);
    void NormalizeAnglesNegPI_PI(float& ang);

    Eigen::MatrixXf PInv(const Eigen::MatrixXf& M);
    
    int64_t GetTickUs();

}