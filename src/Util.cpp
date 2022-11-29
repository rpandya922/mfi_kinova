#include "Util.hpp"

namespace rc
{
    template Eigen::Matrix<float, Eigen::Dynamic, 1> ToEigen<float>(std::vector<float> data);

    void PrintMat(const Eigen::Matrix4f& mat)
    {
        for (int i=0; i<4; ++i)
        {
            for (int j=0; j<4; ++j)
            {
                auto val = mat(i, j);
                if (abs(val) <= 1e-3) val = 0;
                std::cout << val << " ";
            }
            std::cout << "\n";
        }
    }

    template <typename T>
    Eigen::Matrix<T, Eigen::Dynamic, 1> ToEigen(std::vector<T> data)
    {
        Eigen::Matrix<T, Eigen::Dynamic, 1> data_eigen;
        size_t n = data.size();
        data_eigen.resize(n, 1);
        for (size_t i = 0; i < n; ++i)
        {
            data_eigen(i) = data.at(i);
        }
        return data_eigen;
    }

    void PoseMatToTransEulerRad(const Eigen::Matrix4f& mat, float& x, float& y, float& z, float& rx, float& ry, float& rz)
    {
        x = mat(0, 3);
        y = mat(1, 3);
        z = mat(2, 3);
        
        Eigen::Vector3f rot = mat.block<3, 3>(0, 0).eulerAngles(2, 1, 0);
        rz = rot(0);
        ry = rot(1);
        rx = rot(2);
    }

    void PoseMatToTransEulerDeg(const Eigen::Matrix4f& mat, float& x, float& y, float& z, float& rx, float& ry, float& rz)
    {
        PoseMatToTransEulerRad(mat, x, y, z, rx, ry, rz);
        rz = RAD2DEG(rz);
        ry = RAD2DEG(ry);
        rx = RAD2DEG(rx);
    }

    void PoseMatToTransEulerRad(const Eigen::Matrix4f& mat, Vector6f& p)
    {
        p(0) = mat(0, 3);
        p(1) = mat(1, 3);
        p(2) = mat(2, 3);
        
        Eigen::Vector3f rot = mat.block<3, 3>(0, 0).eulerAngles(2, 1, 0);
        p(5) = rot(0);
        p(4) = rot(1);
        p(3) = rot(2);
    }

    void PoseMatToTransEulerDeg(const Eigen::Matrix4f& mat, Vector6f& p)
    {
        PoseMatToTransEulerRad(mat, p);
        p(5) = RAD2DEG( p(5) ) ;
        p(4) = RAD2DEG( p(4) ) ;
        p(3) = RAD2DEG( p(3) ) ;
    }

    float PoseMatToTransAngRad(const Eigen::Matrix4f& mat, Vector6f& p)
    {
        // translation
        p.head(3) = mat.block<3, 1>(0, 3);

        Eigen::AngleAxisf angular_d{mat.block<3, 3>(0, 0)};

        // Angular displacement
        p(3) = angular_d.axis()(0) * angular_d.angle();
        p(4) = angular_d.axis()(1) * angular_d.angle();
        p(5) = angular_d.axis()(2) * angular_d.angle();

        return angular_d.angle();
    }

    void PoseMatToTransAngDeg(const Eigen::Matrix4f& mat, Vector6f& p)
    {
        PoseMatToTransAngRad(mat, p);
        p(5) = RAD2DEG( p(5) );
        p(4) = RAD2DEG( p(4) );
        p(3) = RAD2DEG( p(3) );
    }

    void PoseMatToTransQuat(const Eigen::Matrix4f& mat, Vector7f& p) {
        p(0) = mat(0, 3);
        p(1) = mat(1, 3);
        p(2) = mat(2, 3);

        Eigen::Quaternionf q(mat.block<3, 3>(0, 0));
        q.normalize();
        p(3) = q.x();
        p(4) = q.y();
        p(5) = q.z();
        p(6) = q.w();
    }

    void NormalizeAnglesNegPI_PI(Eigen::VectorXf& vec) {
        for (int i=0; i<vec.size(); ++i) {
            while (vec(i) > M_PI) vec(i) -= 2*M_PI;
            while (vec(i) <= -M_PI) vec(i) += 2*M_PI;
        }
    }

    void NormalizeAnglesNegPI_PI(float& ang) {
        while (ang > M_PI) ang -= 2*M_PI;
        while (ang <= -M_PI) ang += 2*M_PI;
    }

    Eigen::Matrix3f as_symm_matrix(const Eigen::Vector3f& vec) {
        Eigen::Matrix3f res_mat = Eigen::Matrix3f::Zero();
        res_mat << 0, -vec(2), vec(1),
                   vec(2), 0, -vec(0),
                   -vec(1), vec(0), 0;
        return res_mat;
    }

    Eigen::Matrix4f twist_transform(const Eigen::Vector3f& w, const Eigen::Vector3f& q, 
                         float theta) {
        Eigen::Matrix4f res_mat = Eigen::Matrix4f::Zero();
        Eigen::Matrix3f w_hat = as_symm_matrix(w);
        auto v = -w_hat * q;
        auto rot = (Eigen::Matrix3f::Identity() + sinf(theta) * w_hat + 
                    (1 - cosf(theta)) * w_hat * w_hat);
        auto trans = (Eigen::Matrix3f::Identity() - rot) * (w_hat * v) + w * w.transpose() * v * theta; 

        res_mat.block<3, 3>(0, 0) = rot;
        res_mat.block<3, 1>(0, 3) = trans;
        res_mat.block<1, 3>(3, 0).setZero();
        res_mat(3, 3) = 1.0f;

        return res_mat;
    }

    Eigen::Matrix<float, 6, 6> adjoint_transform(const Eigen::Matrix4f& mat) {
        Eigen::Matrix<float, 6, 6> adj;
        adj.setZero();

        auto rot = mat.block<3, 3>(0, 0);
        auto trans = mat.block<3, 1>(0, 3);
        
        adj.block<3, 3>(0, 0) = rot;
        adj.block<3, 3>(0, 3) = as_symm_matrix(trans) * rot;
        adj.block<3, 3>(3, 3) = rot;

        return adj;
    }

    Eigen::MatrixXf PInv(const Eigen::MatrixXf& M)
    {
        auto nrow = M.rows();
        auto ncol = M.cols();
        Eigen::MatrixXf Minv;

        if (nrow > ncol)
        {
            Minv = ( ( M.transpose() * M ).inverse() ) * M.transpose();
        }
        else if (nrow < ncol)
        {
            Minv = M.transpose() * ( ( M * M.transpose() ).inverse() );
        }
        else
        {
            Minv = M.inverse();
        }

        return Minv;
    }

    int64_t GetTickUs()
    {
        struct timespec start;
        clock_gettime(CLOCK_MONOTONIC, &start);

        return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
    }
    
}