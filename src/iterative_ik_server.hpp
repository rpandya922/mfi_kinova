#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 7, 1> Vector7f;

const int Nj_ = 7; // number of joints
Eigen::VectorXf q_, q_dot_; // joint angles (rad, rad/s)

Eigen::Matrix<float, 6, Eigen::Dynamic> J_EE_;
Eigen::Matrix<float, 6, Eigen::Dynamic> J_dot_EE_;

size_t ik_max_attempts_ = 50;
size_t ik_min_attempts_ = 3;
float ik_early_stop_tol_ = 0.05f;
float delta_error_tol_ = 1e-4;
size_t ik_max_iter_ = 1000;


std::vector<std::function<Eigen::Matrix4f(float q)>> link_transform_; // Base->1, 1->2, ... , N->Tool
std::vector<Eigen::Matrix3Xf> J_Joint_; // {<>, 3x1, 3x2, ... , 3x(N-1)} joint linear jacobian
const Eigen::VectorXf a_, alpha_, d_; // DH parameters. [0] base link, [1~N] joint link, m, rad

void cartesian_fk(const Eigen::VectorXf& q_input, Eigen::Matrix4f& res);
void PoseMatToTransQuat(const Eigen::Matrix4f& mat, Vector7f& p);

int64_t GetTickUs();
void GenerateLinkTransform();