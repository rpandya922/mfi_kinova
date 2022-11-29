#include "ArmBase.hpp"
#include <string>
#include <random>

namespace rc
{
    ArmBase::ArmBase(int n_joint, Eigen::VectorXf a, Eigen::VectorXf alpha, Eigen::VectorXf d, 
                     Eigen::VectorXf joints_lb, Eigen::VectorXf joints_ub,
                     const bool& low_level_ctrl, float ik_early_stop_tol, float max_ik_pose_error_tol, 
                     float delta_error_tol, size_t ik_max_iter,  size_t ik_max_attempts, size_t ik_min_attempts,
                     float ik_damping, float ik_alpha) :
        Nj_{n_joint}, a_{a}, alpha_{alpha}, d_{d}, joints_lb_{joints_lb}, joints_ub_{joints_ub}, in_low_level_{false}, low_level_control_{low_level_ctrl},
        tool_force_x_{0}, tool_force_y_{0}, tool_force_z_{0}, ik_early_stop_tol_{ik_early_stop_tol}, max_ik_pose_error_tol_{max_ik_pose_error_tol},
        delta_error_tol_(delta_error_tol), ik_max_iter_(ik_max_iter), ik_max_attempts_{ik_max_attempts}, ik_min_attempts_{ik_min_attempts},
        ik_damping_{ik_damping}, ik_alpha_{ik_alpha}
    {
        this->Init();
    }

    void ArmBase::Init()
    {
        try
        {
            q_ = Eigen::VectorXf::Zero(Nj_, 1);
            q_dot_ = q_;
            joint_torques_ = q_;
            joint_torques_cmd_ = q_;

            // Kinematics
            this->GenerateLinkTransform();
            
            // Dynamic stat
            // EE
            pEE_euler_    = Eigen::VectorXf::Zero(6, 1);
            pEE_euler_fk_ = Eigen::VectorXf::Zero(6, 1);
            // vEE_    = Eigen::VectorXf::Zero(6, 1); // ! Unverified
            vEE_fk_ = Eigen::VectorXf::Zero(6, 1);

            J_EE_    = Eigen::MatrixXf::Zero(6, Nj_);
            J_dot_EE_    = Eigen::MatrixXf::Zero(6, Nj_);

            // Joint
            zj_ = Eigen::MatrixXf::Zero(3, Nj_);
            wj_ = Eigen::MatrixXf::Zero(3, Nj_);
            rj_ = Eigen::MatrixXf::Zero(3, Nj_);
            vj_ = Eigen::MatrixXf::Zero(3, Nj_);
            J_Joint_.clear();
            for (size_t i = 0; i < Nj_; ++i)
            {
                Eigen::Matrix3Xf placeholder_jacobi_ = Eigen::MatrixXf::Zero(3, i);
                J_Joint_.emplace_back(placeholder_jacobi_);
            }
            
        }
        catch(const std::exception& e)
        {
            // SPDLOG_ERROR(e.what());
            throw e;
        }
        
    }

    void ArmBase::ReachCartesian(const Eigen::Affine3f& pose)
    {
        // try
        // {
        //     this->ExecuteCartesianAction(pose);
        //     while (!this->CartesianActionEnded())
        //     {
        //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //     }
        // }
        // catch(const std::exception& e)
        // {
        //     SPDLOG_ERROR(e.what());
        //     throw e;
        // }
    }

    void ArmBase::cartesian_fk(const Eigen::VectorXf& q_input, Eigen::Matrix4f& res) {
        auto f_transform = link_transform_.at(0);
        res = f_transform(0.0f);

        for (size_t i = 0; i < Nj_; ++i)
        {   
            // i -> i+1, result is pose of joint i+1
            f_transform = link_transform_.at(i + 1);
            res = res * f_transform(q_input(i));
        }
    }

    void ArmBase::cartesian_fk_twist(const Eigen::VectorXf& q_input, Eigen::Matrix4f& res) {
        res = Eigen::Matrix4f::Identity();
        for (int i = 0; i < Nj_; ++i)
        {
            res = res * twist_transform(twist_ws_[i], twist_qs_[i], q_input(i));
        }

        res = res * ee_g0_;
    }

    void ArmBase::cartesian_joint_ee_fk_twist(const Eigen::VectorXf& q_input, 
    Eigen::Matrix4f& ee_pose, std::vector<Eigen::Matrix4f>& joint_poses) {
        Eigen::Matrix4f g = Eigen::Matrix4f::Identity();
        joint_poses.clear();
        for (int i = 0; i < Nj_; ++i)
        {
            joint_poses.push_back(g * joint_g0_[i]);
            g = g * twist_transform(twist_ws_[i], twist_qs_[i], q_input(i));
        }

        // End effector
        ee_pose = g * ee_g0_;
    }

    void ArmBase::cartesian_link_ee_fk_twist(const Eigen::VectorXf& q_input, 
    Eigen::Matrix4f& ee_pose, std::vector<Eigen::Matrix4f>& joint_poses) {
        Eigen::Matrix4f g = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f link_g0;
        joint_poses.clear();
        for (int i = 0; i < Nj_; ++i)
        {
            link_g0 = joint_g0_[i];
            link_g0.block<3, 1>(0, 3) += link_com_offset_[i];
            joint_poses.push_back(g * link_g0);
            g = g * twist_transform(twist_ws_[i], twist_qs_[i], q_input(i));
        }

        // End effector
        ee_pose = g * ee_g0_;
    }

    void ArmBase::CalculateGravComp(const Eigen::VectorXf& q_input, Eigen::VectorXf& res) {
        auto J_EE_local = J_EE_;
        bool use_link_offset = false;
        res.setZero();  // NOTE: Super Important!!! Otherwise, initial values could be anything!

        std::vector<Eigen::Matrix3Xf> J_Joint_local;
        for (int i = 0; i < Nj_; ++i)
        {
            Eigen::Matrix3Xf placeholder_jacobi_ = Eigen::MatrixXf::Zero(3, i+1);
            J_Joint_local.emplace_back(placeholder_jacobi_);
        }
        
        J_EE_local.setZero(); 
        CalculateJacobianAll(q_input, J_EE_local, J_Joint_local, use_link_offset);

        // Compensation required for the ith joint
        for (int i=0; i<Nj_; i++) {
            // Compute wrench caused by gravity on the ith joint
            auto joint_grav_force = joint_grav_forces_[i];

            // Aggregate all the torques needed to support each joint
            // Subtract to get the torques to counter gravity
            res.segment(0,i+1) -= J_Joint_local[i].transpose() * joint_grav_force;
        }

        // Compensation required for the EE
        auto ee_and_item_grav_force = ee_grav_force_;
        ee_and_item_grav_force(2) += extra_mass_ * -9.81;
        for (int i=0; i<Nj_; i++) {
            res(i) -= J_EE_local.block<3, 1>(0, i).transpose() * ee_and_item_grav_force;
        }
    }

    void ArmBase::calc_pEE_error(const Vector7f& pEE_cur_quat, const Vector7f& pEE_des_quat, Vector6f& res) {
        // Translation error
        auto T1 = pEE_cur_quat.head(3);
        auto T2 = pEE_des_quat.head(3);
        auto trans_error = (T2 - T1);

        auto q1 = pEE_cur_quat.tail(4);
        auto q2 = pEE_des_quat.tail(4);
        Eigen::Vector3f q1_vec = q1.head(3);
        Eigen::Vector3f q2_vec = q2.head(3);

        // NOTE: DO NOT USE API VERSION, has some stochasticity in output 
        // likely due to some normalization and very small floating numbers
        Eigen::Vector3f q1_cross_q2({q1_vec(1) * q2_vec(2) - q1_vec(2) * q2_vec(1),
                                     q1_vec(2) * q2_vec(0) - q1_vec(0) * q2_vec(2),
                                     q1_vec(0) * q2_vec(1) - q1_vec(1) * q2_vec(0)});
        // NOTE: typical quat error uses -q2 x q1, we used -(-q1 x q2)
        auto rot_error = 4 * (q1(3) * q2_vec - q2(3) * q1_vec + q1_cross_q2);
        // std::cout << rot_error << std::endl;

        res << trans_error, rot_error;
    }

    float ArmBase::iterative_ik(const Eigen::Matrix4f& pEE_des, Eigen::VectorXf& res,
            const Vector7f& q_curr, float max_q_dist_tol) {
        /*
        Implements Damped Least Squares version of iterative Inverse Kinematics to avoid joint singularities.
        Runs multiple attempts from diff initial joint configs, but tries the current config first. Among the solutions
        that achieve required pose error < tol, the solution closest to current joint config is chosen. 

        Returns sum of absolute error between found solution and current joint config. Depending on scenario,
        the joint solution may be unacceptable by requiring large arm motion. In this case, a safe, smooth interpolation
        in joint space is necessary and left to the end user. 
        */
        q_(0) = q_curr(0);
        q_(1) = q_curr(1);
        q_(2) = q_curr(2);
        q_(3) = q_curr(3);
        q_(4) = q_curr(4);
        q_(5) = q_curr(5);
        q_(6) = q_curr(6);

        auto start = GetTickUs();
        Vector7f pEE_des_quat;
        PoseMatToTransQuat(pEE_des, pEE_des_quat);

        // Local, temporary dynamics states that change iteratively
        auto q_local = q_;
        Eigen::VectorXf q_diff(Nj_);
        // std::cout << "Init q: " << q_ << std::endl; 
        Vector6f error_vec;
        Vector7f pEE_quat_local;
        Eigen::Matrix4f pEE_mat_fk_local;
        auto J_EE_local = J_EE_;  // overwritten during computation
        auto J_Joint_local = J_Joint_;

        // generate random joint angles as random seeds for grad desc
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> rand_dist(0, 2*M_PI);

        // Keeping the solution closest to current joint config
        float min_q_dist = 1e6;
        float min_error = 1e6;
        int attempt;
        std::cout << ik_max_attempts_ << ", " << ik_min_attempts_ << std::endl;
        for (attempt=0; attempt < ik_max_attempts_; attempt++) {
            if (attempt == 0) {
                q_local = q_;
            } else {
                // Randomly initialize joints 
                for (int j=0; j<Nj_; j++) q_local(j) = rand_dist(gen);
            }

            // Inner optimization params
            size_t it = 0;
            float error = ik_early_stop_tol_;
            float delta_error = 10 * delta_error_tol_;
            float prev_error = 0.0f;
            while (error >= ik_early_stop_tol_ && it < ik_max_iter_ && delta_error >= delta_error_tol_) {
                it++;
                // Update EE pose
                cartesian_fk(q_local, pEE_mat_fk_local);
                PoseMatToTransQuat(pEE_mat_fk_local, pEE_quat_local);

                // if (it == 1) {
                //     std::cout << "pEE_quat_local" << std::endl;
                //     std::cout << pEE_quat_local << std::endl;
                //     std::cout << "pEE_des_quat" << std::endl;
                //     std::cout << pEE_des_quat << stcd::endl;
                // }

                // Update Jacobian
                CalculateJacobian(q_local, J_EE_local, J_Joint_local);

                // calculate change in error in pose
                calc_pEE_error(pEE_quat_local, pEE_des_quat, error_vec);
                error = error_vec.norm();
                delta_error = fabs(error - prev_error);
                prev_error = error;
                // std::cout << "error_vec" << std::endl;
                // std::cout << error_vec << std::endl;
                // std::cout << "error" << ", " << "delta_error" << std::endl;
                // std::cout << error << ", " << delta_error << std::endl;

                // calculate new target joints
                // auto delta_joints = SolveIK(Jacobian(), pose_error);
                // (Nj x 6) * [(6 x Nj) * (Nj x 6) + (6 x 6)] * (6 x 1) = (Nj x 1)
                auto delta_joints = J_EE_local.transpose() * (
                    J_EE_local * J_EE_local.transpose() + 
                    ik_damping_ * Eigen::MatrixXf::Identity(6, 6)
                    ).inverse() * error_vec;
                // std::cout << "qcur: " << q_local << std::endl;
                // std::cout << "M_cur: " << pEE_mat_fk_local << std::endl;
                // std::cout << "pEE_quat_local: " << pEE_quat_local << std::endl;
                // std::cout << "alpha * delta_joints" << std::endl;
                // std::cout << alpha * delta_joints << std::endl;
                // std::cout << "J_EE_local" << std::endl;
                // std::cout << J_EE_local << std::endl;

                q_local = q_local + ik_alpha_ * delta_joints;

                // std::cout << "delta_joints" << std::endl;
                // std::cout << delta_joints << std::endl;
                // std::cout << "q_local" << std::endl;
                // std::cout << q_local << std::endl;
            }

            // Project back into valid joint range and calculate final error
            NormalizeAnglesNegPI_PI(q_local);  // Must do this first!
            q_local = q_local.cwiseMax(joints_lb_);
            q_local = q_local.cwiseMin(joints_ub_);
            cartesian_fk(q_local, pEE_mat_fk_local);
            PoseMatToTransQuat(pEE_mat_fk_local, pEE_quat_local);
            calc_pEE_error(pEE_quat_local, pEE_des_quat, error_vec);
            error = error_vec.norm();
            // std::cout << "Post q_local: " << q_local << std::endl;
            // std::cout << "Post error: " << error << std::endl;

            // Calculate distance from current joint config
            q_diff = q_local - q_;
            NormalizeAnglesNegPI_PI(q_diff);  // handles wraparound for error in joint angles
            float q_dist = q_diff.cwiseAbs().sum();
            // std::cout << "attempt #" << attempt << " q_diff: " << q_diff << std::endl;
            // std::cout << "q_local: " << q_local << std::endl;
            // std::cout << "num iters: " << it << ", error: " << error << ", q_dist: " << q_diff.cwiseAbs().sum() << std::endl;
            bool found_valid_sol = min_error > max_ik_pose_error_tol_ && error < max_ik_pose_error_tol_ && q_dist < max_q_dist_tol;
            bool found_closer_sol = error < max_ik_pose_error_tol_ && q_dist < min_q_dist;
            if (found_valid_sol || found_closer_sol) {
                res = q_local;

                // prioritize dist to current q over having lower error
                // std::cout << "updated min error: " << min_error << "-> " << error << std::endl;
                // std::cout << "updated min min_q_dist: " << min_q_dist << "-> " << q_diff.cwiseAbs().sum() << std::endl;
                min_error = error;
                min_q_dist = q_dist;
            }
            if (min_error < max_ik_pose_error_tol_ && min_q_dist < max_q_dist_tol && attempt > ik_min_attempts_) break;
        }
        std::cout << "IK ran for " << attempt << " attempts, min_error: " << min_error << " vs thresh: " << max_ik_pose_error_tol_ << std::endl;
        std::cout << "min_q_dist: " << min_q_dist << std::endl;
        std::cout << "Cur q: " << q_ << std::endl; 
        std::cout << "Final q: " << res << std::endl; 
        auto end = GetTickUs();
        std::cout << "Time taken(s): " << (end - start) / 1000000.0f << std::endl; 
        if (min_error >= max_ik_pose_error_tol_) {
            char error_msg[200];
            std::sprintf(error_msg, "IK Failed, error: %.3f", min_error);
            throw std::runtime_error(error_msg);
        }
        return min_q_dist;
    }

    void ArmBase::GenerateLinkTransform()
    {
        try
        {
            link_transform_.clear();
            // Kinematics
            for (size_t i = 0; i < Nj_ + 1; ++i)
            {
                link_transform_.emplace_back(
                    [this, i] (double q) {
                        Eigen::Matrix4f trans_mat;
                        trans_mat << cos(q), -sin(q)*cos(alpha_(i)),  sin(q)*sin(alpha_(i)), a_(i)*cos(q),
                                     sin(q),  cos(q)*cos(alpha_(i)), -cos(q)*sin(alpha_(i)), a_(i)*sin(q),
                                          0,         sin(alpha_(i)),         cos(alpha_(i)),        d_(i),
                                          0,                      0,                      0,            1;
                        return trans_mat;
                    }
                );
            }
        }
        catch(const std::exception& e)
        {
            // SPDLOG_ERROR(e.what());
            throw e;
        }
        
    }

    void ArmBase::CalculateJacobian(const Eigen::VectorXf& q_input, 
            Eigen::Matrix<float, 6, Eigen::Dynamic>& J_EE_out,
            std::vector<Eigen::Matrix3Xf>& J_Joint_out
            ) {
        auto f_transform = link_transform_.at(0);
        auto trans_mat = f_transform(0.0f);
        auto rj_local = rj_;  // overwritten, avoid issue of nonconst Nj_ in Matrix<> template
        auto zj_local = zj_;

        for (size_t i = 0; i < Nj_; ++i)
        {
            // Joint location
            rj_local.col(i) = trans_mat.block<3, 1>(0, 3);
            // Joint direction
            zj_local.col(i) = trans_mat.block<3, 1>(0, 2);
            
            // i -> i+1, result is pose of joint i+1
            f_transform = link_transform_.at(i + 1);
            trans_mat = trans_mat * f_transform(q_input(i));
        }
        Eigen::Vector3f translation_EE_local = trans_mat.block<3, 1>(0, 3);

        // EE jacobian and joint jacobian (2...N)
        for (size_t i = 0; i < Nj_; ++i)
        {
            // EE jacobian
            J_EE_out.block<3, 1>(0, i) = zj_local.col(i).cross( translation_EE_local - rj_local.col(i) );
            J_EE_out.block<3, 1>(3, i) = zj_local.col(i);

            // Joint Jacobian dpj_j/dqi
            for (size_t j = i + 1; j < Nj_; ++j)
            {
                J_Joint_out.at(j).col(i) = zj_local.col(i).cross( rj_local.col(j) - rj_local.col(i) );
            }

        }
    }

    void ArmBase::CalculateJacobianAll(const Eigen::VectorXf& q_input,
    Eigen::Matrix<float, 6, Eigen::Dynamic>& J_EE_out,
            std::vector<Eigen::Matrix3Xf>& J_Joint_out, const bool use_link_offset) {
        /*
        For each joint, take finite difference, which leads to minor 
        change in all the following joints and the EE. 
        Get one Jacobian for each joint and the EE.
        */
        // Approximate EE Jacobian Using Finite Differences
        float eps = 1e-3;
        int start_j;
        std::vector<Eigen::Matrix4f> joint_poses_fwd, joint_poses_back;
        Eigen::Matrix4f ee_pose_fwd, ee_pose_back;
        Eigen::VectorXf q_input_copy = q_input;
        for (int i = 0; i < Nj_; i++) {
            q_input_copy(i) += eps;
            if (use_link_offset) cartesian_link_ee_fk_twist(q_input_copy, ee_pose_fwd, joint_poses_fwd);
            else cartesian_joint_ee_fk_twist(q_input_copy, ee_pose_fwd, joint_poses_fwd);

            q_input_copy(i) -= 2 * eps;
            if (use_link_offset) cartesian_link_ee_fk_twist(q_input_copy, ee_pose_back, joint_poses_back);
            else cartesian_joint_ee_fk_twist(q_input_copy, ee_pose_back, joint_poses_back);

            // ith joint doesn't contribute to itself or any preceding joints
            if (use_link_offset) start_j = i;
            else start_j = i+1;
            for (int j = start_j; j < Nj_; j++) {
                auto joint_pos_fwd = joint_poses_fwd.at(j).block<3, 1>(0, 3);
                auto joint_pos_back = joint_poses_back.at(j).block<3, 1>(0, 3);
                // (f(x+eps) - f(x-eps)) / (2*eps)
                J_Joint_out.at(j).col(i) = (joint_pos_fwd - joint_pos_back) / (2 * eps);
            }

            // EE contribution
            auto ee_pos_fwd = ee_pose_fwd.block<3, 1>(0, 3);
            auto ee_pos_back = ee_pose_back.block<3, 1>(0, 3);
            J_EE_out.block<3, 1>(0, i) = (ee_pos_fwd - ee_pos_back) / (2 * eps);
            
            // reset ith joint
            q_input_copy(i) += eps;  
        }

    }

    void ArmBase::UpdateDynamicState()
    {
        try
        {
            // Get API pose mat and trans/angular pose
            poseEE_ = Eigen::Translation3f{Eigen::Vector3f{pEE_euler_(0),pEE_euler_(1),pEE_euler_(2)}}
                        *Eigen::AngleAxisf{pEE_euler_(5), Eigen::Vector3f::UnitZ()}
                        *Eigen::AngleAxisf{pEE_euler_(4), Eigen::Vector3f::UnitY()}
                        *Eigen::AngleAxisf{pEE_euler_(3), Eigen::Vector3f::UnitX()};
            PoseMatToTransAngRad(poseEE_.matrix(), pEE_);

            /* -------------------------------------------------------------------------- */
            /*                                 Kinematics                                 */
            /* -------------------------------------------------------------------------- */

            // Pose of joint 1
            auto f_transform = link_transform_.at(0);
            auto trans_mat = f_transform(0.0f);

            for (size_t i = 0; i < Nj_; ++i)
            {
                // Joint location
                rj_.col(i) = trans_mat.block<3, 1>(0, 3);
                // Joint direction
                zj_.col(i) = trans_mat.block<3, 1>(0, 2);
                // Joint angular velocity
                if (i == 0) // Bug fixed: i = 0 refers to first joint, not base
                {
                    wj_.col(i) = zj_.col(i) * q_dot_(i);
                }
                else
                {
                    wj_.col(i) = wj_.col(i - 1) + zj_.col(i) * q_dot_(i);
                }
                
                // i -> i+1, result is pose of joint i+1
                f_transform = link_transform_.at(i + 1);
                trans_mat = trans_mat * f_transform(q_(i));
            }

            poseEE_mat_fk_ = trans_mat;
            
            // FK pose
            PoseMatToTransEulerRad(poseEE_mat_fk_, pEE_euler_fk_);
            PoseMatToTransAngRad(poseEE_mat_fk_, pEE_fk_);
            
            // EE jacobian and joint jacobian (2...N)
            for (size_t i = 0; i < Nj_; ++i)
            {
                // EE jacobian
                J_EE_.block<3, 1>(0, i) = zj_.col(i).cross( Eigen::Vector3f{pEE_euler_fk_.head(3)} - rj_.col(i) );
                J_EE_.block<3, 1>(3, i) = zj_.col(i);

                // Joint Jacobian dpj_j/dqi
                for (size_t j = i + 1; j < Nj_; ++j)
                {
                    J_Joint_.at(j).col(i) = zj_.col(i).cross( rj_.col(j) - rj_.col(i) );
                }

            }

            // EE velocity
            vEE_fk_ = J_EE_ * q_dot_;
            for (size_t i = 1; i < Nj_; ++i)
            {
                vj_.col(i) = J_Joint_.at(i) * q_dot_.head(i);
            }

            // Jacobian time derivative
            for (size_t i = 0; i < Nj_; ++i)
            {
                J_dot_EE_.block<3, 1>(0, i) = ( wj_.col(i).cross(zj_.col(i)) ).cross( Eigen::Vector3f{pEE_euler_fk_.head(3)} - rj_.col(i) ) +
                                        zj_.col(i).cross( Eigen::Vector3f{vEE_fk_.head(3)} - vj_.col(i) );
                J_dot_EE_.block<3, 1>(3, i) = wj_.col(i).cross(zj_.col(i));
            }

        }
        catch(const std::exception& e)
        {
            // SPDLOG_ERROR(e.what());
            throw e;
        }
    }

    void ArmBase::UpdateAll()
    {
        try
        {
            this->UpdateFeedback();
            // this->UpdateJointState();
            this->UpdateDynamicState();
        }
        catch(const std::exception& e)
        {
            // SPDLOG_ERROR(e.what());
            throw e;
        }
        
    }

    void ArmBase::ClearTimer()
    {
        // try
        // {
        //     if (timer_thread_.joinable())
        //     {
        //         // SPDLOG_WARN("Waiting for timer...");
        //         timer_thread_.join();
        //         // SPDLOG_WARN("Timer returned");
        //     }
        //     assert(time_out_);
        // }
        // catch(const std::exception& e)
        // {
        //     SPDLOG_ERROR(e.what());
        // }
    }

    void ArmBase::SetTimer(const long& execution_time_ms)
    {
        // try
        // {
        //     assert(!timer_thread_.joinable());
        //     time_out_ = false;
        //     timer_thread_ = std::thread{
        //         [this] (long duration_ms) {
        //             // SPDLOG_INFO("Timer started...");
        //             std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
        //             time_out_ = true;
        //             // SPDLOG_INFO("Time out {} ms.", duration_ms);
        //         }, execution_time_ms
        //     };
        // }
        // catch(const std::exception& e)
        // {
        //     SPDLOG_ERROR(e.what());
        // }
    }

}