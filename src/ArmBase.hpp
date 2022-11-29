#pragma once

#include "common.hpp"
#include "Util.hpp"

namespace rc
{
    class ArmBase {
        public:

            explicit ArmBase(int n_joint, Eigen::VectorXf a, Eigen::VectorXf alpha, Eigen::VectorXf d, 
                Eigen::VectorXf joints_lb, Eigen::VectorXf joints_ub,
                const bool& low_level_ctrl, float ik_early_stop_tol=0.05f, float max_ik_pose_error_tol=0.1f, 
                float ik_delta_error_tol=1e-4, size_t ik_max_iter=1000, size_t ik_max_attempts=50, size_t ik_min_attempts=3,
                float ik_damping=0.1f, float ik_alpha=0.1f);
            ~ArmBase() = default;

            /* -------------------------------------------------------------------------- */
            /*                              Default routines                              */
            /* -------------------------------------------------------------------------- */
            // Blocking Cartesian action
            void ReachCartesian(const Eigen::Affine3f& pose);
            int GetJointCount() const { return Nj_; };
            const bool& LowLevelCtrl() const { return low_level_control_; }

            // Status report
            const float& x() const { return pEE_euler_(0); }
            const float& y() const { return pEE_euler_(1); }
            const float& z() const { return pEE_euler_(2); }
            const float& rx() const { return pEE_euler_(3); }
            const float& ry() const { return pEE_euler_(4); }
            const float& rz() const { return pEE_euler_(5); }

            const float& x_fk() const { return pEE_euler_fk_(0); }
            const float& y_fk() const { return pEE_euler_fk_(1); }
            const float& z_fk() const { return pEE_euler_fk_(2); }
            const float& rx_fk() const { return pEE_euler_fk_(3); }
            const float& ry_fk() const { return pEE_euler_fk_(4); }
            const float& rz_fk() const { return pEE_euler_fk_(5); }

            // Joint angles/velocities (rad)
            const Eigen::VectorXf& q() const { return q_; }
            const Eigen::VectorXf& q_dot() const { return q_dot_; }

            // External joint torques
            const Eigen::VectorXf& JointTorques() const { return joint_torques_; }

            // Joint limits
            const Eigen::VectorXf& joints_lb() const { return joints_lb_; }
            const Eigen::VectorXf& joints_ub() const { return joints_ub_; }

            const Eigen::Affine3f& pose() const { return poseEE_; }
            const Eigen::Matrix4f& pose_mat_fk() const { return poseEE_mat_fk_; }

            // Cartesian: Translation + Angular displacement
            const Vector6f& cartesian()    const { return pEE_; }
            const Vector6f& cartesian_fk() const { return pEE_fk_; }
            void cartesian_fk(const Eigen::VectorXf& q_input, Eigen::Matrix4f& res);
            void cartesian_joint_ee_fk_twist(const Eigen::VectorXf& q_input, 
                                        Eigen::Matrix4f& ee_pose, 
                                        std::vector<Eigen::Matrix4f>& joint_poses);  // all joints + EE
            void cartesian_link_ee_fk_twist(const Eigen::VectorXf& q_input, 
                                        Eigen::Matrix4f& ee_pose, 
                                        std::vector<Eigen::Matrix4f>& joint_poses);  // all links + EE
            void cartesian_fk_twist(const Eigen::VectorXf& q_input,             
                                    Eigen::Matrix4f& res);  // only EE
            
            // Cartesian: Translation + Angular velocity
            // ! Unverified
            // const Vector6f& cartesian_v()    const { return vEE_; }
            const Vector6f& cartesian_v_fk() const { return vEE_fk_; }

            // Dynamic state
            void CalculateJacobian(const Eigen::VectorXf& q_input, 
                                   Eigen::Matrix<float, 6, Eigen::Dynamic>& J_EE,
                                   std::vector<Eigen::Matrix3Xf>& J_Joint);
            void CalculateJacobianAll(const Eigen::VectorXf& q_input, 
                                    Eigen::Matrix<float, 6, Eigen::Dynamic>& J_EE_out,
                                    std::vector<Eigen::Matrix3Xf>& J_Joint_out,
                                    const bool use_link_offset = false);
            void CalculateGravComp(const Eigen::VectorXf& q_input, 
                                     Eigen::VectorXf& res);
            const Eigen::Matrix<float, 6, Eigen::Dynamic>& Jacobian() const { return J_EE_; }
            const Eigen::Matrix<float, 6, Eigen::Dynamic>& DJacobian() const { return J_dot_EE_; }
            const Eigen::Vector3f& joint_grav_forces(const size_t i) const { return joint_grav_forces_[i]; }
            const Eigen::Vector3f& ee_grav_force() const { return ee_grav_force_; }
            

            // Joint locations/velocities
            Eigen::Matrix3Xf JointPos() const { return rj_; }
            Eigen::Matrix3Xf JointVel() const { return vj_; }
            Eigen::Matrix3Xf JointZDir() const { return zj_; }
            Eigen::Matrix3Xf JointAngVel() const { return wj_; }

            // Joint jacobian
            Eigen::Matrix3Xf JointJacob(const size_t& i) const { return J_Joint_.at(i); }

            // forces
            float ToolForceX () const { return tool_force_x_; }
            float ToolForceY () const { return tool_force_y_; }
            float ToolForceZ () const { return tool_force_z_; }

            // Read joint state, cartesian state, and update dynamic state
            void UpdateAll();

            // Inverse Kinematics
            void calc_pEE_error(const Vector7f& pEE_cur_quat, const Vector7f& pEE_des_quat, Vector6f& res);
            float iterative_ik(const Eigen::Matrix4f& pEE_des, Eigen::VectorXf& res,
                               const Vector7f& q__curr, float max_q_dist_tol=1e3f);

            /* -------------------------------------------------------------------------- */
            /*                        High level cartesian movement                       */
            /* -------------------------------------------------------------------------- */
            virtual void ExecuteCartesianAction(Eigen::Affine3f pose) = 0;
            virtual bool CartesianActionEnded() = 0;

            /* -------------------------------------------------------------------------- */
            /*                                Joint movement                              */
            /* -------------------------------------------------------------------------- */
            virtual void ExecuteJointAction(Eigen::VectorXf joint_acc_cmd, float dt) = 0;
            virtual bool JointActionEnded() = 0;
            virtual void SetupLowLevelControl() = 0;
            virtual void ClearLowLevelControl() = 0;

            /* -------------------------------------------------------------------------- */
            /*                                Twist command                               */
            /* -------------------------------------------------------------------------- */
            virtual void ExecuteTwistCommand(Eigen::VectorXf twist_vel_cmd, float dt) = 0;
            
            /* -------------------------------------------------------------------------- */
            /*                                Status report                               */
            /* -------------------------------------------------------------------------- */
            virtual void UpdateFeedback() = 0;

            /* -------------------------------------------------------------------------- */
            /*                                   Gripper                                  */
            /* -------------------------------------------------------------------------- */
            virtual void OpenGripper() = 0;
            virtual void CloseGripper() = 0;
            void SetExtraMass(float extra_mass) { extra_mass_ = extra_mass; }

            /* -------------------------------------------------------------------------- */
            /*                                  Shutdown                                  */
            /* -------------------------------------------------------------------------- */
            virtual void KeepPosition() = 0;
            virtual void Stop() = 0;

        protected:

            bool low_level_control_, in_low_level_;

            /* -------------------------------------------------------------------------- */
            /*                    Arm: BASE---1---2---...---N---TOOL                      */
            /* -------------------------------------------------------------------------- */
            const int Nj_; // number of joints
            Eigen::VectorXf q_, q_dot_; // joint angles (rad, rad/s)
            Eigen::VectorXf joints_lb_, joints_ub_;

            /* -------------------------------------------------------------------------- */
            /*                                 Kinematics                                 */
            /* -------------------------------------------------------------------------- */

            std::vector<std::function<Eigen::Matrix4f(float q)>> link_transform_; // Base->1, 1->2, ... , N->Tool
            const Eigen::VectorXf a_, alpha_, d_; // DH parameters. [0] base link, [1~N] joint link, m, rad

            // Twist Formulation of FK
            std::vector<float> ll_;  // link lengths
            std::vector<Eigen::Vector3f> twist_qs_;
            std::vector<Eigen::Vector3f> twist_ws_;
            std::vector<Eigen::Matrix4f> joint_g0_;  // Zero configuration transform to joints
            std::vector<Eigen::Vector3f> link_com_offset_;
            Eigen::Matrix4f ee_g0_; // Zero configuration transform to end-effector

            /* -------------------------------------------------------------------------- */
            /*                                 Inverse Kinematics                                 */
            /* -------------------------------------------------------------------------- */
            float ik_alpha_;  // update stepsize or scaling factor
            float ik_damping_;  // damping to penalize singularities
            float ik_early_stop_tol_;  // threshold to stop iterative ik
            float max_ik_pose_error_tol_;  // threshold to consider solved ik as a valid solution
            float delta_error_tol_; // threshold mag of change to consider converged
            size_t ik_max_iter_; // max number of iters for a given attempt
            size_t ik_max_attempts_;  // max number of attempts starting from diff initial seeds
            size_t ik_min_attempts_;  // try enough solutions to hopefully pick one close to current joint config

            /* -------------------------------------------------------------------------- */
            /*                                Dynamic state                               */
            /* -------------------------------------------------------------------------- */
            
            /* ------------------------ From API (if applicable) ------------------------ */
            Vector6f pEE_euler_; // Translation + Euler, Direct read
            Eigen::Affine3f poseEE_; // tool pose in base frame, From pEE_euler_
            Vector6f pEE_; // Cartesian: Translation + Angular https://en.wikipedia.org/wiki/Screw_theory#Twist, From poseEE_
            Eigen::VectorXf joint_torques_;
            Eigen::VectorXf joint_torques_cmd_;

            /* ------------------------------- Calculated ------------------------------- */
            Eigen::Matrix4f poseEE_mat_fk_; // tool pose in base frame
            Vector6f pEE_euler_fk_; // Translation + Euler, From poseEE_mat_fk_
            Vector6f pEE_fk_; // Translation + Angular, From poseEE_mat_fk_
            Vector6f vEE_fk_; // Cartesian velocity
            
            Eigen::Matrix<float, 6, Eigen::Dynamic> J_EE_;
            Eigen::Matrix<float, 6, Eigen::Dynamic> J_dot_EE_;
            Eigen::Matrix3Xf zj_; // 3xN z direction of each joint (rotation axis)
            Eigen::Matrix3Xf wj_; // 3xN angular velocity of z axis
            Eigen::Matrix3Xf rj_; // 3xN joint location
            Eigen::Matrix3Xf vj_; // 3xN joint linear velocity
            std::vector<Eigen::Matrix3Xf> J_Joint_; // {<>, 3x1, 3x2, ... , 3x(N-1)} joint linear jacobian

            /* -------------------------------------------------------------------------- */
            /*                                    Force                                   */
            /* -------------------------------------------------------------------------- */
            float tool_force_x_, tool_force_y_, tool_force_z_;

            /* -------------------------------------------------------------------------- */
            /*                                   Helpers                                  */
            /* -------------------------------------------------------------------------- */
            // std::atomic_bool time_out_;
            // std::thread timer_thread_;


            float extra_mass_ = 0.0f;
            std::vector<Eigen::Vector3f> joint_grav_forces_;
            Eigen::Vector3f ee_grav_force_;

        protected:

            /* -------------------------------------------------------------------------- */
            /*                            Dynamic state update                            */
            /* -------------------------------------------------------------------------- */
            void UpdateDynamicState();
            void ClearTimer();
            void SetTimer(const long& execution_time_ms);

        private:
            
            /* -------------------------------------------------------------------------- */
            /*                    Generate link trasformation matrices                    */
            /* -------------------------------------------------------------------------- */
            void Init();
            void GenerateLinkTransform();
    };
}