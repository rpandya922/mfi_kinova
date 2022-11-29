#include "KinovaGen3.hpp"

namespace rc
{

    namespace {
        static void* SendLowLevelCmdThread(void* args)
        {
            // std::cout << "Starting thread!" << std::endl;
            // LowLeveLCtrlArgs* ctrl_args = (LowLeveLCtrlArgs*)args;
            // auto last_cmd_tick_us = GetTickUs();
            // auto base_feedback = ctrl_args->base_cyclic_ptr->Refresh(*(ctrl_args->base_command_ptr), 0);
            // while (1) {
            //     try
            //     {
            //         auto now = GetTickUs();
            //         auto dt = now - last_cmd_tick_us;
            //         if (dt > 1000)
            //         {
            //             auto& actuators = base_feedback.actuators();
            //             for (int jointi = 0; jointi < actuators.size(); ++jointi) {
            //                 ctrl_args->base_command_ptr->mutable_actuators(jointi)->set_position(base_feedback.actuators(jointi).position());
            //             }

            //             // Incrementing unique timestep identifier ensures actuators can reject out of time frames
            //             ctrl_args->base_command_ptr->set_frame_id(ctrl_args->base_command_ptr->frame_id() + 1);
            //             if (ctrl_args->base_command_ptr->frame_id() > 65535)
            //                 ctrl_args->base_command_ptr->set_frame_id(0);

            //             for (int jointi = 0; jointi < actuators.size(); ++jointi) {
            //                 ctrl_args->base_command_ptr->mutable_actuators(jointi)->set_command_id(ctrl_args->base_command_ptr->frame_id());
            //             }

            //             base_feedback = ctrl_args->base_cyclic_ptr->Refresh(*(ctrl_args->base_command_ptr), 0);
            //             last_cmd_tick_us = GetTickUs();
            //         }

            //         // if (now - last_cmd_tick_us > 1000) // Max 1Khz 
            //         // {
            //         //     last_cmd_tick_us = now;
            //         //     std::cout << "Successfully sent commands: " << std::endl;
            //         //     for (int i = 0; i < 7; i++) {
            //         //         float joint_torque = ctrl_args->base_command_ptr->actuators(i).torque_joint();
            //         //         std::cout << "joint " << i << ": " << joint_torque << ", ";
            //         //     }
            //         //     std::cout << std::endl;
            //         // }
            //     }
            //     catch(const std::exception& e)
            //     {
            //         SPDLOG_ERROR(e.what());
            //     }
            // }
        }
    }

    KinovaGen3::KinovaGen3(const std::string& ip_addr, const uint32_t& port, const uint32_t& port_realtime,
                            const std::string& username, const std::string& passwd, const bool& low_level_control):
        ArmBase{JOINT_COUNT,
                ToEigen<float>(std::vector<float>{   0.0f,      0.0f,       0.0f,     0.0f,       0.0f,     0.0f,      0.0f,     0.0f }), // a
                ToEigen<float>(std::vector<float>{   M_PI,    M_PI_2,  1.5f*M_PI,   M_PI_2,  1.5f*M_PI,   M_PI_2, 1.5f*M_PI,     M_PI }), // alpha
                ToEigen<float>(std::vector<float>{ 0.1564f, -0.1284f,   -0.0118f, -0.4208f,   -0.0128f, -0.3143f,      0.0f, -0.1674f - 0.12f }), // d
                ToEigen<float>(std::vector<float>{-M_PI, -2.250f, -M_PI, -2.580f, -M_PI,  -2.0943f, -M_PI}), // joint_lb (-pi, pi]
                ToEigen<float>(std::vector<float>{ M_PI,  2.250f,  M_PI,  2.580f,  M_PI,   2.0943f,  M_PI}), // joint_ub (-pi, pi]
                low_level_control
            }, ip_addr_{ip_addr}, port_{port}, port_realtime_{port_realtime}, username_{username}, passwd_{passwd},
        action_finished_by_notification_{true}, action_abort_by_notification_{true}
    {
        this->InitCommunication();

        // for (size_t i = 0; i < Nj_; ++i)
        // {
        //     auto joint_speed = zero_speeds_.add_joint_speeds();
        //     joint_speed->set_joint_identifier(i);
        //     joint_speed->set_value(0.0f);
        //     joint_speed->set_duration(1);
        // }

        // Define joint and EE masses
        Eigen::Vector3f gravity = {0, 0, -9.81};
        this->joint_grav_forces_ = {
            1.377 * gravity, 
            1.163 * gravity, 
            1.163 * gravity, 
            0.930 * gravity, 
            0.678 * gravity, 
            0.878 * gravity, 
            1.1 * gravity};
        this->ee_grav_force_ = 0.63 * gravity;  // robotiq gripper

        // link lengths
        ll_ = {0.1564f, 
            0.1284f, 
            0.2104f, 
            0.2104f, 
            0.2084f,
            0.1059f, 
            0.1059f, 
            0.0615f + 0.12f};  // 0.12 for gripper center

        twist_qs_.push_back(Eigen::Vector3f{0, 0, 0});
        twist_qs_.push_back(Eigen::Vector3f{0, 0, ll_[0] + ll_[1]});
        twist_qs_.push_back(Eigen::Vector3f{0, -0.0118f, 0});
        twist_qs_.push_back(Eigen::Vector3f{0, 0, ll_[0] + ll_[1] + ll_[2] + ll_[3]});
        twist_qs_.push_back(Eigen::Vector3f{0, -0.0246f, 0});
        twist_qs_.push_back(Eigen::Vector3f{0, 0, ll_[0] + ll_[1] + ll_[2] + ll_[3] + ll_[4] + ll_[5]});
        twist_qs_.push_back(Eigen::Vector3f{0, -0.0246f, 0});

        twist_ws_.push_back(Eigen::Vector3f{0, 0, -1});
        twist_ws_.push_back(Eigen::Vector3f{0, 1, 0});
        twist_ws_.push_back(Eigen::Vector3f{0, 0, -1});
        twist_ws_.push_back(Eigen::Vector3f{0, 1, 0});
        twist_ws_.push_back(Eigen::Vector3f{0, 0, -1});
        twist_ws_.push_back(Eigen::Vector3f{0, 1, 0});
        twist_ws_.push_back(Eigen::Vector3f{0, 0, -1});

        Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();

        // Zero configuration transform to:
        // Joint 1
        mat.block<3, 1>(0, 3) << 0.0, 0.0, ll_[0];
        joint_g0_.push_back(mat);
        link_com_offset_.push_back(Eigen::Vector3f{0, 0.0103, 0.073});  // world frame, not joint frame

        // Joint 2
        mat.block<3, 1>(0, 3) << 0.0, -0.0054f, ll_[0] + ll_[1];
        joint_g0_.push_back(mat);
        link_com_offset_.push_back(Eigen::Vector3f{0, -0.0132, 0.0995});  // world frame, not joint frame

        // Joint 3
        mat.block<3, 1>(0, 3) << 0.0, -0.0118f, ll_[0] + ll_[1] + ll_[2];
        joint_g0_.push_back(mat);
        link_com_offset_.push_back(Eigen::Vector3f{0, 0.006, 0.117});  // world frame, not joint frame

        // Joint 4
        mat.block<3, 1>(0, 3) << 0.0, -0.0182f, ll_[0] + ll_[1] + ll_[2] + ll_[3];
        joint_g0_.push_back(mat);
        link_com_offset_.push_back(Eigen::Vector3f{0, -0.015, 0.0754});  // world frame, not joint frame

        // Joint 5
        mat.block<3, 1>(0, 3) << 0.0, -0.0246f, ll_[0] + ll_[1] + ll_[2] + ll_[3] + ll_[4];
        joint_g0_.push_back(mat);
        link_com_offset_.push_back(Eigen::Vector3f{0, 0.009, 0.0638});  // world frame, not joint frame

        // Joint 6
        mat.block<3, 1>(0, 3) << 0.0, -0.0246f, ll_[0] + ll_[1] + ll_[2] + ll_[3] + ll_[4] + ll_[5];
        joint_g0_.push_back(mat);
        link_com_offset_.push_back(Eigen::Vector3f{0, -0.009, 0.045});  // world frame, not joint frame

        // Joint 7
        mat.block<3, 1>(0, 3) << 0.0, -0.0246f, ll_[0] + ll_[1] + ll_[2] + ll_[3] + ll_[4] + ll_[5] + ll_[6];
        joint_g0_.push_back(mat);
        link_com_offset_.push_back(Eigen::Vector3f{0, 0, 0.024});  // world frame, not joint frame

        // End effector
        ee_g0_ = mat;
        ee_g0_.block<3, 1>(0, 3) << 0.0, -0.0246f, ll_[0] + ll_[1] + ll_[2] + ll_[3] + ll_[4] + ll_[5] + ll_[6] + ll_[7];


        // std::cout << low_level_control_ << std::endl;
        // SPDLOG_INFO("Low level status: {}", low_level_control_);
    }

    KinovaGen3::~KinovaGen3()
    {
        try
        {
            if (low_level_control_) {
                delete thread_args_;
            }
            this->Stop();
        }
        catch(const std::exception& e)
        {
            // SPDLOG_ERROR(e.what());
            throw e;
        }
    }
    // KinovaGen3::~KinovaGen3() = default;

    void KinovaGen3::InitCommunication()
    {
        // try
        // {
        //     SPDLOG_INFO("Init com...");

        //     // API objects
        //     transport_ptr_ = std::make_shared<k_api::TransportClientTcp>();
        //     router_ptr_ = std::make_shared<k_api::RouterClient>(&(*transport_ptr_), 
        //         [](k_api::KError err){
        //             cout << "_________ callback error _________" << err.toString();
        //         }
        //     );
        //     transport_ptr_->connect(ip_addr_, port_);

        //     if (low_level_control_) {
        //         transport_realtime_ptr_ = std::make_shared<k_api::TransportClientUdp>();
        //         router_realtime_ptr_ = std::make_shared<k_api::RouterClient>(&(*transport_realtime_ptr_), 
        //             [](k_api::KError err){
        //                 cout << "_________ callback error _________" << err.toString();
        //                 }
        //         );
        //         transport_realtime_ptr_->connect(ip_addr_, port_realtime_);
        //     }

        //     // Session data connection info
        //     session_info_ptr_ = std::make_shared<k_api::Session::CreateSessionInfo>();
        //     session_info_ptr_->set_username(username_);
        //     session_info_ptr_->set_password(passwd_);
        //     session_info_ptr_->set_session_inactivity_timeout(6e4); // ms
        //     session_info_ptr_->set_connection_inactivity_timeout(2e3);

        //     // Session manager service wrapper
        //     SPDLOG_INFO("Creating session for communication...");
        //     session_manager_ptr_ = std::make_shared<k_api::SessionManager>(&(*router_ptr_));
        //     session_manager_ptr_->CreateSession(*session_info_ptr_);

        //     if (low_level_control_) {
        //         session_manager_realtime_ptr_ = std::make_shared<k_api::SessionManager>(&(*router_realtime_ptr_));
        //         session_manager_realtime_ptr_->CreateSession(*session_info_ptr_);  // can use default session info
        //     }

        //     // Create services
        //     base_ptr_ = std::make_shared<k_api::Base::BaseClient>(&(*router_ptr_));
        //     if (low_level_control_) {
        //         base_cyclic_ptr_ = std::make_shared<k_api::BaseCyclic::BaseCyclicClient>(&(*router_realtime_ptr_));
        //         actuator_config_ptr_ = std::make_shared<k_api::ActuatorConfig::ActuatorConfigClient>(&(*router_ptr_));
        //     } else {
        //         base_cyclic_ptr_ = std::make_shared<k_api::BaseCyclic::BaseCyclicClient>(&(*router_ptr_));
        //     }

        //     // Create messages
        //     action_ptr_ = std::make_shared<k_api::Base::Action>();
        //     action_ptr_->set_name("Training Movement");
        //     action_ptr_->set_application_data("");

        //     twist_cmd_ptr_ = std::make_shared<k_api::Base::TwistCommand>();
        //     servoing_mode_ptr_ = std::make_shared<k_api::Base::ServoingModeInformation>();

        //     base_command_.clear_actuators();

        //     assert(Nj_ == base_ptr_->GetActuatorCount().count());

        //     SPDLOG_INFO("Kinova initialized.");
        // }
        // catch(const std::exception& e)
        // {
        //     SPDLOG_ERROR(e.what());
        //     throw e;
        // }
    }

    void KinovaGen3::Stop()
    {
        // try
        // {
        //     if (session_manager_ptr_ != nullptr)
        //     {
        //         this->KeepPosition();
        //         this->ClearActionNotification();
                
        //         //!MARK LOW LEVEL DOESN'T WORK
        //         this->ClearLowLevelControl();

        //         session_manager_ptr_->CloseSession();
        //         if (low_level_control_) session_manager_realtime_ptr_->CloseSession();

        //         router_ptr_->SetActivationStatus(false);
        //         transport_ptr_->disconnect();
        //         if (low_level_control_) {
        //             router_realtime_ptr_->SetActivationStatus(false);
        //             transport_realtime_ptr_->disconnect();
        //         }

        //         transport_ptr_ = nullptr;
        //         router_ptr_ = nullptr;
        //         session_manager_ptr_ = nullptr;

        //         if (low_level_control_) {
        //             transport_realtime_ptr_ = nullptr;
        //             router_realtime_ptr_ = nullptr;
        //             session_manager_realtime_ptr_ = nullptr;
        //         }
        //     }
        // }
        // catch(const std::exception& e)
        // {
        //     SPDLOG_ERROR(e.what());
        //     throw e;
        // }
        
    }

    // std::function<void(k_api::Base::ActionNotification)> 
    // check_for_end_or_abort(bool& finished, bool& aborted)
    // {
        // return [&finished, &aborted](k_api::Base::ActionNotification notification)
        // {
        //     // The action is finished when we receive a END or ABORT event
        //     switch(notification.action_event())
        //     {
        //         case k_api::Base::ActionEvent::ACTION_ABORT:
        //             spdlog::warn("EVENT : {}", k_api::Base::ActionEvent_Name(notification.action_event()));
        //             aborted = true;
        //             break;
        //         case k_api::Base::ActionEvent::ACTION_END:
        //             SPDLOG_INFO("EVENT : {}", k_api::Base::ActionEvent_Name(notification.action_event()));
        //             finished = true;
        //             break;
        //         default:
        //             SPDLOG_INFO("EVENT : {}", k_api::Base::ActionEvent_Name(notification.action_event()));
        //             break;
        //     }
        // };
    // }

    void KinovaGen3::ClearActionNotification()
    {
        // try
        // {
        //     action_finished_by_notification_ = true;
        //     action_abort_by_notification_ = true;
        //     if (action_notification_.active_)
        //     {
        //         SPDLOG_INFO("Unsubscribing notification handle...");
        //         base_ptr_->Unsubscribe(action_notification_.handle_);
        //         action_notification_.active_ = false;
        //     }
        // }
        // catch(const std::exception& e)
        // {
        //     SPDLOG_ERROR(e.what());
        //     throw e;
        // }
    }

    void KinovaGen3::SetupActionNotification()
    {
        // try
        // {
        //     assert(!action_notification_.active_);
        //     action_finished_by_notification_ = false;
        //     action_abort_by_notification_ = false;
        //     // auto options = k_api::Common::NotificationOptions();
        //     action_notification_.active_ = true;
        //     action_notification_.handle_ = base_ptr_->OnNotificationActionTopic(
        //         check_for_end_or_abort(action_finished_by_notification_, action_abort_by_notification_),
        //         k_api::Common::NotificationOptions()
        //     );
        // }
        // catch(const std::exception& e)
        // {
        //     SPDLOG_ERROR(e.what());
        //     throw e;
        // }
    }

    void KinovaGen3::ExecuteCartesianAction(Eigen::Affine3f pose)
    {
        // try
        // {
        //     float x, y, z, rx, ry, rz;
        //     PoseMatToTransEulerDeg(pose.matrix(), x, y, z, rx, ry, rz);

        //     auto constrained_pose_ptr = action_ptr_->mutable_reach_pose();
        //     auto pose_ptr = constrained_pose_ptr->mutable_target_pose();
        //     pose_ptr->set_x(x);
        //     pose_ptr->set_y(y);
        //     pose_ptr->set_z(z);
        //     pose_ptr->set_theta_x(rx);
        //     pose_ptr->set_theta_y(ry);
        //     pose_ptr->set_theta_z(rz);

        //     this->ClearActionNotification();
        //     this->SetupActionNotification();

        //     SPDLOG_INFO("[ ExecuteCartesianAction ] Translation - (x,y,z): {{{}, {}, {}}}  Rotation - (x,y,z): {{{}, {}, {}}}",
        //                     x, y, z, rx, ry, rz);

        //     base_ptr_->ExecuteAction(*action_ptr_);

        // }
        // catch(const std::exception& e)
        // {
        //     SPDLOG_ERROR(e.what());
        //     throw e;
        // }
        
    }

    bool KinovaGen3::CartesianActionEnded()
    {
        // try
        // {
        //     if (action_finished_by_notification_)
        //     {
        //         this->ClearActionNotification();
        //         return true;
        //     }
        //     if (action_abort_by_notification_)
        //     {
        //         this->ClearActionNotification();
        //         throw std::exception();
        //         return true;
        //     }
        //     else
        //     {
        //         return false;
        //     }
        // }
        // catch(const std::exception& e)
        // {
        //     SPDLOG_ERROR(e.what());
        //     throw e;
        // }
        
    }

    void KinovaGen3::KeepPosition()
    {
        // try
        // {
        //     action_ptr_->clear_reach_pose();
        //     k_api::Base::JointSpeeds joint_speeds_stop;
        //     std::vector<float> stop = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        //     for(size_t i = 0 ; i < stop.size(); ++i)
        //     {
        //         auto joint_speed = joint_speeds_stop.add_joint_speeds();
        //         joint_speed->set_joint_identifier(i);
        //         joint_speed->set_value(stop.at(i));
        //         joint_speed->set_duration(1);
        //     }
        //     base_ptr_->SendJointSpeedsCommand(joint_speeds_stop);
        //     SPDLOG_WARN("Enforcing current position...");
        // }
        // catch(const std::exception& e)
        // {
        //     SPDLOG_ERROR(e.what());
        //     throw e;
        // }
        
    }

    void KinovaGen3::UpdateFeedback()
    {
        // try
        // {
        //     // ! Congested commands block feedback
        //     // this->ClearLowLevelCmd();

        //     feed_back_ = base_cyclic_ptr_->RefreshFeedback();
        //     // SPDLOG_INFO("Robot feedback updated.");

        //     // Pose
        //     pEE_euler_(0) = feed_back_.base().tool_pose_x();
        //     pEE_euler_(1) = feed_back_.base().tool_pose_y();
        //     pEE_euler_(2) = feed_back_.base().tool_pose_z();

        //     pEE_euler_(5) = DEG2RAD(feed_back_.base().tool_pose_theta_z());
        //     pEE_euler_(4) = DEG2RAD(feed_back_.base().tool_pose_theta_y());
        //     pEE_euler_(3) = DEG2RAD(feed_back_.base().tool_pose_theta_x());

        //     // Twist
        //     // ! Unverified
        //     // vEE_(0) = feed_back_.base().tool_twist_linear_x();
        //     // vEE_(1) = feed_back_.base().tool_twist_linear_y();
        //     // vEE_(2) = feed_back_.base().tool_twist_linear_z();

        //     // vEE_(3) = DEG2RAD(feed_back_.base().tool_twist_angular_x());
        //     // vEE_(4) = DEG2RAD(feed_back_.base().tool_twist_angular_y());
        //     // vEE_(5) = DEG2RAD(feed_back_.base().tool_twist_angular_z());

        //     // force
        //     tool_force_x_= feed_back_.base().tool_external_wrench_force_x();
        //     tool_force_y_ = feed_back_.base().tool_external_wrench_force_y();
        //     tool_force_z_ = feed_back_.base().tool_external_wrench_force_z();

        //     std::cout << "Joint Torques" << std::endl;
        //     assert(q_.size() == Nj_);
        //     for (size_t i = 0; i < Nj_; ++i)
        //     {
        //         q_(i) = DEG2RAD(feed_back_.actuators(i).position());
        //         q_dot_(i) = DEG2RAD(feed_back_.actuators(i).velocity());
        //         //joint_torques_(i) = feed_back_.actuators(i).torque();
        //         joint_torques_(i) = feed_back_.actuators(i).torque() + joint_torques_cmd_(i);
        //         std::cout << joint_torques_(i) << std::endl;
        //     }
            

        // }
        // catch(const std::exception& e)
        // {
        //     SPDLOG_ERROR(e.what());
        //     throw e;
        // }
        
    }

    void KinovaGen3::GripperLowLevelHelper(bool is_open)
    {
        
        // k_api::BaseCyclic::Feedback base_feedback = base_cyclic_ptr_->Refresh(base_command_);
        // float position_error;

        // float target_position;
        // float target_velocity;
        // if (is_open) {
        //     target_position = 0.0;   // fully opened or until force limit triggers
        //     target_velocity = 0.0;
        // } else {
        //     target_position = 100.0; // fully closed or until force limit triggers (ie: grab object)
        //     target_velocity = 30.0;  // apply non-zero velocity to mimic "stronger" grasp
        // }

        // std::cout << "Gripper Low Level Helper" << std::endl;
        // int min_steps = 30;
        // int step = 0;
        // while (true)
        // {
        //     try
        //     {
        //         float velocity;
        //         float actual_position, actual_velocity;

        //         // Refresh cyclic data (send command and get feedback)
        //         base_feedback = base_cyclic_ptr_->Refresh(base_command_);
        //         actual_position = base_feedback.interconnect().gripper_feedback().motor()[0].position();
        //         actual_velocity = base_feedback.interconnect().gripper_feedback().motor()[0].velocity();
        //         std::cout << "Gripper position and velocity at step " << step << std::endl;
        //         std::cout << actual_position << ", " << actual_velocity << std::endl;

        //         position_error = target_position - actual_position;

        //         if (fabs(position_error) < 1.5f || (fabs(actual_velocity) < 0.1 && step > min_steps))  // [0, 100]
        //         {
        //             gripper_command_ptr_->set_velocity(target_velocity);
        //             base_cyclic_ptr_->Refresh(base_command_);
        //             std::cout << "not moving, stopping!" << std::endl;
        //             break;
        //         }

        //         velocity = 2.0 * fabs(position_error);
        //         if (velocity > 100.0)
        //         {
        //             velocity = 100.0;
        //         }
                
        //         gripper_command_ptr_->set_position(target_position);
        //         gripper_command_ptr_->set_velocity(velocity);
        //         step++;
        //     }
        //     catch(const std::exception& ex)
        //     {
        //         std::cerr << "Error occurred: " << ex.what() << std::endl;
        //     }
        //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // }
    }

    void KinovaGen3::OpenGripper()
    {
    //     try
    //     {
    //         std::cout << "Open Gripper, low_level_control_: " << low_level_control_ << std::endl;
    //         if (!low_level_control_) {
    //             k_api::Base::GripperCommand gripper_command;
    //             k_api::Base::Gripper gripper_feedback;
    //             k_api::Base::GripperRequest gripper_request;

    //             auto finger = gripper_command.mutable_gripper()->add_finger();

    //             // determine when gripper opened
    //             gripper_request.set_mode(k_api::Base::GRIPPER_POSITION);
                
    //             std::cout << "Opening gripper using speed command..." << std::endl;
    //             gripper_command.set_mode(k_api::Base::GRIPPER_SPEED);
    //             finger->set_finger_identifier(1);
    //             finger->set_value(0.3);
    //             base_ptr_->SendGripperCommand(gripper_command);
                    
    //             bool is_motion_completed = false;
    //             while(!is_motion_completed)
    //             {
    //                 float position =0.0f;
    //                 gripper_feedback = base_ptr_->GetMeasuredGripperMovement(gripper_request);

    //                 if (gripper_feedback.finger_size())
    //                 {
    //                     position = gripper_feedback.finger(0).value();
    //                     cout << "Reported position : " << position << std::endl;
    //                 }

    //                 if (position < 0.01f)
    //                 {
    //                     is_motion_completed = true;
    //                 }
    //             }
    //         } else {
    //             GripperLowLevelHelper(true);
    //         }

    //     }
    //     catch(const std::exception& e)
    //     {
    //         SPDLOG_ERROR(e.what());
    //         throw e;
    //     }
    }

    void KinovaGen3::CloseGripper()
    {
    //     try
    //     {
    //         if (!low_level_control_) {
    //             k_api::Base::GripperCommand gripper_command;
    //             k_api::Base::Gripper gripper_feedback;
    //             k_api::Base::GripperRequest gripper_request;

    //             auto finger = gripper_command.mutable_gripper()->add_finger();

    //             // determine when gripper closed
    //             gripper_request.set_mode(k_api::Base::GRIPPER_SPEED);

    //             std::cout << "Closing gripper using speed command..." << std::endl;
    //             gripper_command.set_mode(k_api::Base::GRIPPER_SPEED);
    //             finger->set_finger_identifier(1);
    //             finger->set_value(-0.3);
    //             base_ptr_->SendGripperCommand(gripper_command);
                    
    //             bool is_motion_completed = false;
    //             while(!is_motion_completed)
    //             {
    //                 float speed = 0.0;
    //                 gripper_feedback = base_ptr_->GetMeasuredGripperMovement(gripper_request);
    //                 if (gripper_feedback.finger_size())
    //                 {
    //                     speed = gripper_feedback.finger(0).value();
    //                     cout << "Reported speed : " << speed  << std::endl;
    //                 }

    //                 if (speed == 0.0f)
    //                 {
    //                     is_motion_completed = true;
    //                 }
    //             }
    //         } else {
    //             GripperLowLevelHelper(false);
    //         }

    //     }
    //     catch(const std::exception& e)
    //     {
    //         SPDLOG_ERROR(e.what());
    //         throw e;
    //     }
    }




    void KinovaGen3::ExecuteJointAction(Eigen::VectorXf joint_acc_cmd, float dt)
    {
    //     try
    //     {
    //         if (joint_acc_cmd.size() != Nj_)
    //         {
    //             SPDLOG_ERROR("joint_acc_cmd.size() is {} while Nj_ is {}", joint_acc_cmd.size(), Nj_);
    //             throw std::logic_error("Input size error.");
    //         }

    //         if (!low_level_control_)
    //         {
    //             std::cout << "ExecuteJointAction not low level?" << std::endl;
    //             auto joint_vel_cmd = this->q_dot_ + joint_acc_cmd * dt;

    //             k_api::Base::JointSpeeds joint_speeds;
    //             k_api::Base::JointSpeeds joint_speeds_lower;
                
    //             for (size_t i = 0; i < Nj_; ++ i)
    //             {
    //                 auto joint_speed = joint_speeds.add_joint_speeds();
    //                 joint_speed->set_joint_identifier(i);
    //                 joint_speed->set_value(fmod( RAD2DEG(joint_vel_cmd(i)), 360.0f ));
    //                 joint_speed->set_duration(1); //! This doesn't work
    //                 // std::cout << " Joint " << i+1 << " cmd: " << fmod( RAD2DEG(joint_vel_cmd(i)), 360.0f ) << std::endl;

    //                 //! Run for a while, then damp
    //                 auto joint_speed_lower = joint_speeds_lower.add_joint_speeds();
    //                 joint_speed_lower->set_joint_identifier(i);
    //                 joint_speed_lower->set_value(fmod( RAD2DEG(joint_vel_cmd(i)) * 0.90f, 360.0f ));
    //                 joint_speed_lower->set_duration(1); //! This doesn't work
                    
    //             }

    //             //! Run for a while, then damp
    //             base_ptr_->SendJointSpeedsCommand(joint_speeds);
    //             // this->SendDelayedCmd(50, joint_speeds_lower);

    //         }
    //         else
    //         {
    //             this->ClearLowLevelCmd();
    //             // SPDLOG_INFO("Sending low level cmd...");
    //             this->SendLowLevelCmd(joint_acc_cmd);
    //             // SPDLOG_INFO("Sent!");
    //         }

    //     }
    //     catch(const std::exception& e)
    //     {
    //         SPDLOG_ERROR(e.what());
    //         throw e;
    //     }
        
    }

    void KinovaGen3::ClearDelayedCmd()
    {
    //     try
    //     {
    //         if (delayed_cmd_thread_.joinable())
    //         {
    //             delayed_cmd_thread_.join();
    //         }
    //     }
    //     catch(const std::exception& e)
    //     {
    //         SPDLOG_ERROR(e.what());
    //     }
    }

    // void KinovaGen3::SendDelayedCmd(const long& execution_time_ms, k_api::Base::JointSpeeds joint_speeds)
    // {
    //     try
    //     {
    //         // Clear delayed cmd when sending next cmd
    //         this->ClearDelayedCmd();
    //         delayed_cmd_thread_ = std::thread{
    //             [this] (long duration_ms, k_api::Base::JointSpeeds joint_speeds) {

    //                 std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
    //                 base_ptr_->SendJointSpeedsCommand(joint_speeds);

    //             }, execution_time_ms, joint_speeds
    //         };
    //     }
    //     catch(const std::exception& e)
    //     {
    //         SPDLOG_ERROR(e.what());
    //     }
    // }

    bool KinovaGen3::JointActionEnded()
    {
    //     try
    //     {
    //         if (action_finished_by_notification_)
    //         {
    //             this->ClearActionNotification();
    //             return true;
    //         }
    //         else if (action_abort_by_notification_)
    //         {
    //             this->ClearActionNotification();
    //             throw std::exception();
    //             return true;
    //         }
    //         else
    //         {
    //             return false;
    //         }
    //     }
    //     catch(const std::exception& e)
    //     {
    //         SPDLOG_ERROR(e.what());
    //         throw e;
    //     }
        
    }

    // auto lambda_fct_callback = [](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data)
    // {
    //     // We are printing the data of the moving actuator just for the example purpose,
    //     // avoid this in a real-time loop
    //     std::string serialized_data;
    //     google::protobuf::util::MessageToJsonString(data.actuators(1), &serialized_data);
    //     std::cout << serialized_data << std::endl << std::endl;
    // };

    void KinovaGen3::ClearLowLevelCmd()
    {
    //     try
    //     {
    //         clear_cmd_ = true;
    //         if (joint_vel_thread_.joinable())
    //         {
    //             joint_vel_thread_.join();
    //             SPDLOG_WARN("[ low level ] cleared.");
    //         }
    //         assert(clear_cmd_);
    //     }
    //     catch(const std::exception& e)
    //     {
    //         SPDLOG_ERROR(e.what());
    //     }
    }

    void KinovaGen3::SendLowLevelCmd(Eigen::VectorXf joint_torque)
    {
    //     auto servoing_mode = base_ptr_->GetServoingMode();
    //     // std::cout << "Current servoing mode: " << servoing_mode.servoing_mode() << std::endl;
    //     if (servoing_mode.servoing_mode() != k_api::Base::ServoingMode::LOW_LEVEL_SERVOING) {
    //         std::cout << "Not in low level, setting..." << std::endl;
    //         servoing_mode_ptr_->set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    //         base_ptr_->SetServoingMode(*servoing_mode_ptr_);
    //         feed_back_ = base_cyclic_ptr_->RefreshFeedback();
    //     } else {
    //         // std::cout << "Successfully in low level servoing!" << std::endl;
    //     }

    //     Eigen::VectorXf action_grav_comp(Nj_);
    //     // std::cout << "arm_ptr_->q()" << std::endl;
    //     // std::cout << arm_ptr_->q() << std::endl;
    //     CalculateGravComp(q(), action_grav_comp);

    //     for (size_t i = 0; i < Nj_; ++i)
    //     {
    //         // std::cout << "Sending torques " << joint_torque << std::endl;
    //         base_command_.mutable_actuators(i)->set_torque_joint(joint_torque(i));
    //         joint_torques_cmd_(i) = action_grav_comp(i);
    //     }
    }

    void KinovaGen3::SetupLowLevelControl()
    {
    //     try
    //     {
    //         std::cout << "low_level_control " << low_level_control_ << std::endl;

    //         if (in_low_level_) this->ClearLowLevelControl();
    //         assert(!in_low_level_);

    //         SPDLOG_INFO("[ Kinova ] Setting LOW_LEVEL_SERVOING.");

    //         servoing_mode_ptr_->set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    //         base_ptr_->SetServoingMode(*servoing_mode_ptr_);
    //         feed_back_ = base_cyclic_ptr_->RefreshFeedback();

    //         // Initialize each actuator(not including gripper) to their current position
    //         for (int i = 0; i < this->GetJointCount(); ++i)
    //         {
    //             // Save the current actuator position, to avoid a following error
    //             base_command_.add_actuators()->set_position(feed_back_.actuators(i).position());
    //         }
            
    //         // Initialize Gripper
    //         float gripper_initial_position = feed_back_.interconnect().gripper_feedback().motor()[0].position();
    //         // Initialize interconnect command to current gripper position.
    //         base_command_.mutable_interconnect()->mutable_command_id()->set_identifier(0);
    //         gripper_command_ptr_ = base_command_.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
    //         gripper_command_ptr_->set_position(gripper_initial_position);
    //         gripper_command_ptr_->set_velocity(0.0);
    //         gripper_command_ptr_->set_force(50.0);  // force limit

    //         // Send a first frame
    //         base_cyclic_ptr_->Refresh(base_command_);

    //         servoing_mode_ptr_->set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
    //         base_ptr_->SetServoingMode(*servoing_mode_ptr_);
    //         feed_back_ = base_cyclic_ptr_->RefreshFeedback();

    //         // Set all actuators in torque mode
    //         std::cout << "Setting torque control mode!" << std::endl;
    //         auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
    //         control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);
    //         for (int i = 1; i <= Nj_; ++i) {
    //             std::cout << "Setting joint " << i << std::endl;
    //             actuator_config_ptr_->SetControlMode(control_mode_message, i);
    //         }

    //         std::cout << "Successfully initialized torque control mode!" << std::endl;

    //         in_low_level_ = true;

    //         // create new process as a member
    //         // and then launch it
    //         const int max_thread_priority = sched_get_priority_max(SCHED_FIFO);

    //         thread_args_ = new LowLeveLCtrlArgs();
    //         thread_args_->base_cyclic_ptr = base_cyclic_ptr_.get();
    //         thread_args_->base_command_ptr = &base_command_;

    //         pthread_t high_freq_ptid;
    //         pthread_create(&high_freq_ptid, NULL, &SendLowLevelCmdThread, (void *)thread_args_);
    //         struct sched_param params;
    //         params.sched_priority = max_thread_priority;
    //         pthread_setschedparam(high_freq_ptid, SCHED_FIFO, &params);
    //     }
    //     catch(const std::exception& e)
    //     {
    //         SPDLOG_ERROR(e.what());
    //         throw e;
    //     }
        
    }

    void KinovaGen3::ClearLowLevelControl()
    {
    //     try
    //     {
    //         if (in_low_level_)
    //         {
    //             this->ClearLowLevelCmd();
                
    //             SPDLOG_INFO("[ Kinova ] Setting SINGLE_LEVEL_SERVOING.");

    //             servoing_mode_ptr_->set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    //             base_ptr_->SetServoingMode(*servoing_mode_ptr_);
    //             base_command_.clear_actuators();

    //             in_low_level_ = false;
    //         }
    //     }
    //     catch(const std::exception& e)
    //     {
    //         SPDLOG_ERROR(e.what());
    //         throw e;
    //     }

    }


    void KinovaGen3::ExecuteTwistCommand(Eigen::VectorXf twist_vel_cmd, float dt)
    {
    //     try
    //     {
    //         auto command = k_api::Base::TwistCommand();
    //         command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_BASE);
    //         command.set_duration(0);  // Unlimited time to execute

    //         auto twist = command.mutable_twist();
    //         twist->set_linear_x(twist_vel_cmd(0));
    //         twist->set_linear_y(twist_vel_cmd(1));
    //         twist->set_linear_z(twist_vel_cmd(2));
    //         twist->set_angular_x(twist_vel_cmd(3));
    //         twist->set_angular_y(twist_vel_cmd(4));
    //         twist->set_angular_z(twist_vel_cmd(5));
    //         base_ptr_->SendTwistCommand(command);
    //     }
    //     catch(const std::exception& e)
    //     {
    //         SPDLOG_ERROR(e.what());
    //         throw e;
    //     }
    }

}