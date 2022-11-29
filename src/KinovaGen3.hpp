#pragma once

#include "ArmBase.hpp"

// namespace k_api = Kinova::Api;

namespace rc
{
    #define JOINT_COUNT 7

    typedef struct Notification {

        Notification(){ active_ = false; }

        bool active_;
        // k_api::Common::NotificationHandle handle_;

    } Notification_t;

    struct LowLeveLCtrlArgs {
        // k_api::BaseCyclic::BaseCyclicClient* base_cyclic_ptr;
        // k_api::BaseCyclic::Command* base_command_ptr;
        int xx;
    };

    class KinovaGen3 : public ArmBase {
        public:

            KinovaGen3(const std::string& ip_addr, const uint32_t& port, const uint32_t& port_realtime,
                        const std::string& username, const std::string& passwd, 
                        const bool& low_level_control);
            ~KinovaGen3();

            /* -------------------------------------------------------------------------- */
            /*                        High level cartesian movement                       */
            /* -------------------------------------------------------------------------- */
            void ExecuteCartesianAction(Eigen::Affine3f pose);
            bool CartesianActionEnded();
            
            /* -------------------------------------------------------------------------- */
            /*                          High level joint movement                         */
            /* -------------------------------------------------------------------------- */
            void ExecuteJointAction(Eigen::VectorXf joint_acc_cmd, float dt);
            bool JointActionEnded();
            void SetupLowLevelControl();
            void ClearLowLevelControl();

            /* -------------------------------------------------------------------------- */
            /*                                Twist command                               */
            /* -------------------------------------------------------------------------- */
            void ExecuteTwistCommand(Eigen::VectorXf twist_vel_cmd, float dt);


            /* -------------------------------------------------------------------------- */
            /*                                Status report                               */
            /* -------------------------------------------------------------------------- */
            void UpdateFeedback();

            /* -------------------------------------------------------------------------- */
            /*                                   Gripper                                  */
            /* -------------------------------------------------------------------------- */
            void OpenGripper();
            void CloseGripper();
            void GripperLowLevelHelper(bool is_open);

            /* -------------------------------------------------------------------------- */
            /*                                  Shutdown                                  */
            /* -------------------------------------------------------------------------- */
            void KeepPosition();
            void Stop();

        private:

            /* -------------------------------------------------------------------------- */
            /*                           Communication Interface                          */
            /* -------------------------------------------------------------------------- */

            const std::string ip_addr_, username_, passwd_;
            const uint32_t port_, port_realtime_;
            // std::shared_ptr<k_api::TransportClientTcp> transport_ptr_;
            // std::shared_ptr<k_api::TransportClientUdp> transport_realtime_ptr_;
            // std::shared_ptr<k_api::RouterClient> router_ptr_;
            // std::shared_ptr<k_api::RouterClient> router_realtime_ptr_;
            // std::shared_ptr<k_api::Session::CreateSessionInfo> session_info_ptr_;
            // std::shared_ptr<k_api::SessionManager> session_manager_ptr_;
            // std::shared_ptr<k_api::SessionManager> session_manager_realtime_ptr_;
            // std::shared_ptr<k_api::Base::BaseClient> base_ptr_;
            // // Cyclic
            // std::shared_ptr<k_api::BaseCyclic::BaseCyclicClient> base_cyclic_ptr_;
            // // Pose control
            // std::shared_ptr<k_api::Base::Action> action_ptr_;
            // // Velocity control
            // std::shared_ptr<k_api::Base::TwistCommand> twist_cmd_ptr_;
            // // Cartesian following controjoint_count_l
            // std::shared_ptr<k_api::Base::ServoingModeInformation> servoing_mode_ptr_;
            // // Torque Control
            // std::shared_ptr<k_api::ActuatorConfig::ActuatorConfigClient> actuator_config_ptr_;

            /* -------------------------------------------------------------------------- */
            /*                            Command and Feedback                            */
            /* -------------------------------------------------------------------------- */

            bool action_finished_by_notification_, action_abort_by_notification_;
            Notification_t action_notification_;
            // k_api::BaseCyclic::Command base_command_;
            // k_api::BaseCyclic::Feedback feed_back_;
            // k_api::BaseCyclic::BaseFeedback base_feedback_;
            // k_api::GripperCyclic::MotorCommand* gripper_command_ptr_;

            // std::atomic_bool clear_cmd_;
            // std::thread joint_vel_thread_;

            // std::thread delayed_cmd_thread_;

            // std::mutex feedback_mutex_;

            // k_api::Base::JointSpeeds zero_speeds_;

            LowLeveLCtrlArgs* thread_args_;

        private:

            /* -------------------------------------------------------------------------- */
            /*                            Arm specific methods                            */
            /* -------------------------------------------------------------------------- */
            void InitCommunication();
            void ClearActionNotification();
            void SetupActionNotification();

            void ClearLowLevelCmd();
            void SendLowLevelCmd(Eigen::VectorXf joint_acc_cmd);
            // void SendLowLevelCmdThread(void* args);

            void ClearDelayedCmd();
            // void SendDelayedCmd(const long& execution_time_ms, k_api::Base::JointSpeeds joint_speeds);
    };
}