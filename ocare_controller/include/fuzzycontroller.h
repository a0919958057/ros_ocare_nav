#ifndef FUZZYCONTROLLER_H
#define FUZZYCONTROLLER_H

#include<vector>

#include<ros/node_handle.h>
#include<controller_interface/controller.h>
#include<hardware_interface/joint_command_interface.h>
#include<trajectory_msgs/JointTrajectoryPoint.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/JointState.h>
#include<sensor_msgs/Imu.h>

#include<math.h>

#define WHEEL_TORQUE_LIMIT 200.0

namespace ocare_controllers
{
    class FuzzyController : public controller_interface::
            Controller<hardware_interface::EffortJointInterface>
    {
        enum class JointType {
            ARM,
            WHEEL
        };


        // Parameter for IMU topic
        std::string imu_topic;

        // current node
        ros::NodeHandle m_node;

        /* registered robot and joints */
        hardware_interface::EffortJointInterface* m_robot;
        std::vector<hardware_interface::JointHandle> m_joints;

        hardware_interface::JointHandle m_left_wheel;
        hardware_interface::JointHandle m_right_wheel;

        // subcriber for command
        ros::Subscriber m_sub_command;
        // subcriber for diff Twist command
        ros::Subscriber m_sub_diff_command;
        // subcriber for pose_imu
        ros::Subscriber m_pose_imu;

        // Status of robot;
        double m_roll;
        double m_pitch;
        double m_yaw;

        // command of robot
        double m_cmd_yaw;
        double m_cmd_vel;



        void command_callback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &referencePoint);
        void command_twist_callback(const geometry_msgs::Twist::ConstPtr &referenceTwist);
        void callback_imu(const sensor_msgs::Imu::ConstPtr& msg);

        static double cot_angle(double _degree);

    public:
        FuzzyController();
        ~FuzzyController();

        bool init(hardware_interface::EffortJointInterface *,ros::NodeHandle &);
        void starting(const ros::Time& time);
        void update(const ros::Time& time,const ros::Duration& duration);

        bool read_parameter(JointType);
    };
}

#endif // FUZZYCONTROLLER_H
