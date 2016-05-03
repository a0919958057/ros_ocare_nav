#ifndef FUZZYCONTROLLER_H
#define FUZZYCONTROLLER_H

#include<vector>

#include<ros/node_handle.h>
#include<controller_interface/controller.h>
#include<hardware_interface/joint_command_interface.h>
#include<trajectory_msgs/JointTrajectoryPoint.h>
#include<sensor_msgs/JointState.h>

namespace ocare_controllers
{
    class FuzzyController : public controller_interface::
            Controller<hardware_interface::EffortJointInterface>
    {
        ros::NodeHandle m_node;

        hardware_interface::EffortJointInterface* m_robot;
        std::vector<hardware_interface::JointHandle> m_joints;

        ros::Subscriber m_sub_command;

        void command_callback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr &referencePoint);

    public:
        FuzzyController();
        ~FuzzyController();

        bool init(hardware_interface::EffortJointInterface *,ros::NodeHandle &);
        void starting(const ros::Time& time);
        void update(const ros::Time& time,const ros::Duration& duration);
    };
}

#endif // FUZZYCONTROLLER_H
