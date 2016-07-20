#include "ocare_hw_node.h"
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <urdf/model.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_main_node");
    ros::NodeHandle node("ocare");
    ros::NodeHandle node2;



    ROS_INFO("Successful robot_main_node");
    OcareRobot robot;
    ROS_INFO("Successful initial the OcareRobot");
    robot.init(&node2);

    controller_manager::ControllerManager cm(&robot, node);
    ROS_INFO("Successful create ControllerManager");


    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_INFO("Create spinner");


     ros::Rate r(1.0 / robot.getPeriod().toSec());


    while(ros::ok()) {

        ros::Time now = robot.getTime();
        ros::Duration dt = robot.getPeriod();
        robot.read(now, dt);

        cm.update(now, dt);

        robot.write(now, dt);
        r.sleep();

    }
    spinner.stop();
    return 0;
}
