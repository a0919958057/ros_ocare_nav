#include "ocare_hw_node.h"
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>



int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_main_node");
    ros::NodeHandle node;

    OcareRobot robot;
    controller_manager::ControllerManager cm(&robot);

     ros::Rate r(20); // 20 Hz
    while(ros::ok()) {
        robot.update_state_from_robot();
        robot.write_cmd_to_robot();

        ros::spinOnce(); // call all the callbacks waiting to be called at that point in time
        r.sleep();
    }

    return 0;
}
