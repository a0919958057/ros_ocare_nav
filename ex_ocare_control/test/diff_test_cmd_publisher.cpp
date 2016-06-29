#include<ros/ros.h>

// Use the enum of Topic CMD for Diff and Arm
#include"ocare_hw_node.h"

#include"std_msgs/MultiArrayDimension.h"
#include"std_msgs/MultiArrayLayout.h"
#include"std_msgs/UInt16MultiArray.h"

int main(int argc, char** argv) {
    ros::init(argc,argv,"diff_test_cmd_publisher_node");

    // Create a ros node
    ros::NodeHandle node;

    ros::Publisher diff_pub =
            node.advertise<std_msgs::UInt16MultiArray>("diff_mode_cmd",100);

    ros::Rate r(10);


    //cmd_message[DiffTopicCMD::DIFF_MODE_CMD] = DiffModbus::MODE_CONTROLLABLE_CMD;
    //cmd_message[DiffTopicCMD::TRACK_TORQUE_MODE_CMD] = DiffModbus::TORQUE_MED_CMD;


    while(ros::ok()) {

        std_msgs::UInt16MultiArray cmd_message;

        cmd_message.data.clear();

        cmd_message.data.push_back((uint16_t)DiffModbus::MODE_CONTROLLABLE_CMD);
        cmd_message.data.push_back((uint16_t)DiffModbus::TORQUE_MED_CMD);

        diff_pub.publish(cmd_message);

        // Sleep for a while
        r.sleep();
    }



    return 0;
}
