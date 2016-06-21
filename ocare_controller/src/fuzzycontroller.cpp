#include <sys/mman.h>
#include "fuzzycontroller.h"
#include<tf/tf.h>
#include<angles/angles.h>
#include <pluginlib/class_list_macros.h>



ocare_controllers::FuzzyController::FuzzyController() :
    m_roll(0.0), m_pitch(0.0), m_yaw(0.0), m_cmd_vel(0.0), m_cmd_yaw(0.0) {
}

ocare_controllers::FuzzyController::~FuzzyController() {

}

void ocare_controllers::FuzzyController::command_callback(
        const trajectory_msgs::JointTrajectoryPoint::ConstPtr &referencePoint) {
    // TODO: get TrajectoryPoint and set the value to Class
    const double *pos_ = referencePoint->positions.data();
}

void ocare_controllers::FuzzyController::command_twist_callback(
        const geometry_msgs::Twist::ConstPtr &referenceTwist) {

    // Set the reference Z axis angle
    m_cmd_yaw = referenceTwist->angular.z;
    m_cmd_vel = referenceTwist->linear.x;

}

bool ocare_controllers::FuzzyController::init(
        hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n) {

    // Read the topic name from Parameter
    if(!m_node.hasParam("imu_topic"))
        ROS_ERROR("Usage : rosrun robot_controller robot_controller_node _imu_topic:=<topic>");
    m_node.param<std::string>("imu_topic",imu_topic,"/imu");

     // Register node for parent and robot handle
    m_node = n;
    m_robot = robot;

    // read the parameter at parameter server and registe to class
    /****If you need to use arm , you have to change the m_robot from Effort to Position****/
    if(!read_parameter(JointType::WHEEL)) return false;
    //if(!read_parameter(JointType::ARM)) return false;



    // setup the subscribe for command
    m_sub_command = m_node.subscribe("command", 1000, &FuzzyController::command_callback, this);

    // setup the subscribe for imu data
    m_pose_imu = m_node.subscribe(imu_topic,100,&FuzzyController::callback_imu, this);

    std::string robot_desc_string;
    if(!m_node.getParam("/robot_description",robot_desc_string))
            {
        ROS_ERROR("Could not find '/robot_description'.");
        return false;
    }

    return true;
}

void ocare_controllers::FuzzyController::starting(const ros::Time &time) {
    struct sched_param param;
    param.sched_priority=sched_get_priority_max(SCHED_FIFO);
    if(sched_setscheduler(0,SCHED_FIFO,&param) == -1)
    {
            ROS_WARN("Failed to set real-time scheduler.");
            return;
    }
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
            ROS_WARN("Failed to lock memory.");
}

void ocare_controllers::FuzzyController::update(const ros::Time &time, const ros::Duration &duration) {

    double err = m_cmd_yaw - m_yaw;
    err = cot_angle(err);

    double left_torque_     = err * 100     + m_cmd_vel;
    double right_torque_    = err * (-100)  + m_cmd_vel;

    if (left_torque_ > WHEEL_TORQUE_LIMIT) {
        left_torque_ = WHEEL_TORQUE_LIMIT;
    }
    if (right_torque_ > WHEEL_TORQUE_LIMIT) {
        right_torque_ = WHEEL_TORQUE_LIMIT;
    }

    if (left_torque_ < -WHEEL_TORQUE_LIMIT) {
        left_torque_ = -WHEEL_TORQUE_LIMIT;
    }
    if (right_torque_ < -WHEEL_TORQUE_LIMIT) {
        right_torque_ = -WHEEL_TORQUE_LIMIT;
    }

    m_left_wheel.setCommand(left_torque_);
    m_left_wheel.setCommand(right_torque_);


   return;
}

void ocare_controllers::FuzzyController::callback_imu(const sensor_msgs::Imu::ConstPtr& msg) {

    //ROS_INFO("Imu Seq: [%d]", msg->header.seq);

    tf::Matrix3x3 m(tf::Quaternion(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w));
    double roll, pitch, yaw;
    m.getRPY(roll,pitch,yaw);

    m_roll = angles::to_degrees(roll);
    m_pitch = angles::to_degrees(pitch);
    m_yaw = angles::to_degrees(yaw);

    //ROS_INFO("Imu Orientation x:[%f], y:[%f], z:[%f]", roll, pitch, yaw);
}

bool ocare_controllers::FuzzyController::read_parameter(JointType _type) {

    if(_type == JointType::ARM) {

        XmlRpc::XmlRpcValue joint_names;
        if(!m_node.getParam("arms",joint_names)) {
            ROS_ERROR("No 'joints' in controller. (namespace: %s)",
                    m_node.getNamespace().c_str());
            return false;
        }

        if(joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("'joints' is not a struct. (namespace: %s)",
                    m_node.getNamespace().c_str());
            return false;
        }

        for(int i=0; i < joint_names.size();i++)
        {
            XmlRpc::XmlRpcValue &name_value=joint_names[i];
            if(name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
            {
                ROS_ERROR("joints are not strings. (namespace: %s)",
                        m_node.getNamespace().c_str());
                return false;
            }

            ROS_INFO("Find \"%s\" joint !", ((std::string)name_value).data() );
            hardware_interface::JointHandle j=m_robot->
                    getHandle((std::string)name_value);
            m_joints.push_back(j);
        }
    } else if (_type == JointType::WHEEL) {

        XmlRpc::XmlRpcValue joint_names;
        if(!m_node.getParam("wheels",joint_names)) {
            ROS_ERROR("No 'wheel joints' in controller. (namespace: %s)",
                    m_node.getNamespace().c_str());
            return false;
        }

        if(joint_names.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("'wheel joints' is not a struct. (namespace: %s)",
                    m_node.getNamespace().c_str());
            return false;
        }


        XmlRpc::XmlRpcValue name_value;
        name_value = joint_names[0];
        if(name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            ROS_ERROR("joints are not strings. (namespace: %s)",
                    m_node.getNamespace().c_str());
            return false;
        }

        ROS_INFO("Find \"%s\" wheel joints !", ((std::string)name_value).data() );
        m_left_wheel=m_robot->
                getHandle((std::string)name_value);

        name_value = joint_names[1];
        if(name_value.getType() != XmlRpc::XmlRpcValue::TypeString)
        {
            ROS_ERROR("wheel joints are not strings. (namespace: %s)",
                    m_node.getNamespace().c_str());
            return false;
        }

        ROS_INFO("Find \"%s\" wheel joints !", ((std::string)name_value).data() );
        m_right_wheel=m_robot->
                getHandle((std::string)name_value);
    }


    return true;
}

double ocare_controllers::FuzzyController::cot_angle(double _degree) {

    if( _degree < 0.0) {
        double times_ = fabs(_degree) / (2*M_PI);
        return ( fmod( _degree + ( 2*M_PI * (floor(times_)+1) ) + M_PI  ,2*M_PI) - M_PI);
    } else if ( _degree > 0.0) {
        return ( fmod( _degree + M_PI ,2*M_PI) - M_PI);
    } else {
        return 0.0;
    }

}

PLUGINLIB_EXPORT_CLASS(ocare_controllers::FuzzyController,
        controller_interface::ControllerBase)
