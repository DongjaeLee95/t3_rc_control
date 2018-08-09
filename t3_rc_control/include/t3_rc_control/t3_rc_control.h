#pragma once
#include <ros/ros.h>
#include <vector>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>

class T3_rc_control
{
public:
    T3_rc_control();
    ~T3_rc_control();

    std::string robotis_wing_id, robotis_body_id;



    // geometry_msgs::Quaternion T3_orientation;
    geometry_msgs::Vector3 T3_orientation_rpy;
    geometry_msgs::Vector3 T3_angular_velocity;
    geometry_msgs::Vector3 T3_linear_acceleration;

    double robotis_wing_angle, robotis_body_angle;
    double robotis_wing_vel, robotis_body_vel;


    void DynamixelPublish(double pos_input);

    //frame_id definition how??

    //SHOULD BE FIXED
    //Robotis_wing id = 0;
    //Robotis_body id = 1;

protected:
    void ImuSensorCallBack(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void DynamixelCallBack(const sensor_msgs::JointState::ConstPtr& robotis_msg);

private:
    ros::Subscriber IMU_sub_;
    ros::Subscriber Dynamixel_sub_;

    ros::Publisher Dynamixel_pub_;

    ros::NodeHandle nh_;
};
