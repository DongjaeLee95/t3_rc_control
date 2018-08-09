#include "t3_rc_control/t3_rc_control.h"

T3_rc_control::T3_rc_control():nh_("~")
{
    ROS_INFO("T3_rc_control Initiated");
    IMU_sub_ = nh_.subscribe("/imu/data", 10, &T3_rc_control::ImuSensorCallBack, this);
    Dynamixel_sub_ = nh_.subscribe("/joint_states",10, &T3_rc_control::DynamixelCallBack, this);

    nh_.getParam("Robotis_wing_id", robotis_wing_id);
    nh_.getParam("Robotis_body_id", robotis_body_id);
//    nh_.getParam("turtle1_name", tracker_name);
//    nh_.getParam("turtle2_name", target_name);

}

T3_rc_control::~T3_rc_control()
{
    ros::shutdown();
}

void T3_rc_control::ImuSensorCallBack(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    geometry_msgs::Quaternion q = imu_msg->orientation;
    double roll, pitch, yaw;

    roll = std::atan2(2*(q.w*q.x + q.y*q.z),1-2*(q.x*q.x + q.y*q.y));
    pitch = std::asin(2*(q.w*q.y - q.z*q.x));
    yaw = std::atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z));

    T3_orientation_rpy.x = roll;
    T3_orientation_rpy.y = pitch;
    T3_orientation_rpy.z = yaw;

    T3_angular_velocity = imu_msg->angular_velocity;
    T3_linear_acceleration = imu_msg->linear_acceleration;
}

void T3_rc_control::DynamixelCallBack(const sensor_msgs::JointState::ConstPtr &robotis_msg)
{
    std::vector<std::string> model_ids = robotis_msg->name;
    std::vector<double> angle_array = robotis_msg->position;
    std::vector<double> velocity_array = robotis_msg->velocity;


    int wing_idx = (int) (std::find(model_ids.begin(),model_ids.end(),this->robotis_wing_id) - model_ids.begin());
    int body_idx = (int) (std::find(model_ids.begin(),model_ids.end(),this->robotis_body_id) - model_ids.begin());
    if (wing_idx < model_ids.size()) {
        this->robotis_wing_angle = angle_array[wing_idx]; //current robotis_wing_angle position
        this->robotis_wing_vel = velocity_array[wing_idx]; //current robotis_wing_angle position
        //ROS_INFO("tracker_idx: %d", tracker_idx);
        //   this->tracker_twist = twist_vector[tracker_idx];
    }
    else
        ROS_WARN_ONCE("specified robotis_wing_id was not found in Dynamixel");

    if (body_idx<model_ids.size()) {
        this->robotis_body_angle = angle_array[body_idx]; //current robotis_wing_angle position
        this->robotis_body_vel = velocity_array[body_idx]; //current robotis_wing_angle position
        //ROS_INFO("target_idx: %d", target_idx);
        //   this->target_twist = twist_vector[target_idx];
    }
    else
        ROS_WARN_ONCE("specified robotis_body_id was not found in Dynamixel");
}

void T3_rc_control::DynamixelPublish(double pos_input)
{
    Dynamixel_pub_ = nh_.advertise<sensor_msgs::JointState>("/goal_dynamixel_position",10);
    // Dynamixel_pub_.publish(T3_rc_control::controller());
    // only position(angle) is needed to be published since this is position control.
    // name or velocity are no need to be published

    // TEST

    sensor_msgs::JointState published_msg;
    published_msg.name.push_back(robotis_wing_id);
    published_msg.name.push_back(robotis_body_id);

    //published_msg.position.push_back(robotis_wing_angle);
    published_msg.position.push_back(pos_input);
    published_msg.position.push_back(robotis_body_angle);

    published_msg.velocity.push_back(robotis_wing_vel);
    published_msg.velocity.push_back(robotis_body_vel);

    Dynamixel_pub_.publish(published_msg);


     //T3_rc_control::controller() : should be designed to publish goal position
}