#include <ros/ros.h>
#include "t3_rc_control/t3_rc_control.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "t3_rc_control_main");

    T3_rc_control T3RcControl;

    ros::Rate loop_rate(10);

    double pos_input;

    while(ros::ok())
    {
        pos_input += 0.1570796;
        T3RcControl.DynamixelPublish(sin(pos_input));

        ros::spinOnce();

        loop_rate.sleep();
    }
    //ros::spin();

    return 0;
}
