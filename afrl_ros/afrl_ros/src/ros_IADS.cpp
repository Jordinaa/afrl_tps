/*Sends message from ROS to IADS*/
#include "ros/ros.h"
#include <IADSInterface.h>
#include <uas_info.h>
// https://mavsdk.mavlink.io/main/en/cpp/api_reference/ for existing telem data for IADS
using namespace std;

// enum
// {
//     IADS__TEST_INPUT
//     , IADS__TEST_INPUT2
//     , IADS__TEST_INPUT3
//     , IADS__TEST_INPUT4
//     , IADS__TEST_INPUT5
//     , IADS__TEST_INPUT6
//     , IADS__TOTAL
// };

enum
{
    IADS__AIRSPEED,
    IADS__ALTITUDE,
    IADS__LATITUDE,
    IADS__LONGITUDE,
    IADS__ROLL_ANGLE,
    IADS__PITCH_ANGLE,
    IADS__HEADING,
    IADS__ROLL_RATE,
    IADS__PITCH_RATE,
    IADS__YAW_RATE,
    IADS__X_ACCEL,
    IADS__Y_ACCEL,
    IADS__Z_ACCEL,
    IADS__AILERON,
    IADS__ELEVATOR,
    IADS__RUDDER,
    IADS__THROTTLE,
    IADS__FLAPS,
    IADS__GROUNDSPEED,
    IADS__LCL_POSN_X,
    IADS__LCL_POSN_Y,
    IADS__LCL_POSN_Z,
    IADS__LCL_VEL_X,
    IADS__LCL_VEL_Y,
    IADS__LCL_VEL_Z,
    IADS__CLIMB_RATE,
    IADS__ATTITUDE_CMD_Q1,
    IADS__ATTITUDE_CMD_Q2,
    IADS__ATTITUDE_CMD_Q3,
    IADS__ATTITUDE_CMD_Q4,
    IADS__ROLL_RATE_CMD,
    IADS__PITCH_RATE_CMD,
    IADS__YAW_RATE_CMD,
    IADS__TOTAL = 33
};



//implement iads protocol
//need to set this ip of local host
// IADSInterface iads( "192.168.231.110", 1500, IADS__TOTAL);
IADSInterface iads( "10.3.21.10", 1500, IADS__TOTAL);

int main(int argc, char **argv) 
{
    ros::init(argc,argv,"test_user");
    ros::NodeHandle nh;
    
    ros:: Rate rate(100);

    string ip_address; 
    //string ip = nh.getParam("iads_broker_ip", ip_address);
    cout<<"ip is " << nh.getParam("iads_broker_ip", ip_address) << endl;
    //instantiate UASInfo object 
    UASInfo uas_info(&nh);

    while (ros::ok())
    {
        cout<<"uas rpy deg is " << uas_info.attitude_deg << endl;
        iads.setParameter(IADS__ROLL_RATE, uas_info.attitude_rate_deg[0])
            .setParameter(IADS__PITCH_RATE, uas_info.attitude_rate_deg[1])
            .setParameter(IADS__YAW_RATE, uas_info.attitude_rate_deg[2])
            .setParameter(IADS__ROLL, uas_info.attitude_deg[0])
            .setParameter(IADS__PITCH, uas_info.attitude_deg[1])
            .setParameter(IADS__HEADING, uas_info.vfr[0])
            .setParameter(IADS__AIRSPEED, uas_info.vfr[1])
            .setParameter(IADS__ALTITUDE, uas_info.vfr[2])
        .sendData();
        // cout<<"sending message"<<endl;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


