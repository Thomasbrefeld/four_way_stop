#ifndef INTERSECTION_H_
#define INTERSECTION_H_

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>
#include <dbw_polaris_msgs/GearCmd.h>
#include <dataspeed_ulc_msgs/UlcReport.h>
#include <four_way_stop/commanderConfig.h>
#include <dynamic_reconfigure/server.h>
#include <piksi_rtk_msgs/VelNed.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"

#include <queue>

class Intersection{
public:
    Intersection(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
    void run();
private:
    void navCB(const sensor_msgs::NavSatFix::ConstPtr&);
    void velCB(const piksi_rtk_msgs::VelNed::ConstPtr& msg){vel = *msg;};
    void scanCB(const sensor_msgs::LaserScan::ConstPtr&);
    void gearCB(const dbw_polaris_msgs::GearCmd::ConstPtr& msg){gear = *msg;};
    void ulcCB(const dataspeed_ulc_msgs::UlcReport::ConstPtr& msg){ulcReport = *msg;};
    void waypointCmdCB(const geometry_msgs::Twist::ConstPtr& msg){waypointCmd = *msg;};
    void dynamicReconfigureCB(four_way_stop::commanderConfig&, uint32_t);

    double distance(sensor_msgs::NavSatFix, sensor_msgs::NavSatFix);
    double calcHeading(sensor_msgs::NavSatFix, sensor_msgs::NavSatFix);
    void stopFun();

    dynamic_reconfigure::Server<four_way_stop::commanderConfig> server;

    ros::NodeHandle nh;

    ros::Subscriber navSub;
    ros::Subscriber velSub;
    ros::Subscriber scanSub;
    ros::Subscriber gearSub;
    ros::Subscriber ulcSub;
    ros::Subscriber waypointSub;

    ros::Publisher enablePub;
    ros::Publisher disablePub;
    ros::Publisher twistPub;
    ros::Publisher gearPub;
    ros::Publisher waypointPub;

    sensor_msgs::NavSatFix nav;
    sensor_msgs::NavSatFix lastNav;
    sensor_msgs::NavSatFix distNav;
    sensor_msgs::LaserScan scan;
    dbw_polaris_msgs::GearCmd gear;
    dataspeed_ulc_msgs::UlcReport ulcReport;
    piksi_rtk_msgs::VelNed vel;
    geometry_msgs::Twist waypointCmd;

    geometry_msgs::Twist setCmd;
    dbw_polaris_msgs::GearCmd setGear;

    enum Status {initiate, stop, waypoint, lidarWait, turn};

    std::queue<sensor_msgs::NavSatFix> waypoints;

    Status status;
    int turnSequence;
    int sleepCount;

    bool running;

    double rateSleep;
    
    double speed;
    double maxTurn;
    double waypointFoundDistance;
    
    double heading;
    double lastHeading;

    double changeAngle;
    double turningAgnle;

    double maxRange;
    double minRange;
    double maxIndex;
    double minIndex;
};

#endif