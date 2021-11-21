#include "intersection.h"

Intersection::Intersection(ros::NodeHandle &node, ros::NodeHandle &priv_nh){
    navSub = node.subscribe<sensor_msgs::NavSatFix>("/piksi/navsatfix_best_fix", 1, &Intersection::navCB, this);
    velSub = node.subscribe<piksi_rtk_msgs::VelNed>("/piksi/vel_ned", 1, &Intersection::velCB, this);
    scanSub = node.subscribe<sensor_msgs::LaserScan>("/scan", 1, &Intersection::scanCB, this);
    gearSub = node.subscribe<dbw_polaris_msgs::GearCmd>("/vehicle/gear_cmd", 1, &Intersection::gearCB, this);
    ulcSub = node.subscribe<dataspeed_ulc_msgs::UlcReport>("vehicle/ulc_report", 1, &Intersection::ulcCB, this);
    waypointSub = node.subscribe<geometry_msgs::Twist>("/waypoint/cmd", 1, &Intersection::waypointCmdCB, this);

    enablePub = node.advertise<std_msgs::Empty>("/vehicle/enable", 1);
    disablePub = node.advertise<std_msgs::Empty>("/vehicle/disable", 1);
    twistPub = node.advertise<geometry_msgs::Twist>("/vehicle/cmd_vel", 1);
    gearPub = node.advertise<dbw_polaris_msgs::GearCmd>("/vehicle/gear_cmd", 1);
    waypointPub = node.advertise<sensor_msgs::NavSatFix>("/waypoint/waypoint", 1);

    server.setCallback(boost::bind(&Intersection::dynamicReconfigureCB, this, _1, _2));

    rateSleep = .1;
    speed = 2.5;
    maxTurn = 1.3;
    changeAngle = 0;
    turningAgnle = .1;

    status = initiate;
    running = false;

    sensor_msgs::NavSatFix tmp;
    
    tmp.latitude = 42.47225543898477;
    tmp.longitude = -83.25000730747014;
    waypoints.push(tmp);
    tmp.latitude = 42.47238102165528;
    tmp.longitude = -83.25000858228485;
    waypoints.push(tmp);
    
    waypointFoundDistance = .00003;

    maxRange = 10;
    minRange = 2;
    maxIndex = 30;
    minIndex = -30;
}

void Intersection::dynamicReconfigureCB(four_way_stop::commanderConfig& msg, uint32_t level){
    if (!msg.initiate)
        status = initiate;
    else
        status = waypoint;
    
    speed = msg.speed;
    waypointFoundDistance = msg.waypointFoundDistance;
    maxTurn = msg.maxTurn;
    turningAgnle = msg.turningAgnle;
}

double Intersection::calcHeading(sensor_msgs::NavSatFix nav1, sensor_msgs::NavSatFix nav2){
    double dLon = (nav2.longitude-nav1.longitude);
    double y = sin(dLon) * cos(nav2.latitude);
    double x = cos(nav1.latitude)*sin(nav2.latitude) - sin(nav1.latitude)*cos(nav2.latitude)*cos(dLon);
    double out = abs(atan2(y, x));
    // if (out < 0)
    //     out += M_PI;
    return out;
}

void Intersection::navCB(const sensor_msgs::NavSatFix::ConstPtr& msg){
    if (nav.header.seq != 0)
        lastNav = nav;
    else {
        nav = *msg;
        return;
    }
    nav = *msg;

    if (ulcReport.speed_meas > .1){
        lastHeading = heading;

        heading = calcHeading(nav, lastNav);

        double calcChange = heading - lastHeading;
        if (calcChange > 1.57)
            changeAngle += calcChange - M_PI;
        else if (calcChange < - 1.57)
            changeAngle += calcChange + M_PI;
        else 
            changeAngle += calcChange;

        //ROS_INFO_STREAM("HEADING: " << heading << " LastHeading: " << lastHeading << " calcChange: " << calcChange << " changeAngle: " << changeAngle);
    }
}

void Intersection::scanCB(const sensor_msgs::LaserScan::ConstPtr& msg){
    int count = 0;
    float range;

    //double avg_range = 0;
    //int range_count = 0;

    for (int index; index < msg->ranges.size(); index++){
        float loc = (index * scan.angle_increment - M_PI) * 180 / M_PI;
        float range = scan.ranges.at(index);
        float inten = scan.intensities.at(index);

        if (minIndex < loc && maxIndex > loc){
            if (range > minRange && range < maxRange){
                if (inten > 75){
                    //ROS_INFO_STREAM("loc: " << loc << "range: " << range << "inten: " << inten);
                    //avg_range += range;
                    //range_count++; 
                    count++;
                }
            }
        }
    }

    if (status == waypoint || status == lidarWait){
        if (count > 0)
            status = lidarWait;
        else
            status = waypoint;
    }
}

double Intersection::distance(sensor_msgs::NavSatFix point1, sensor_msgs::NavSatFix point2){
    return sqrt(pow(point1.latitude - point2.latitude, 2) + pow(point1.longitude - point2.longitude, 2));
}

void Intersection::stopFun(){
    setCmd.linear.x = 0;
    
    while (ulcReport.speed_meas != 0){
        ros::spinOnce();
        twistPub.publish(setCmd);
        ros::Duration(rateSleep).sleep();
    }
}

void Intersection::getDirection(){
    float alpha = .05; //lower alpha gives speed less influence
    while (ulcReport.speed_meas == 0){
        ros::spinOnce();
        setCmd.linear.x = (alpha * speed) + (1.0 - alpha) * setCmd.linear.x;
        twistPub.publish(setCmd);
        ros::Duration(rateSleep).sleep();
    }
}

void Intersection::changeGear(dbw_polaris_msgs::GearCmd setGear){
    if (gear == setGear)
        return;

    stopFun();
    gearPub.publish(setGear);

    while (gear != setGear){
        ros::spinOnce();
        twistPub.publish(setCmd);
        ros::Duration(rateSleep).sleep();
    }
    getDirection();
}

void Intersection::run(){
    ros::Duration(2).sleep();
    enablePub.publish(std_msgs::Empty());
    waypointPub.publish(waypoints.front());
    ROS_INFO_STREAM("Four Way Stop Node Started!");

    while (ros::ok()){
        ros::spinOnce();

        if (status != initiate && !running)
            running = true;
        if (status == initiate && running)
            running = false;

        setCmd.linear.x = 0;
        setCmd.linear.y = 0;
        setCmd.linear.z = 0; 
        setCmd.angular.x = 0;
        setCmd.angular.y = 0;
        setCmd.angular.z = 0;

        if (running){
            switch (status){
                case waypoint:
                    setGear.cmd.gear = dbw_polaris_msgs::Gear::DRIVE;
                    changeGear(setGear);
                    if (distance(nav, waypoints.front()) < waypointFoundDistance){
                        status = turn;
                        waypoints.push(waypoints.front());
                        waypoints.pop();
                        waypointPub.publish(waypoints.front());
                    }
                    setCmd = waypointCmd;
                    break;
                case turn:
                    switch (turnSequence){
                        case 1:
                            setGear.cmd.gear = dbw_polaris_msgs::Gear::DRIVE;
                            changeGear(setGear);
                            turnSequence++;
                            changeAngle = 0;
                        case 2:
                            setCmd.linear.x = speed;
                            setCmd.angular.z = maxTurn;

                            if (abs(changeAngle) < .10)
                                break;

                            setGear.cmd.gear = dbw_polaris_msgs::Gear::REVERSE;
                            changeGear(setGear);
                            distNav = nav;
                            turnSequence++;
                        case 3:
                            setCmd.linear.x = -speed;
                            setCmd.angular.z = maxTurn/2;

                            if (distance(nav, distNav) < .00007)
                                break;
                            
                            turnSequence++;
                        case 4:
                            setGear.cmd.gear = dbw_polaris_msgs::Gear::DRIVE;
                            changeGear(setGear);
                            status = waypoint;
                        default:
                            turnSequence = 1;
                    }
                    break;
                case lidarWait:
                    setCmd.linear.x = 0;
                    break;
                case stop:
                    setCmd.linear.x = 0;
                    break;
                default:
                    ROS_INFO_STREAM("status: default");
            }
        }

        ROS_INFO_STREAM(" status: " << status << " CMD: x: " << setCmd.linear.x << " z: " << setCmd.angular.z);
        twistPub.publish(setCmd);
        ros::Duration(rateSleep).sleep();
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "four_way_stop_node");
    ros::NodeHandle node;
    ros::NodeHandle priv_nh("~");

    Intersection intersection(node, priv_nh);
    
    intersection.run();
}



