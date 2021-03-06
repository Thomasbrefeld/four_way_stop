#include "intersection.h"

Intersection::Intersection(ros::NodeHandle &node, ros::NodeHandle &priv_nh){
    navSub = node.subscribe<sensor_msgs::NavSatFix>("/piksi/navsatfix_best_fix", 1, &Intersection::navCB, this);
    scanSub = node.subscribe<sensor_msgs::LaserScan>("/scan", 1, &Intersection::scanCB, this);
    gearSub = node.subscribe<dbw_polaris_msgs::GearCmd>("/vehicle/gear_cmd", 1, &Intersection::gearCB, this);
    ulcSub = node.subscribe<dataspeed_ulc_msgs::UlcReport>("vehicle/ulc_report", 1, &Intersection::ulcCB, this);
    steeringSub = node.subscribe<dbw_polaris_msgs::SteeringReport>("/vehicle/steering_report", 1, &Intersection::steeringCB, this);
    waypointSub = node.subscribe<geometry_msgs::Twist>("/waypoint/cmd", 1, &Intersection::waypointCmdCB, this);

    enablePub = node.advertise<std_msgs::Empty>("/vehicle/enable", 1);
    twistPub = node.advertise<geometry_msgs::Twist>("/vehicle/cmd_vel", 1);
    gearPub = node.advertise<dbw_polaris_msgs::GearCmd>("/vehicle/gear_cmd", 1);
    waypointPub = node.advertise<sensor_msgs::NavSatFix>("/waypoint/waypoint", 1);

    server.setCallback(boost::bind(&Intersection::dynamicReconfigureCB, this, _1, _2));

    //variables
    status = initiate;
    rateSleep = .1;
    speed = 2.5;
    maxTurn = 1.3;
    stopCount = 20;
    changeAngle = 0;
    turningAgnle = 25;
    turnSpeedMulti = 1.2;
    minForwardSpeed = .5;
    reverseDistance = .00007;
    waypointFoundDistance = .00003;
    minLidarStopCount = 0;
    clearToMove = 0;

    //waypoints
    sensor_msgs::NavSatFix tmp;
    tmp.latitude = 42.47607849737354;
    tmp.longitude = -83.24802861225244;
    waypoints.push(tmp);
    tmp.latitude = 42.47584758130652;
    tmp.longitude = -83.24767034306423;
    waypoints.push(tmp);
    /*
    sensor_msgs::NavSatFix tmp;
    tmp.latitude = 42.47227875741365;
    tmp.longitude = -83.25001776433625;
    waypoints.push(tmp);
    tmp.latitude = 42.472405873261486;
    tmp.longitude = -83.250016340915;
    waypoints.push(tmp);
    */
    
    //lidar box
    maxRange = 10;
    minRange = 2;
    maxIndex = 10;
    minIndex = -30;
}

void Intersection::dynamicReconfigureCB(four_way_stop::commanderConfig& msg, uint32_t level){
    if (!msg.initiate)
        status = initiate;
    else
        status = waypoint;
    
    speed = msg.speed;
    rateSleep = msg.rateSleep;
    waypointFoundDistance = msg.waypointFoundDistance;
    maxTurn = msg.maxTurn;
    turningAgnle = msg.turningAgnle;
    turnSpeedMulti = msg.turnSpeedMulti;
    minForwardSpeed = msg.minForwardSpeed;
    reverseDistance = msg.reverseDistance;
    stopCount = msg.stopCount;
    minLidarStopCount = msg.minLidarStopCount;

    maxRange = msg.maxRange;
    minRange = msg.minRange;
    maxIndex = msg.maxIndex;
    minIndex = msg.minIndex;
}

double Intersection::calcHeading(sensor_msgs::NavSatFix nav1, sensor_msgs::NavSatFix nav2){
    double dLon = (nav2.longitude-nav1.longitude);
    double y = sin(dLon) * cos(nav2.latitude);
    double x = cos(nav1.latitude)*sin(nav2.latitude) - sin(nav1.latitude)*cos(nav2.latitude)*cos(dLon);
    double out = abs(atan2(y, x));
    return out;
}

void Intersection::steeringCB(const dbw_polaris_msgs::SteeringReport::ConstPtr& msg){
    if (steer.header.seq != 0)
        lastSteer = steer;
    else {
        steer = *msg;
        return;
    }
    steer = *msg;

    double timeChange = (steer.header.stamp - lastSteer.header.stamp).toSec();
    changeAngle += timeChange * (steer.steering_wheel_angle * (ulcReport.speed_meas * .5));
    //ROS_INFO_STREAM(changeAngle << " - " << timeChange * (steer.steering_wheel_angle * ulcReport.speed_meas));
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
    }
}

void Intersection::scanCB(const sensor_msgs::LaserScan::ConstPtr& msg){
    if (status == initiate || status == turn)
        return;
    int count = 0;
    float range;
    
    for (int index = 0; index < msg->ranges.size(); index++){

        float loc = (index * msg->angle_increment - M_PI) * 180 / M_PI;
        float range = msg->ranges.at(index);
        float inten = msg->intensities.at(index);
        
        if (minIndex < loc && maxIndex > loc){
            if (range > minRange && range < maxRange){
                if (inten > 25){
                    ROS_INFO_STREAM(loc);
                    count++;
                    clearToMove = 0;
                }
            }
        }
    }
    if (clearToMove < 5){
        status = lidarWait;
        clearToMove++;
    }
    else{
        //ROS_INFO_STREAM("ELSE");
        if (status == waypoint || status == lidarWait){
            //ROS_INFO_STREAM("Count: " << count);
            if (count > minLidarStopCount){
                //ROS_INFO_STREAM("LIDAR WAIT");
                status = lidarWait;
            }
            else{
                //ROS_INFO_STREAM("WAYPOIUNT");
                status = waypoint;
            }
        }
    }
    //ROS_INFO_STREAM("STATUS ++++ " << status);
}

double Intersection::distance(sensor_msgs::NavSatFix point1, sensor_msgs::NavSatFix point2){
    return sqrt(pow(point1.latitude - point2.latitude, 2) + pow(point1.longitude - point2.longitude, 2));
}

void Intersection::stopFun(){
    setCmd.linear.x = 0;
    setCmd.angular.z = 0;
    
    while (ulcReport.speed_meas != 0){
        ros::spinOnce();
        twistPub.publish(setCmd);
        ros::Duration(rateSleep).sleep();
    }
    for (int i = 0; i < stopCount; ++i){
        ros::spinOnce();
        twistPub.publish(setCmd);
        ros::Duration(rateSleep).sleep();
    }

}

void Intersection::run(){
    ros::Duration(1).sleep();
    enablePub.publish(std_msgs::Empty());
    waypointPub.publish(waypoints.front());
    ROS_INFO_STREAM("Four Way Stop Node Started!");

    while (ros::ok()){
        ros::spinOnce();

        setCmd.linear.x = 0;
        setCmd.linear.y = 0;
        setCmd.linear.z = 0; 
        setCmd.angular.x = 0;
        setCmd.angular.y = 0;
        setCmd.angular.z = 0;

        if (status != initiate){
            switch (status){
                case waypoint:
                    setGear.cmd.gear = dbw_polaris_msgs::Gear::DRIVE;
                    gearPub.publish(setGear);
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
                            //ROS_INFO_STREAM("TurnSequence " << turnSequence);
                            changeAngle = 0;
                            turnSequence++;
                        case 2:
                            //ROS_INFO_STREAM("TurnSequence " << turnSequence);
                            setCmd.linear.x = speed * turnSpeedMulti;
                            setCmd.angular.z = maxTurn;

                            ROS_INFO_STREAM("ChangeAngle: " << changeAngle << " ABS(GhangeAngle) " << abs(changeAngle) << " turning angle: " << turningAgnle);
                            if (abs(changeAngle) < turningAgnle)
                                break;


                            distNav = nav;
                            stopFun();
                            setGear.cmd.gear = dbw_polaris_msgs::Gear::REVERSE;
                            gearPub.publish(setGear);
                            stopFun();
                            turnSequence++;
                        case 3: 
                            //ROS_INFO_STREAM("TurnSequence " << turnSequence);
                            setCmd.linear.x = -speed * turnSpeedMulti;
                            setCmd.angular.z = 0;
                            changeAngle = 0;

                            ROS_INFO_STREAM("ulcReport.speed_meas: " << ulcReport.speed_meas << " minForwardSpeed: " << -minForwardSpeed );
                            if (ulcReport.speed_meas > -minForwardSpeed)
                                break;

                            turnSequence++;

                        case 4:
                            //ROS_INFO_STREAM("TurnSequence " << turnSequence);
                            setCmd.linear.x = -speed * .9;
                            setCmd.angular.z = maxTurn * .75;

                            //ROS_INFO_STREAM("ChangeAngle: " << changeAngle << " ABS(GhangeAngle) " << abs(changeAngle) << " turning angle: " << turningAgnle);
                            if (abs(changeAngle) < reverseDistance)
                                break;
                            
                            stopFun();
                            setGear.cmd.gear = dbw_polaris_msgs::Gear::DRIVE;
                            gearPub.publish(setGear);
                            stopFun();
                            turnSequence++;
                        case 5:
                            //ROS_INFO_STREAM("TurnSequence " << turnSequence);
                            setCmd.linear.x = speed;
                            setCmd.angular.z = 0;

                            if (ulcReport.speed_meas < minForwardSpeed)
                                break;

                            status = waypoint;
                        default:
                            turnSequence = 1;
                    }
                    break;
                case lidarWait:
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
