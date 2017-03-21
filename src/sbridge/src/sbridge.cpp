#include "sbridge.h"

sbridge::sbridge(std::string publishedName) {
    ros::NodeHandle param("~");
    ros::NodeHandle sNH;

    driveControlSubscriber = sNH.subscribe((publishedName + "/driveControl"), 10, &sbridge::cmdHandler, this);

    heartbeatPublisher = sNH.advertise<std_msgs::String>((publishedName + "/sbridge/heartbeat"), 1, false);
    skidsteerPublish = sNH.advertise<geometry_msgs::Twist>((publishedName + "/skidsteer"), 10);
    infoLogPublisher = sNH.advertise<std_msgs::String>("/infoLog", 1, true);

    float heartbeat_publish_interval = 2;
    publish_heartbeat_timer = sNH.createTimer(ros::Duration(heartbeat_publish_interval), &sbridge::publishHeartBeatTimerEventHandler, this);


     ROS_INFO("constructor");

}

void sbridge::cmdHandler(const geometry_msgs::Twist::ConstPtr& message) {
    double linearVel = (message->linear.x);
    double angularVel = (message->angular.z);
    int sat = 255;
    int Kpv = 255;
    int Kpa = 200;

    //Propotinal
    float PV = Kpv * linearVel;
    if (PV > sat) //limit the max and minimum output of proportinal
        PV = sat;
    if (PV < -sat)
        PV= -sat;

    //Propotinal
    float PA = Kpa * angularVel;
    if (PA > sat) //limit the max and minimum output of proportinal
        PA = sat;
    if (PA < -sat)
        PA= -sat;

    float turn = PA/60;
    float forward = PV/355;

    if (forward >= 150){

        forward -= (abs(turn)/5);
    }

    if (linearVel >= 0 && forward <= 0)
    {
        forward = 0;
    }
    if (linearVel <= 0 && forward >= 0)
    {
        forward = 0;
    }
    /*std_msgs::String msg;
   stringstream ss;
   ss << "";
   msg.data = ss.str();
   infoLogPublisher.publish(msg);*/

    velocity.linear.x = forward,
            velocity.angular.z = turn;
    skidsteerPublish.publish(velocity);
}

void sbridge::publishHeartBeatTimerEventHandler(const ros::TimerEvent& event) {
    std_msgs::String msg;
    msg.data = "";
    heartbeatPublisher.publish(msg);

    ROS_INFO("%ds, %dnsec", event.last_real.sec, event.last_real.nsec);
}

sbridge::~sbridge() {
}
