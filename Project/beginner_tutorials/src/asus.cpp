#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
ROS_INFO("Angle [%f] and distance [%f]", msg->angle_min, msg->ranges[0]);
}

int main(int argc, char **argv){
ros::init(argc,argv,"listener");
ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("chatter", 100, callback);

ros::spin();
return 0;
}
