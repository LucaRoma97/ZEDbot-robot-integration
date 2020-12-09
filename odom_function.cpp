#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#define PI 3.14

long _PreviousLeftEncoderCounts = 0;
long _PreviousRightEncoderCounts = 0;
ros::Time current_time_encoder, last_time_encoder;
const int ticks_rotation {20};
const float lengthBetweenTwoWheels = 0.132;
const float radius = 0.0318;
const float DistancePerCount = (PI * 2) / ticks_rotation; // from ticks to radians

// publishers
ros::Publisher odom_pub ;   
ros::Publisher yaw_pub ;
ros::Publisher leftright_pub ;

double x;
double y;
double th;

double vx;
double vy;
double vth;
double omega_left;
double omega_right;
double deltaLeft;
double deltaRight;
double v_left;
double v_right;
//double tolerance {1.05};

void WheelCallback(const geometry_msgs::Vector3::ConstPtr& ticks)
{
   geometry_msgs::Vector3 robot;
    
   current_time_encoder = ros::Time::now();
   double dt = (current_time_encoder - last_time_encoder).toSec();
   deltaLeft = ticks->x ;//- _PreviousLeftEncoderCounts;
   deltaRight = ticks->y ;//- _PreviousRightEncoderCounts;

   

   omega_left = (deltaLeft * DistancePerCount) / (current_time_encoder - last_time_encoder).toSec(); // radians per second
   omega_right = (deltaRight * DistancePerCount) / (current_time_encoder - last_time_encoder).toSec();

   v_left = omega_left * radius;
   v_right = omega_right * radius;

   ROS_INFO("left [%f], right [%f]", v_left, v_right);

   vx = ((v_right + v_left) / 2);
   vy = 0;
   vth = ((v_right - v_left)/lengthBetweenTwoWheels);
  
   robot.x = v_left;
   robot.y = v_right;
   
   last_time_encoder = current_time_encoder;
   leftright_pub.publish(robot);

 // _PreviousLeftEncoderCounts = ticks->x;
  //_PreviousRightEncoderCounts = ticks->y;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);   
  yaw_pub = n.advertise<std_msgs::Float64>("yaw_angle", 50);
  leftright_pub = n.advertise<geometry_msgs::Vector3>("left_right", 50);
  ros::Subscriber sub = n.subscribe("wheel_encoder", 100, WheelCallback);
  tf::TransformBroadcaster odom_broadcaster;


  ros::Time current_time, last_time;
  current_time = ros::Time::now(); // try to get rid of this
  last_time = ros::Time::now(); // try to get rid of this


  ros::Rate r(2);  //0.2
  while(n.ok()){

    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    //ROS_INFO("left [%f], right [%f]", dt, vx); 

    x += delta_x;
    y += delta_y;
    th += delta_th;

    // copy theta to yaw
    std_msgs::Float64 yaw;
    yaw.data = th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "chassis";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "chassis";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);
    yaw_pub.publish(yaw);

    last_time = current_time;
    ros::spinOnce(); 
    r.sleep();
  }
}
