#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <math.h> 

ros::Publisher Velocities_pub;
geometry_msgs::Vector3 velocity;

double leftvel, rightvel;
double errorr,errorl;
double iPartl{100}, pPartl;
double iPartr{100}, pPartr;
const int Kpl {2};
const int Kil {3};
const int Kpr {2};
const int Kir {3};
const int maxVel{255};
//float left_desvel{0.45};
//float right_desvel{0.45};
float right_desvel, left_desvel;


void PubVelocity()
{
    iPartl += Kil*errorl;
   
    pPartl = Kpl*errorl;

    iPartr += Kir*errorr;

    pPartr = Kpr*errorr; //put to 0 if .... see the reference

    velocity.x = pPartl + iPartl;

    velocity.y = pPartr + iPartr;

    if (velocity.x > maxVel)

        velocity.x = maxVel;

    if (velocity.y > maxVel)

        velocity.y = maxVel;

    ROS_INFO("left [%f], right [%f]", velocity.x, velocity.y);

    Velocities_pub.publish(velocity);

}

void wheelCallback(const geometry_msgs::Vector3& odom)
{

   leftvel = odom.x;

   rightvel = odom.y;

   errorl = - leftvel + left_desvel;

   errorr = - rightvel + right_desvel;

   PubVelocity();

}


void desvelCallback(const geometry_msgs::Twist::ConstPtr& Dvelocity)
{

   left_desvel = Dvelocity->linear.x - Dvelocity->angular.z/2;
   
   right_desvel = Dvelocity->linear.x + Dvelocity->angular.z/2;

   errorl = - leftvel + left_desvel;

   errorr = - rightvel + right_desvel;

   PubVelocity();

}

/*
void veldesCallback(const geometry_msgs::Vector3::ConstPtr& velocity)
{

   errorl = leftvel - velocity->x;

   errorr = rightvel - velocity->y;

   PubVelocity();

}

*/

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "Veldocking");

  ros::NodeHandle n;

  Velocities_pub = n.advertise<geometry_msgs::Vector3>("cmd_vel", 1000);
    
  ros::Subscriber sub = n.subscribe("left_right", 10, wheelCallback);

  ros::Subscriber dc_sub = n.subscribe("des_cmd_vel", 1, desvelCallback);

  //ros::Subscriber cd_sub = n.subscribe("cmd_des_vel", 1, veldesCallback);

  ros::spin();

  return 0;
}
