
/*
 * Arduino function that subscribes on ROS a Twist velocity message and translate into a 
 * value between 0 and 255 sending the consequent PWM signal 
 */

#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include "TimerOne.h"
#define enA 6
#define enB 5
#define in1 9
#define in2 10
#define in3 11
#define in4 12                                                                                                                                                                                                                     

ros::NodeHandle  nh;

geometry_msgs::Vector3 vel;
ros::Publisher odom_pub("wheel_encoder", &vel);
//double odomvel;


const byte MOTOR1 = 2;  // Motor 1 Interrupt Pin - INT 0
const byte MOTOR2 = 3;  // Motor 2 Interrupt Pin - INT 1
                                                                                                                                                                                                                     
unsigned int counter1 = 0;
unsigned int counter2 = 0;

float diskslots = 20;  // Change to match value of encoder disk


void ISR_count1()  
{
  counter1++;  // increment Motor 1 counter value
} 


// Motor 2 pulse count ISR
void ISR_count2()  
{
  counter2++;  // increment Motor 2 counter value
} 



void ISR_timerone()
{
  vel.x = counter1;
  vel.y = counter2;
  counter1 = 0;
  counter2 = 0;
  odom_pub.publish(&vel);
}

void velocityCallback(const geometry_msgs::Vector3& motor)
{
  analogWrite(enA, motor.x); // Send PWM signal to L298N Enable pin
  analogWrite(enB, motor.y);
}

void setup()
{
  nh.initNode();
  nh.advertise(odom_pub);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  // Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  Timer1.initialize(100000); // set timer for .1sec
  
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &velocityCallback);

void loop()
{
  attachInterrupt(digitalPinToInterrupt (MOTOR1), ISR_count1, RISING);  // Increase counter 1 when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt (MOTOR2), ISR_count2, RISING);  // Increase counter 2 when speed sensor pin goes High
  Timer1.attachInterrupt( ISR_timerone ); // Enable the timer
  nh.subscribe(sub);
  nh.spinOnce();  
  delay(1);
}
