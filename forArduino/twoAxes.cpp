/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servo_xs
 * using ROS and the arduiono
 *
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo servo_x;
Servo servo_y;

void servo_x_cb( const std_msgs::UInt16& cmd_msg){
  servo_x.write(cmd_msg.data); //set servo_x angle, should be from 0-180
}

void servo_y_cb( const std_msgs::UInt16& cmd_msg){
  servo_y.write(cmd_msg.data); //set servo_y angle, should be from 0-180
}

ros::Subscriber<std_msgs::UInt16> sub_x("servo_x", servo_x_cb);
ros::Subscriber<std_msgs::UInt16> sub_y("servo_y", servo_x_cb);

void setup(){

  nh.initNode();
  nh.subscribe(sub_x);
  nh.subscribe(sub_y);

  servo_x.attach(9); //attach it to pin 9
  servo_y.attach(10); //attach it to pin 10
}

void loop(){
  nh.spinOnce();
  delay(1);
}
