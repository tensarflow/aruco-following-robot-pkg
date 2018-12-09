/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servo_ys
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

Servo servo_y;

void servo_y_cb( const std_msgs::UInt16& cmd_msg){
  servo_y.write(cmd_msg.data); //set servo_y angle, should be from 0-180
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led
}


ros::Subscriber<std_msgs::UInt16> sub("servo_y", servo_y_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);

  servo_y.attach(9); //attach it to pin 9
}

void loop(){
  nh.spinOnce();
  delay(1);
}
