// Arduino Teensy and ROS(1) Noetic with BNO055 IMU and 2 Quadrature Encoders
// Use with Diffbot Github repo, and Grove motor controller
// IMU: Red to 3.3V, Black GRND, Yellow SCL 19, Blue SDA 18
// Encoder setup. Plan to use Teensy 3.2 or 4.0

#define ENCODER_OPTIMIZE_INTERRUPTS //Only for Teensy or Arduino
#define ENCODER_USE_INTERRUPTS
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <ros.h>
#include "ros/time.h"
#include <std_msgs/Int32.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "sensor_msgs/Imu.h"

long low_wrap = -2147483647;
long high_wrap = 2147483647;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

Encoder encoderRight( 9, 10 ); // Pins for MOTOR A encoder data
Encoder encoderLeft( 11, 12 ); // pins for MOTOR B encoder data

ros::NodeHandle nh;

std_msgs::Int32 encoder_left_value, encoder_right_value;
ros::Publisher encoder_left_pub("encoder_left_value", &encoder_left_value);
ros::Publisher encoder_right_pub("encoder_right_value", &encoder_right_value);
//ros::Publisher distance_pub("distance_value", &distance_value);
sensor_msgs::Imu imu_msg; 
ros::Publisher imu_pub("imu/data", &imu_msg);


void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
//  nh.subscribe(sub);
  nh.loginfo("Wheel Encoders:");
  nh.advertise(encoder_left_pub);
  nh.advertise(encoder_right_pub);  
  nh.advertise(imu_pub);
  
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
   
  while (!nh.connected())
  {
    nh.spinOnce();
  }
  nh.loginfo("TEENSY CONNECTED");
  delay(1);
}


void loop()
{
  long newLeft, newRight;
  newLeft = encoderLeft.read();
  newRight = encoderRight.read();
  if (newLeft > high_wrap) {newLeft  = 0;
  }
  if (newLeft < low_wrap)  {newLeft  = 0;
  }
  if (newRight > high_wrap){newRight = 0;
  }
  if (newRight < low_wrap) {newRight = 0;
  }
  
  encoder_left_value.data = newLeft;
  encoder_right_value.data = newRight;
  //  publish two encoder messages
  encoder_left_pub.publish( &encoder_left_value );
  encoder_right_pub.publish( &encoder_right_value );

  nh.spinOnce();

  //static sensor_msgs::Imu msg;
  //msg.header.frame_id = "base_link";
  //msg.header.stamp = ros::Time();
     
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Quaternion quat = bno.getQuat();
  
  //nh.loginfo("In main loop now.");
	
  // http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Imu.html
  sensor_msgs::Imu imu_msg;
    //ros::Time now = ros::Time();
    imu_msg.header.stamp = ros::Time();
    imu_msg.header.frame_id = "base_link";
    imu_msg.orientation.x = quat.w();
    imu_msg.orientation.y = quat.x();
    imu_msg.orientation.z = quat.y();
    imu_msg.orientation.w = quat.z();
    imu_msg.linear_acceleration.x = (euler.x());
    imu_msg.linear_acceleration.y = (euler.y());
    imu_msg.linear_acceleration.z = (euler.z());
    imu_msg.angular_velocity.x = (gyro.x());
    imu_msg.angular_velocity.y = (gyro.y());
    imu_msg.angular_velocity.z = (gyro.z());
  // https://github.com/Russ76/ros_mpu6050_node-1/blob/master/src/mpu6050_node.cpp
  imu_pub.publish(&imu_msg);
  
  // Turn down delay for proper output speed
delay (400);

}
