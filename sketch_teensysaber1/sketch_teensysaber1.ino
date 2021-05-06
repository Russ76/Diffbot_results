// Teensy board
// Uses Sabertooth motor controller
// Motor scale for needed input: -0.5 to 0.5
// Will work directly with Joy and teleop_twist_joy messages
// Callback routine multiplies number by 70 for full range
// Encoder messages published
// Adafruit_MPU6050 code added

#include <ros.h>
#include "ros/time.h"
#include <std_msgs/Int32.h>
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/Twist.h>
// #include <diffbot_msgs/Encoder.h>
#include <std_msgs/Empty.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

#define GRAVITY 9.81 // 0.00059855
#define MOTOR_TIMEOUT_MS 1000

// int motorTimer;

// Motor definitions to make life easier:
// #define MOTOR_A 1
// #define MOTOR_B 0
int pwrLeft = 64;  // start with brakes on
int pwrRight = 192; // this is Sabertooth's method of simplified serial
// Each motor has 7 bits of speed resolution
float angularboost = 1.75;
float linx;
float angZ;

// Encoder setup
#define ENCODER_OPTIMIZE_INTERRUPTS //Only for Teensy (3.2 tested, works well!)
#include <Encoder.h>
Encoder encoderLeft(2, 3); // MOTOR A encoder data
Encoder encoderRight(4, 5);// MOTOR B encoder data

ros::NodeHandle nh;

// Callbacks

void resetCb( const std_msgs::Empty& reset)
{
  encoderLeft.write(0);
  encoderRight.write(0);
  nh.loginfo("Reset both wheel encoders to zero");
}

void messageCb( const geometry_msgs::Twist& msg)
{
	linx = msg.linear.x * 70;
	angZ = msg.angular.z *70 * angularboost;
// motorTimer = millis();

	if(linx == 0){  // turning
		pwrRight = angZ;
		pwrLeft = (-1) * pwrRight;
	}else if(angZ == 0){ // fordward / backward
		pwrLeft = pwrRight = linx;
	}else{ // moving doing arcs
		pwrLeft = linx - angZ;
		pwrRight = linx + angZ;
	}
     pwrLeft = pwrLeft + 64;
  if (pwrLeft > 127) pwrLeft = 127;
  if (pwrLeft < 1) pwrLeft = 1;
     pwrRight = pwrRight + 192;
  if (pwrRight > 255) pwrRight = 255;
  if (pwrRight < 128) pwrRight = 128;
}

sensor_msgs::Imu imu_msg; 
ros::Publisher imu_pub("imu/data", &imu_msg);

std_msgs::Int32 encoder_left_value, encoder_right_value;
ros::Publisher encoder_left_pub("encoder_left_value", &encoder_left_value);
ros::Publisher encoder_right_pub("encoder_right_value", &encoder_right_value);

// ros::Subscriber<std_msgs::Int32> sub("motor_left", &left_motorCb);
// ros::Subscriber<std_msgs::Int32> sub_2("motor_right", &right_motorCb);
ros::Subscriber<std_msgs::Empty> sub_reset("reset", resetCb); 
ros::Subscriber<geometry_msgs::Twist> sub("/jet_drive_controller/cmd_vel", &messageCb );
 
void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub_reset);
//  nh.subscribe(sub_2);
  nh.loginfo("Wheel Encoders:");
  nh.advertise(encoder_left_pub);
  nh.advertise(encoder_right_pub);  
  nh.advertise(imu_pub);
  
   Serial2.begin(9600);
  
   if (!mpu.begin()) {
    nh.loginfo("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  nh.loginfo("MPU6050 Found!");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
    while (!nh.connected())
    {
        nh.spinOnce();
    }
    nh.loginfo("TEENSY CONNECTED");
    delay(1);
  //motorTimer = millis();
}
 
void loop()
{
  long newLeft, newRight;
  newLeft = encoderLeft.read();
  newRight = encoderRight.read();

  encoder_left_value.data = newLeft;
  encoder_right_value.data = newRight;
//  publish two encoder messages
  encoder_left_pub.publish(&encoder_left_value);
  encoder_right_pub.publish(&encoder_right_value);
  
  nh.spinOnce();

    /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  delay(15);
  
  //nh.loginfo("In main loop now.");
	
	// http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Imu.html
	sensor_msgs::Imu imu_msg;
 //       ros::Time now = ros::Time::now();
        ros::Time ros_time;
        imu_msg.header.stamp = ros_time;
        imu_msg.header.frame_id = "base_link";
        imu_msg.linear_acceleration.x = (a.acceleration.x) * GRAVITY;
        imu_msg.linear_acceleration.y = (a.acceleration.y) * GRAVITY;
        imu_msg.linear_acceleration.z = (a.acceleration.z) * GRAVITY;
        imu_msg.angular_velocity.x = (g.gyro.x)*3.1415926/180;
        imu_msg.angular_velocity.y = (g.gyro.y)*3.1415926/180;
        imu_msg.angular_velocity.z = (g.gyro.z)*3.1415926/180;
	// https://github.com/Russ76/ros_mpu6050_node-1/blob/master/src/mpu6050_node.cpp
	imu_pub.publish(&imu_msg);
	
  Serial2.write(pwrLeft); // motor speed
  Serial2.write(pwrRight); // Power motors both directions
	
}
