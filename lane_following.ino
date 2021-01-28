#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
//#include <std_msgs/MultiArrayLayout.h>
//#include <std_msgs/MultiArrayDimension.h>
//#include <std_msgs/UInt16MultiArray.h>
//#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>

ros::NodeHandle  nh;
float radAngle = 0;
int leftPWM = 0;
int rightPWM = 0;
int scale = 1;

// Initialize motor pins
int speedLeft = 9;  // Color?
int dir1Left = 4;  // Color?
int dir2Left = 5;  // Color?

int speedRight = 10;  // Color?
int dir1Right = 6;    // Color?
int dir2Right = 7;    // Color?

// Define function for handling steering angle
void steering_function( const std_msgs::Int16& cmd_msg) {
  // Store receieved data in variable
//  radAngle = (int) cmd_msg.data;
  radAngle = cmd_msg.data;

  digitalWrite(13, HIGH-digitalRead(13));  //toggle led as visual indicator
   // Forward drive
  digitalWrite(dir1Left, HIGH);
  digitalWrite(dir2Left, LOW);
  digitalWrite(dir1Right, HIGH);
  digitalWrite(dir2Right, LOW);
  // Left turn
  if (radAngle > 0) { 
    leftPWM = 50; 
    rightPWM = scale*radAngle + 50;
    
  } 
  // Straight trajectory
  else if (radAngle == 0) {
    leftPWM = 50;
    rightPWM = 50;
  }
  // right turn
  else if (radAngle < 0) {
    leftPWM = scale*radAngle + 50;
    rightPWM = 50;
  }
  // Apply calculated pwm to control individual motor speed
    analogWrite(speedLeft, leftPWM); 
    analogWrite(speedRight, rightPWM);
}

// Define the ROS subscribers
// Node subscribes to "steering" topic and references steering_function function
ros::Subscriber<std_msgs::Int16> sub1("steering", steering_function);

void setup(){
 // Complete setup for relevant pins
  pinMode(speedLeft, OUTPUT);
  pinMode(dir1Left, OUTPUT);
  pinMode(dir2Left, OUTPUT);
  pinMode(speedRight, OUTPUT);
  pinMode(dir1Right, OUTPUT);
  pinMode(dir2Right, OUTPUT);
  pinMode(13, OUTPUT);
  
  // open the serial port at 57600 bps:
  Serial.begin(57600);

// Initialize the node and define topic subscribers
  nh.initNode();
  nh.subscribe(sub1);
//  nh.subscribe(sub2); 
}

void loop(){
  nh.spinOnce();
  delay(1); 
}
