#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <ros/time.h>
#define encodPinM2      21                       // Quadrature encoder A pin
#define encodPinM1      20                       // Quadrature encoder B pin
#define encodPinM4      18                       // Quadrature encoder A pin
#define encodPinM3      19                     // Quadrature encoder B pin
#define M1A              12                       // PWM outputs to L298N H-Bridge motor driver module
#define M1B              13
#define M2A              10                       // PWM outputs to L298N H-Bridge motor driver module
#define M2B              11
#define M3A              7                       // PWM outputs to L298N H-Bridge motor driver module
#define M3B              6
#define M4A              9                       // PWM outputs to L298N H-Bridge motor driver module
#define M4B              8
#define LOOPTIME        100

double kpM1 = 0 , kiM1 = 200000 , kdM1 = 0;             // modify for optimal performance
double inputM1 = 0, outputM1 = 0, setpointM1 = 0;
double kpM2 = 0, kiM2 = 200000, kdM2 = 0;             // modify for optimal performance
double inputM2 = 0, outputM2 = 0, setpointM2 = 0;
double kpM3 = 0 , kiM3 = 200000 , kdM3 = 0;             // modify for optimal performance
double inputM3 = 0, outputM3 = 0, setpointM3 = 0;
double kpM4 = 0, kiM4 = 200000, kdM4 = 0;             // modify for optimal performance
double inputM4 = 0, outputM4 = 0, setpointM4 = 0;
long temp;
volatile long encoderPosM1 = 0, encoderPosM2 = 0,  encoderPosM3 = 0, encoderPosM4 = 0;
double v_L, v_R;
unsigned long lastMilli = 0;
PID myPIDM1(&inputM1, &outputM1, &setpointM1, kpM1, kiM1, kdM1, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'
PID myPIDM2(&inputM2, &outputM2, &setpointM2, kpM2, kiM2, kdM2, DIRECT);
PID myPIDM3(&inputM3, &outputM3, &setpointM3, kpM3, kiM3, kdM3, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'
PID myPIDM4(&inputM4, &outputM4, &setpointM4, kpM4, kiM4, kdM4, DIRECT);
int distancefr=0, distanceleft=0, distanceright=0, durationfr=0, durationleft=0, durationright=0;
void encoderL();
void encoderR();

const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;

const float diameter = 0.085;                   //Wheel radius, in m
const float wheelbase = 0.20;               //Wheelbase, in m

float speed_req = 0;                         //Desired linear speed for the robot, in m/s
float angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

float speed_req_left = 0;                    //Desired speed for left wheel in m/s                //Actual speed for left wheel in m/s
float speed_cmd_left = 0;                    //Command speed for left wheel in m/s

float speed_req_right = 0;                   //Desired speed for right wheel in m/s

void encoderM1();
void encoderM2();
void encoderM3();
void encoderM4();
ros::NodeHandle nh;
void publishSpeed(double time);
//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {                                                //Reset the counter for number of main loops without communication
  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message

  speed_req_left = speed_req - angular_speed_req * (wheelbase / 2); //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req * (wheelbase / 2); //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
geometry_msgs::Vector3 speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("wheels_vel", &speed_msg);  
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_act_right = 0;  
void setup() {
    pinMode(encodPinM1, INPUT);                  // quadrature encoder input A
    pinMode(encodPinM2, INPUT);
    pinMode(encodPinM3, INPUT);                  // quadrature encoder input A
    pinMode(encodPinM4, INPUT);
    pinMode(M1A, OUTPUT);
    pinMode(M1B, OUTPUT); 
    pinMode(M2A, OUTPUT); 
    pinMode(M2B, OUTPUT); // quadrature encoder input A
    pinMode(M3A, OUTPUT);
    pinMode(M3B, OUTPUT);
    pinMode(M4A, OUTPUT); 
    pinMode(M4B, OUTPUT);
    attachInterrupt(2, encoderM2, FALLING);
    attachInterrupt(3, encoderM1, FALLING);
    attachInterrupt(5, encoderM4, FALLING);
    attachInterrupt(4, encoderM3, FALLING); // update encoder position v
    nh.initNode();                            //init ROS node
    nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
    nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
    nh.advertise(speed_pub);   
    myPIDM1.SetMode(AUTOMATIC);
    myPIDM1.SetSampleTime(1);
    myPIDM1.SetOutputLimits(0, 255);
    myPIDM2.SetMode(AUTOMATIC);
    myPIDM2.SetSampleTime(1);
    myPIDM2.SetOutputLimits(0, 255);
    myPIDM3.SetMode(AUTOMATIC);
    myPIDM3.SetSampleTime(1);
    myPIDM3.SetOutputLimits(0, 255);
    myPIDM4.SetMode(AUTOMATIC);
    myPIDM4.SetSampleTime(1);
    myPIDM4.SetOutputLimits(0, 255);
}

void loop() {
  nh.spinOnce();
  if ((millis() - lastMilli) >= LOOPTIME)
  { // enter timed loop
    
    lastMilli = millis();
    if (speed_req_left < 0) {
      setpointM1 =-speed_req_left;
      inputM1 = (encoderPosM1 / 220.00) * 10 * 3.14 * diameter;
      encoderPosM1 = 0;
      myPIDM1.Compute();
      setpointM2 =-speed_req_left;
      inputM2 = (encoderPosM2 / 220.00) * 10 * 3.14 * diameter;
      encoderPosM2 = 0;
      myPIDM2.Compute();
      analogWrite(M1A, 255-outputM1);                           // drive motor CW
      digitalWrite(M1B, HIGH);
      analogWrite(M2A, 255-outputM2);                           // drive motor CW
      digitalWrite(M2B, HIGH);
       speed_act_left = -inputM1;    
    }
    if (speed_req_left >= 0) {
      setpointM1 = (speed_req_left);
      inputM1 = (encoderPosM1 / 220.00) * 10 * 3.14 * diameter ;
       encoderPosM1 = 0;
      myPIDM1.Compute();
      analogWrite(M1A, outputM1);                             // drive motor CW
      digitalWrite(M1B, LOW);  
      setpointM2 = (speed_req_left);
      inputM2 = (encoderPosM2 / 220.00) * 10 * 3.14 * diameter ;
       encoderPosM2 = 0;
      myPIDM2.Compute();
      analogWrite(M2A, outputM2);                             // drive motor CW
      digitalWrite(M2B, LOW); 
      speed_act_left = inputM1;     
    }
    if (speed_req_right < 0) {
      setpointM3 =-speed_req_right;
      inputM3 = (encoderPosM3 / 220.00) * 10 * 3.14 * diameter;
      encoderPosM3 = 0;
      myPIDM3.Compute();
       analogWrite(M3A, outputM3);                           // drive motor CW
      digitalWrite(M3B, LOW); 
      setpointM4 =-speed_req_right;
      inputM4 = (encoderPosM4 / 220.00) * 10 * 3.14 * diameter;
      encoderPosM4 = 0;
      myPIDM4.Compute();
       analogWrite(M4A, outputM4);                           // drive motor CW
      digitalWrite(M4B, LOW); 
      speed_act_left = -inputM3;  
    }

    if (speed_req_right >= 0) {
      setpointM3 = (speed_req_right);
      inputM3 = (encoderPosM3 / 220.00) * 10 * 3.14 * diameter;
      encoderPosM3 = 0;
      myPIDM3.Compute(); 
      analogWrite(M3A, 255-outputM3);                             // drive motor CW
      digitalWrite(M3B, HIGH);
      setpointM4 = (speed_req_right);
      inputM4 = (encoderPosM4 / 220.00) * 10 * 3.14 * diameter;
      encoderPosM4 = 0;
      myPIDM4.Compute(); 
      analogWrite(M4A, 255-outputM4);                             // drive motor CW
      digitalWrite(M4B, HIGH);
      speed_act_left = inputM3; 
    }
     publishSpeed(LOOPTIME); 
  }
}
void encoderM1()  {                                     // pulse and direction, direct port reading to save cycles
  if (digitalRead(encodPinM1) == LOW )  encoderPosM1++;          // if(digitalRead(encodPinB1)==HIGH)   count ++;            // if(digitalRead(encodPinB1)==LOW)   count --;
}
void encoderM2()  {                                     // pulse and direction, direct port reading to save cycles
  if (digitalRead(encodPinM2) == LOW )  encoderPosM2++;          // if(digitalRead(encodPinB1)==HIGH)   count ++;            // if(digitalRead(encodPinB1)==LOW)   count --;
}
void encoderM3()  {                                     // pulse and direction, direct port reading to save cycles
  if (digitalRead(encodPinM3) == LOW )  encoderPosM3++;          // if(digitalRead(encodPinB1)==HIGH)   count ++;            // if(digitalRead(encodPinB1)==LOW)   count --;
}
void encoderM4()  {                                     // pulse and direction, direct port reading to save cycles
  if (digitalRead(encodPinM4) == LOW )  encoderPosM4++;          // if(digitalRead(encodPinB1)==HIGH)   count ++;            // if(digitalRead(encodPinB1)==LOW)   count --;
}
void publishSpeed(double time) {
  speed_msg.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}

