/*************************************************
 * Project: Mr. Sticky
 * File: motor_control
 * Last update: 16.06.2017
 *************************************************/

// This is the command in the command line to use to command the motors:

//rostopic pub -1 /motor std_msgs/Int16MultiArray "layout:
//  dim:
//  - label: ''
//    size: 0
//    stride: 0
//  data_offset: 0
//data: [ 0, 0, 0]"


// ROS environment
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <sticky_robot/enc_array_msg.h>
#include <sticky_robot/ir_array_msg.h>
#include <sticky_robot/bumper_msg.h>

#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

//Definition of the pins used on the arduino for motors and encoders
#define M2_PWM 5 //left
#define M2_EN 4
#define M1_PWM 7 //right
#define M1_EN 6
#define CONV_PWM 9
#define CONV_EN 8

#define ENC1_A 2  
#define ENC1_B 3 
#define ENC2_A 20
#define ENC2_B 21
#define ENCC_A 18
#define ENCC_B 19
#define INTERRUPT_PIN_ENC1_A 0// this is the order of the interrupt pins, no conflict with pins
#define INTERRUPT_PIN_ENC1_B 1
#define INTERRUPT_PIN_ENC2_A 3
#define INTERRUPT_PIN_ENC2_B 2
#define INTERRUPT_PIN_ENCC_A 5
#define INTERRUPT_PIN_ENCC_B 4

//Pins for the IR sensors 
#define FRONT_SENSOR_CONVEYOR 0 // Sharp IR GP2Y0A21YK0F
#define LEFT_SENSOR_OBSTACLE 1 
#define CENTER_SENSOR_OBSTACLE 2 
#define RIGHT_SENSOR_OBSTACLE 3 
#define BACK_SENSOR_CONVEYOR 4
#define BOTTOM_SENSOR_CONVEYOR 5

//Pins for the bumpers
#define PIN_BUMPER_LEFT 10// define limit switch and allocate its pin
#define PIN_BUMPER_RIGHT 11


ros::NodeHandle nh;
sticky_robot::enc_array_msg enc_readings;
ros::Publisher encoder("encoders", &enc_readings);
sticky_robot::ir_array_msg ir_readings;
ros::Publisher IR("IR", &ir_readings);

sticky_robot::bumper_msg bmp_msg;
ros::Publisher bumpers("bumpers", &bmp_msg);

void motor_cb(const std_msgs::Int16MultiArray& my_ctrl_msg)
{
  int left_speed = my_ctrl_msg.data[0];
  int right_speed = my_ctrl_msg.data[1];
  int conv_speed = my_ctrl_msg.data[2];

  movingMotors(left_speed, right_speed, conv_speed);
}
ros::Subscriber<std_msgs::Int16MultiArray> control_sub("control", motor_cb);

int32_t encoder1_pos = 0;
int32_t encoder2_pos = 0;
int32_t encoderC_pos = 0;

//State of the 2 channels (A and B) of each encoder
boolean Enc1_A_set = false;
boolean Enc1_B_set = false;
boolean Enc2_A_set = false;
boolean Enc2_B_set = false;
boolean EncC_A_set = false;
boolean EncC_B_set = false;


// the setup routine runs once when you press reset:
void setup() {    
  Serial.begin(57600);

  nh.initNode();
  nh.subscribe(control_sub);
  nh.advertise(encoder);
  nh.advertise(IR);
  nh.advertise(bumpers);

  //initialisation of the pin for the motors
  pinMode(M1_PWM,OUTPUT);
  pinMode(M1_EN,OUTPUT);
  pinMode(CONV_PWM, OUTPUT);
  pinMode(M2_PWM,OUTPUT);
  pinMode(M2_EN,OUTPUT);
  pinMode(CONV_EN, OUTPUT);
//initialisation of the pins for the encoder
  pinMode(ENC1_A, INPUT);
  pinMode(ENC1_B, INPUT);
  pinMode(ENC2_A, INPUT);
  pinMode(ENC2_B, INPUT);
  pinMode(ENCC_A, INPUT);
  pinMode(ENCC_B, INPUT);
  digitalWrite(ENC1_A, HIGH);
  digitalWrite(ENC1_B, HIGH); 
  digitalWrite(ENC2_A, HIGH);
  digitalWrite(ENC2_B, HIGH);
  digitalWrite(ENCC_A, HIGH);
  digitalWrite(ENCC_B, HIGH); 

  //Initialisation of the interruption for the encoders
  attachInterrupt(INTERRUPT_PIN_ENC1_A, managementInterruptEnc1A, CHANGE); //Interrupt of the encoder 1 (output 0)
  attachInterrupt(INTERRUPT_PIN_ENC1_B, managementInterruptEnc1B, CHANGE); //Interrupt of the encoder 1 (output 1)
  attachInterrupt(INTERRUPT_PIN_ENC2_A, managementInterruptEnc2A, CHANGE); //Interrupt of the encoder 1 (output 0)
  attachInterrupt(INTERRUPT_PIN_ENC2_B, managementInterruptEnc2B, CHANGE); //Interrupt of the encoder 1 (output 1)
  attachInterrupt(INTERRUPT_PIN_ENCC_A, managementInterruptEncCA, CHANGE); //Interrupt of the encoder 1 (output 0)
  attachInterrupt(INTERRUPT_PIN_ENCC_B, managementInterruptEncCB, CHANGE); //Interrupt of the encoder 1 (output 1)

  pinMode(FRONT_SENSOR_CONVEYOR, INPUT);
  pinMode(LEFT_SENSOR_OBSTACLE, INPUT);   
  pinMode(CENTER_SENSOR_OBSTACLE, INPUT);
  pinMode(RIGHT_SENSOR_OBSTACLE, INPUT);   
  pinMode(BACK_SENSOR_CONVEYOR, INPUT);
  pinMode(BOTTOM_SENSOR_CONVEYOR, INPUT);

  pinMode(PIN_BUMPER_LEFT, INPUT);
  pinMode(PIN_BUMPER_RIGHT, INPUT);

  delay(100);
}

void movingMotors(int left_speed, int right_speed, int conv_speed)
{

  if(conv_speed){
    digitalWrite(M1_EN, HIGH);
    analogWrite(M1_PWM, 0);

    digitalWrite(M2_EN, HIGH);
    analogWrite(M2_PWM, 0);

    if(conv_speed < 0)
    {
      digitalWrite(CONV_EN, conv_speed);
      analogWrite(CONV_PWM, LOW);
    }
    else
    {
      digitalWrite(CONV_EN, LOW);
      analogWrite(CONV_PWM, conv_speed); 
    }  
  } 
  else { // conveyor should not turn
    digitalWrite(CONV_EN, LOW);
    digitalWrite(CONV_PWM, LOW);
    if (left_speed > 0){
      digitalWrite(M1_EN, LOW);
      analogWrite(M1_PWM, left_speed);
    } 
    else {
      digitalWrite(M1_EN, HIGH);
      analogWrite(M1_PWM, -left_speed); 
    }
    if (right_speed > 0){
      digitalWrite(M2_EN, HIGH);
      analogWrite(M2_PWM, right_speed);
    }
    else {
      digitalWrite(M2_EN, LOW);
      analogWrite(M2_PWM, -right_speed);
    }
  }
}



//Interruption which is called when the state of the channel A of the encoder 1 changes
void managementInterruptEnc1A()
{
  Enc1_A_set = digitalRead(ENC1_A) == HIGH;
  if(Enc1_A_set != Enc1_B_set)
    encoder1_pos += 1;
  else
    encoder1_pos += -1;
}

//Interruption which is called when the state of the channel B of the encoder 1 changes
void managementInterruptEnc1B()
{
  Enc1_B_set = digitalRead(ENC1_B) == HIGH;

  if(Enc1_A_set == Enc1_B_set)
    encoder1_pos += 1;
  else
    encoder1_pos += -1;
}



//Interruption which is called when the state of the channel A of the encoder 2 changes
void managementInterruptEnc2A()
{
  Enc2_A_set = digitalRead(ENC2_A) == HIGH;

  if(Enc2_A_set != Enc2_B_set)
    encoder2_pos += -1;
  else
    encoder2_pos += 1;
}

//Interruption which is called when the state of the channel B of the encoder 2 changes
void managementInterruptEnc2B()
{
  Enc2_B_set = digitalRead(ENC2_B) == HIGH;

  if(Enc2_A_set == Enc2_B_set)
    encoder2_pos += -1; 
  else
    encoder2_pos += 1;
}




//Interruption which is called when the state of the channel A of the encoder 3 changes
void managementInterruptEncCA()
{
  EncC_A_set = digitalRead(ENCC_A) == HIGH;

  if(EncC_A_set != EncC_B_set)
    encoderC_pos += 1;
  else
    encoderC_pos += -1;
}

//Interruption which is called when the state of the channel B of the encoder 3 changes
void managementInterruptEncCB()
{
  EncC_B_set = digitalRead(ENCC_B) == HIGH;

  if(EncC_A_set == EncC_B_set)
    encoderC_pos += 1;
  else
    encoderC_pos += -1;
}

void loop() {
  nh.spinOnce();
  
  int volts = analogRead(FRONT_SENSOR_CONVEYOR);
  ir_readings.ir_front = volts; 
  volts = analogRead(LEFT_SENSOR_OBSTACLE);
  ir_readings.ir_left = volts;
  volts = analogRead(CENTER_SENSOR_OBSTACLE);
  ir_readings.ir_center = volts;
  volts = analogRead(RIGHT_SENSOR_OBSTACLE);
  ir_readings.ir_right = volts;
  volts = analogRead(BACK_SENSOR_CONVEYOR);
  ir_readings.ir_back = volts;
  volts = analogRead(BOTTOM_SENSOR_CONVEYOR);
  ir_readings.ir_bottom = volts;
  
  IR.publish(&ir_readings);
  
  
  enc_readings.l_encoder = encoder1_pos;
  enc_readings.r_encoder = encoder2_pos;
  enc_readings.c_encoder = encoderC_pos;
  encoder.publish(&enc_readings);
  
  
  bool switch1 = !digitalRead(PIN_BUMPER_LEFT); //define the left switch as switch 1, convert to normally false
  bool switch2 = !digitalRead(PIN_BUMPER_RIGHT);

  bmp_msg.bumper_left = switch1;
  bmp_msg.bumper_right = switch2;
  bumpers.publish(&bmp_msg);

  delay(100);
}
