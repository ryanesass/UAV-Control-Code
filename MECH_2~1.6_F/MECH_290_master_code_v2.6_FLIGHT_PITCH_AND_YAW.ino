#include <math.h>
#include <Wire.h>
#include <LIDARLite.h>
#include <Adafruit_PWMServoDriver.h>

/*
 * Ryan Sass
 * Santa Clara University
 * rsass@scu.edu or ryan.sass@gmail.com
 * 
 * Date Published: 5/16/2016
 * Version Number: 2.6
 * File: MECH_290_master_code_v2.6_FLIGHT_PITCH_AND_YAW.ino
 * 
 * Full Control Command Code
 * 
 * Intended to be run on Arduino MEGA 2560 r3
 * 
 * 
 ****** I/O PIN ASSIGNMENTS ****** 
 * 
 *     ~~~ Input ~~~ 
 * A0 - LIDAR Sensor Left
 * A1 - LIDAR Sensor Right
 * A4 - Receiver Roll Signal
 * A5 - Receiver Pitch Signal
 * A6 - Receiver Yaw Signal
 * A7 - Receiver Gear Signal
 * 
 *     ~~~ Output ~~~
 *  03 - [PWM] Roll Signaling
 *  06 - [PWM] Pitch Signaling
 *  10 - [PWM] Yaw Signaling
 *  45 - Control Lockout Signal (tied to Gear Signal)
 *  
 */


//VARIABLES TO BE INITIALIZED
/*
roll - roll output pin value
pitch - pitch output pin value
yaw - yaw output pin value
CLS - control lockout signal output pin value 
h_val_10 - value between 0 - 1024 that delineates what is 'HIGH'
ctrl_enable - CONTROL ENABLE - allows yaw control to occur. It is a signal input.
lidar_l - LIDAR SIGNAL LEFT - Signal input of the left LIDAR unit. 
lidar_r - LIDAR SIGNAL RIGHT - Signal input of the right LIDAR unit. 
lidar_diff - LIDAR DIFFERENCE - subtraction between left and right LIDAR units. 
lidar_yaw_db  - LIDAR YAW DEADBAND - The deadband window for LIDAR difference (lidar_diff) that sets to 0. 
yaw_error - YAW ERROR - The difference between LIDAR difference (lidar_diff) value and yaw setpoint (yaw_sp). 
yaw_sp - YAW SET POINT - hard coded value of the yaw set point, the difference between the two LIDAR units.
yaw_p - YAW PROPORTIONAL GAIN - the proportioanl gain for the yaw control
yaw_PWM - YAW PWM SIGNAL - the converted PWM output signal

*/
//variable assignments
bool success1,success2;

//int roll = 3;
int pitch = 2;
int yaw = 4;
int CLS = 45;
int h_val_10 = 612;
int lidar_yaw_db_o = 3;
int yaw_only_db = 40;
int yaw_sp = 0;
int32_t pwm_freq = 45;
int sig_avg = 25;
int yawpitch = 0; //yaw = 0, pitch = 1
int yaw_PWM_neutral = 278;
int lidar_pitch_db = 1;
int pitch_sp = 670;
int pitch_PWM_neutral = 278;
int pitch_trim = 11;
int pitch_error = 0;
int pitch_temp = 0;
int ctrl_enable,lidar_l,lidar_r,lidar_diff,yaw_error,yaw_PWM,x,pitch_ave,pitch_PWM,output1,pitch_error_old,pitch_delta,lidar_yaw_db;

float db_grow;
float angle,tx,ty,tz;
float pitch_P = 1.0;
float yaw_P = 1.0;
//float pitch_P = 10.5;
//float yaw_P = 11.5;


int sensorPins[] = {2,3}; // Array of pins connected to the sensor Power Enable lines
unsigned char addresses[] = {0x66,0x68};
LIDARLite myLidarLite;
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver();


void setup() {   

  // initialize serial communication at 9600 bits per second:
//  Serial.begin(9600);
  
  //Turn pins on
 
  pinMode(31,OUTPUT);
  digitalWrite(31,HIGH);
  //initializing Lidar signals
  myLidarLite.begin();
  myLidarLite.changeAddressMultiPwrEn(2,sensorPins,addresses,false);
  //start PWM board
  pwm1.begin();
  pwm1.setPWMFreq(48);
 
  //pinout assignments
  pinMode(CLS,OUTPUT);
  ctrl_enable = 1;
  lidar_yaw_db = lidar_yaw_db_o;
  pwmneutral();
  attachInterrupt(digitalPinToInterrupt(18),stopDataLogging,RISING);
  pinMode(18, INPUT);
  initializeDataLogger();
}

/*******************************************START OF MAIN CODE*********************************************/
void loop() {  
    //process LIDAR sensors
    String s(millis());
    s += " ";
    logData(s);
    lidarprocess();    
    
    yaw_control();//yaw vs pitch control
    //Serial.print("Yaw Pitch \t");
    //Serial.println(yawpitch);
    if(yawpitch == 1) pitch_control();
    
    //set PWM values to controller
    pwmset();   
    logData('\n');
   
    //print any serial outputs for debugging
    //printoutput();
}
/*********************************************END OF MAIN CODE*********************************************/

void enab_sig_proc(){
  ctrl_enable = analogRead(A7);

  //character gear signal to equivalent boolean
  if(ctrl_enable >= h_val_10) {
    //enable signal - enable TRUE
    ctrl_enable = HIGH;
  } else {
    //disable signal - enable FALSE
    ctrl_enable = LOW;
  }

  //signal yaw command signal relay to connect
  digitalWrite(CLS,ctrl_enable);
}

void lidarprocess(){
  //LIDAR signal input & averaging
  x = 0; lidar_r = 0; lidar_l = 0;
  while(x < sig_avg){
    lidar_r += myLidarLite.distance(true,true,0x66);  
    lidar_l += myLidarLite.distance(true,true,0x68);
    x++;
    delay(2);
  }
  lidar_r /= x;
  lidar_l /= x;
  logFloat(lidar_r, " lidar_r: ");
  logFloat(lidar_l, " lidar_l: ");
  yawcheck();
  
  //LIDAR signaling diferencing
  pitch_ave = (lidar_l + lidar_r)/2;
  if(yawpitch != -1){
    anglecalc();
  } else {
    angle = 0;
  }
   
  db_grow = pitch_sp - pitch_ave;
  db_grow /= pitch_sp;
    
}

void yawcheck(){
  if((lidar_l > 0) && (lidar_r > 0)){
    if((pitch_ave > 1250) || (pitch_ave < 250)){
      yawpitch = 0;
      digitalWrite(31,HIGH);
    } else {
      if(abs(angle) <= yaw_only_db) {
        yawpitch = 1;  
        digitalWrite(31,LOW);
      } else {
        yawpitch = 0; 
        digitalWrite(31,HIGH);
      }      
    }    
  } else {
    yawpitch = -1;  
    digitalWrite(31,HIGH);
  }    
}

void yaw_control(){
     
  //set point signal conditioning
  if(lidar_l > lidar_r){
    yaw_error = angle - yaw_sp;
  } else {
    yaw_error = yaw_sp - angle;    
  }
  if(abs(yaw_error) <= lidar_yaw_db) yaw_error = 0;   
    
  //yaw PWM signal conditioning
//  yaw_error = (yaw_error > 45 || yaw_error < -45) ? (abs(yaw_error)/yaw_error)*45 : yaw_error;
  if(yaw_error > 45) yaw_error = 45;
  if(yaw_error < -45) yaw_error = -45;
  yaw_error /= yaw_P;
  yaw_PWM = yaw_error + yaw_PWM_neutral;  
  logFloat(yaw_PWM, " yaw_PWM: ");
  if(yawpitch != 1)  pitch_PWM = pitch_trim + pitch_PWM_neutral;
}

void pitch_control(){
  pitch_error_old = pitch_temp;
  pitch_error = pitch_sp - pitch_ave;
  pitch_temp = pitch_error;
  pitch_error /= 10;

  //signal difference deadbanding    
  if(abs(pitch_error) <= lidar_pitch_db) pitch_error = 0;
     
  //proportional gain
  pitch_error *= pitch_P;

  //yaw PWM signal conditioning
  if(pitch_error > 40) pitch_error = 40;
  if(pitch_error < -40) pitch_error = -40;
  pitch_error /= 2.2;
  pitch_delta = pitch_temp - pitch_error_old;
  pitch_delta *= (1/3.0);
  if(pitch_delta > 30) pitch_delta = 30;
  if(pitch_delta < -30) pitch_delta = -30;
  if(abs(pitch_delta) < 1) pitch_delta = 0;
  pitch_PWM = pitch_error + pitch_trim + pitch_delta + pitch_PWM_neutral;
  if(pitch_PWM > pitch_trim + pitch_PWM_neutral + 15)   pitch_PWM = pitch_trim + 15 + pitch_PWM_neutral;
  if(pitch_PWM < pitch_trim + pitch_PWM_neutral - 20)   pitch_PWM = pitch_trim - 20 + pitch_PWM_neutral;
  logFloat(pitch_PWM, " pitch_PWM: ");
}

void anglecalc(){
  
  if(lidar_l < lidar_r){
    tx = pitch_ave - lidar_l;
  }else{
    tx = pitch_ave - lidar_r;
  }
  
  angle = lidar_l;
  tz = lidar_r;
  ty = (sqrt( (angle * angle) + (tz * tz) - (2 * angle * tz * 0.9848) ) * 0.5 );
  angle = ( (ty * ty) - (tx * tx) );
  if(angle < 0.05) angle = 0;
  
  tz = sqrt(angle);
  if((tx == 0) || (ty == 0)){
    angle = 0;
  } else {
    angle = ( (tx*tx) + (ty*ty) - (tz*tz) ) / (2 * tx * ty);
  }  
  
  angle = (90 - (180 / 3.14159) * acos(angle));
}

void printoutput(){
  //printing output to monitor
  Serial.print("\n\n***** START ****** \n\n\n");
  Serial.print("\n Angle \t");
  Serial.print(angle);
  Serial.print("\n Pitch Ave \t");
  Serial.println(pitch_ave);
  Serial.print("\n");
  Serial.print("\n LIDAR L \t");
  Serial.print(lidar_l);
  Serial.print("\n LIDAR R \t");
  Serial.print(lidar_r);
  Serial.print("\n");
  Serial.print("\n Mode \t");
  if(yawpitch == 0){
    Serial.print("YAW");
  }else{
    if(yawpitch == -1){
      Serial.print("BAD SIGNAL");
    }else{
      Serial.print("YAW & PITCH");
    }
  }
  Serial.print("\n Pitch PWM signal \t");
  Serial.print(pitch_PWM);
  Serial.print("\n Yaw PWM signal \t");
  Serial.print(yaw_PWM);
  Serial.print("\n");
  output1 = pitch_PWM;
  if(output1 == pitch_PWM_neutral + pitch_trim){
    Serial.print("\nNEUTRAL ");
  }else{
    if(output1 > pitch_PWM_neutral + pitch_trim){
      Serial.print("\nBACKWARD ");
    } else {
      Serial.print("\nFORWARD ");
    }
  }
  Serial.print(output1);
  Serial.print("\n\n***** BREAK****** \n\n\n");
  delay(1000);
}

void pwmset(){
  pwm1.setPWM(yaw, 0, yaw_PWM);
  delay(1);
  pwm1.setPWM(pitch, 0, pitch_PWM);
}

void pwmneutral(){
  pwm1.setPWM(0, 0, yaw_PWM_neutral);
  pwm1.setPWM(1, 0, yaw_PWM_neutral);
  //pwm1.setPWM(2, 0, yaw_PWM_neutral);
  pwm1.setPWM(3, 0, yaw_PWM_neutral);
  //pwm1.setPWM(4, 0, yaw_PWM_neutral);
  pwm1.setPWM(5, 0, yaw_PWM_neutral);
  pwm1.setPWM(6, 0, yaw_PWM_neutral);
  pwm1.setPWM(7, 0, yaw_PWM_neutral);
  pwm1.setPWM(8, 0, yaw_PWM_neutral);  
 }

