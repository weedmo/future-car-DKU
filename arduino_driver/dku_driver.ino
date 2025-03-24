#include "Car_Library.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <Encoder.h>

void longitudinal_command(int speed);
void longitudinal_command2(int16_t speed, int8_t dir);
void lateral_command(int target);

void get_sonar();

const int trig_lf = 53;
const int trig_rf = 51;
const int trig_cf = 49;
const int echo_lf = 52;
const int echo_rf = 50;
const int echo_cf = 48;

const int trig_lr = 46;
const int trig_rr = 44;
const int trig_cr = 42;
const int echo_lr = 47;
const int echo_rr = 45;
const int echo_cr = 43;

const int steering_motor_pin_1  = 10;
const int steering_motor_pin_2  = 7;
const int left_motor_pin_1      = 9;
const int left_motor_pin_2      = 12;
const int right_motor_pin_1     = 8;
const int right_motor_pin_2     = 11;
// const int endstop_pin = 21;
const int res_pin = A0;

int16_t us_ranges[6] = {0};
int count = 0;
bool halt = false;

Encoder encoder = Encoder(2, 3);
ros::NodeHandle  nh;

std_msgs::Int16 pub_msg;
std_msgs::Int16MultiArray pub_range_msg;

int16_t target_longitudinal = 0;
int16_t target_latteral = 0;
void cb_longicmd(const std_msgs::Int16& msg) {
  target_longitudinal = msg.data;
  count = 0;
  halt = false;
}
void cb_lattcmd(const std_msgs::Int16& msg) {
  target_latteral = msg.data;
  count = 0;
  halt = false;
}
ros::Subscriber<std_msgs::Int16> sub_longi("longi_cmd", cb_longicmd );
ros::Subscriber<std_msgs::Int16> sub_latt("latt_cmd", cb_lattcmd );
ros::Publisher publisher("steering", &pub_msg);
ros::Publisher publisher_us("ultrasonics", &pub_range_msg);

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(publisher);
  nh.advertise(publisher_us);
  nh.subscribe(sub_longi);
  nh.subscribe(sub_latt);

  pinMode(steering_motor_pin_1, OUTPUT);
  pinMode(steering_motor_pin_2, OUTPUT);
  pinMode(left_motor_pin_1, OUTPUT);
  pinMode(left_motor_pin_2, OUTPUT);
  pinMode(right_motor_pin_1, OUTPUT);
  pinMode(right_motor_pin_2, OUTPUT);
  // pinMode(endstop_pin, INPUT_PULLUP);
  pinMode(trig_rf, OUTPUT);
  pinMode(trig_cf, OUTPUT);
  pinMode(trig_lf, OUTPUT);
  pinMode(trig_rr, OUTPUT);
  pinMode(trig_cr, OUTPUT);
  pinMode(trig_lr, OUTPUT);
  pinMode(echo_rf, INPUT);
  pinMode(echo_cf, INPUT);
  pinMode(echo_lf, INPUT);
  pinMode(echo_rr, INPUT);
  pinMode(echo_cr, INPUT);
  pinMode(echo_lr, INPUT);

  // initialize_steering(100);
}

void loop() {
  get_sonar();
  count ++;
  if ( count > 30 ) {
    halt = true;
    motor_hold(steering_motor_pin_1, steering_motor_pin_2);
    motor_hold(left_motor_pin_1, left_motor_pin_2);
    motor_hold(right_motor_pin_1, right_motor_pin_2);
  }
  if (halt == false) {
    lateral_command(target_latteral);
    // longitudinal_command(target_longitudinal);
    longitudinal_command2(target_longitudinal, target_latteral/15);
    publisher.publish(&pub_msg);
  }
  pub_range_msg.data = us_ranges;
  pub_range_msg.data_length = 6;
  publisher_us.publish(&pub_range_msg);

  pub_msg.data = analogRead(A0);
  publisher.publish(&pub_msg);
  nh.spinOnce();
  delay(30);
}

// void initialize_steering(int speed) {
//   Serial.println("Start");
//   for (int i = 0 ; i < 30 ; i ++ ) {
//     motor_forward(steering_motor_pin_1, steering_motor_pin_2, speed);
//     delay(100);
//   }
//   motor_backward(steering_motor_pin_1, steering_motor_pin_2, speed);
  
//   while (digitalRead(endstop_pin));
//   Serial.println("Endstop down");

//   while (!digitalRead(endstop_pin)){
//     delay(1);
//     motor_forward(steering_motor_pin_1, steering_motor_pin_2, speed);
//   }
//   Serial.println("Endstop up");
  
//   encoder.write(0);

//   while(encoder.read() < 400){
//     motor_forward(steering_motor_pin_1, steering_motor_pin_2, speed);
//   }
//   motor_hold(steering_motor_pin_1, steering_motor_pin_2);
// }

void get_sonar() {
  us_ranges[0] = ultrasonic_distance(trig_lf,echo_lf);
  delayMicroseconds(100);
  us_ranges[1] = ultrasonic_distance(trig_cf,echo_cf);
  delayMicroseconds(100);
  us_ranges[2] = ultrasonic_distance(trig_rf,echo_rf);
  delayMicroseconds(100);
  us_ranges[3] = ultrasonic_distance(trig_rr,echo_rr);
  delayMicroseconds(100);
  us_ranges[4] = ultrasonic_distance(trig_cr,echo_cr);
  delayMicroseconds(100);
  us_ranges[5] = ultrasonic_distance(trig_lr,echo_lr);
  delayMicroseconds(100);
}


void lateral_command(int target) {  
  float kp = 1.5;
  float ki = 0.0;
  float kd = 0.3;

  int y = map(analogRead(A0), 60, 1023, 600, -600);
  int e = target - y;
  static int ie = 0;
  static int pe = e;

  ie += e;
  if (ie > 100) ie = 100;
  if ( ie < -100 ) ie = -100;

  int u = int(kp*e + ki*ie + kd*(e-pe));
  pe = e;
  if ( u > 255 ) u = 255;
  if ( u < -255 ) u = -255;

  if ( u>0 ) {
    motor_backward(steering_motor_pin_1, steering_motor_pin_2, u);
  }
  else {
    motor_forward(steering_motor_pin_1, steering_motor_pin_2, -u);
  }
}


void longitudinal_command2(int16_t speed, int8_t dir) {
  
    if (speed > 0) {
    int lspeed = speed-dir > 255 ? 255 : speed-dir; 
    int rspeed = speed+dir > 255 ? 255 : speed+dir;
    motor_forward(left_motor_pin_1, left_motor_pin_2, lspeed);
    motor_forward(right_motor_pin_1, right_motor_pin_2, rspeed);
  }
  else if (speed < 0) {
    speed = -speed;
    int lspeed = speed-dir > 255 ? 255 : speed-dir; 
    int rspeed = speed+dir > 255 ? 255 : speed+dir;
    motor_backward(left_motor_pin_1, left_motor_pin_2, lspeed);
    motor_backward(right_motor_pin_1, right_motor_pin_2, rspeed);
  }
  else {
    motor_backward(left_motor_pin_1, left_motor_pin_2, 0);
    motor_backward(right_motor_pin_1, right_motor_pin_2, 0);
  }
}

void longitudinal_command(int16_t speed) {
  if (speed > 0) {
    motor_forward(left_motor_pin_1, left_motor_pin_2, speed);
    motor_forward(right_motor_pin_1, right_motor_pin_2, speed);
  }
  else {
    motor_backward(left_motor_pin_1, left_motor_pin_2, -speed);
    motor_backward(right_motor_pin_1, right_motor_pin_2, -speed);
  }
}