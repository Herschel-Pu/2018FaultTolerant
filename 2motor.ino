#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Pose.h>
#include<tf/tf.h>



//Define ports for Left encoder
#define LeftEncoderInterruptA 0
#define LeftEncoderInterruptB 1


//Define ports for Right encoder
#define RightEncoderInterruptA 2
#define RightEncoderInterruptB 3


//set up variables foe left encoder 
volatile bool _LeftEncoderASet;
volatile bool _LeftEncoderBSet;
volatile bool _LeftEncoderAPrev;
volatile bool _LeftEncoderBPrev;
volatile long _LeftEncoderTicks = 0;
long left_encoder_vec[100];


//set up variables foe right encoder 
volatile bool _RightEncoderASet;
volatile bool _RightEncoderBSet;
volatile bool _RightEncoderAPrev;
volatile bool _RightEncoderBPrev;
volatile long _RightEncoderTicks = 0;
long right_encoder_vec[100];
long time_vector[100];

//only  set it up once
volatile int tcnt2 = 236;
//volatile int tcnt2 = 99;
int vel_window,buf_size=100;
long enc_prev,count_enc=0;

//variables for velocity - left encoder
//volatile long Old_LeftEncoderTicks=0;
//volatile long New_LeftEncoderTicks=0;
//long Left_Nticks;
//float Left_vrad_l=0;

//variables for velocity - right encoder
//volatile long Old_RightEncoderTicks=0;
//volatile long New_RightEncoderTicks=0;
//long Right_Nticks;
//float Right_vrad_l=0;

float D_S_l;
float D_S_r;
float x=0,y=0,theta=0;
//float dt;

//Time
unsigned long current_time, previous_time;
long t=0,prev_micros = 0;

// ros message set up
ros::NodeHandle  nh;
std_msgs::Float32 Wr_msg;
std_msgs::Float32 Wl_msg;
std_msgs::Int32 counter_right;
std_msgs::Int32 counter_left;
// ros message current sensor
std_msgs::Float32 current_l;
std_msgs::Float32 voltage_l;
std_msgs::Float32 current_r;
std_msgs::Float32 voltage_r;


geometry_msgs::Pose odom;

//ros publishers setup
ros::Publisher Ard_odom("arduino_odom", &odom);
ros::Publisher W_r("Vel_sens/wr", &Wr_msg);
ros::Publisher W_l("Vel_sens/wl", &Wl_msg);
ros::Publisher cnt_right("Count_right", &counter_right);
ros::Publisher cnt_left("Count_left", &counter_left);
//ros publish current sensor
ros::Publisher crt_l("current",&current_l);
ros::Publisher vtg_l("voltage",&voltage_l);
ros::Publisher crt_r("current",&current_r);
ros::Publisher vtg_r("voltage",&voltage_r);

int value_l = 0;
int value_r = 0;
float x_l_new = 0;
float y_l_new = 0;
float y_l_pre = 0;
float x_r_new = 0;
float y_r_new = 0;
float y_r_pre = 0;



//Servo Initialization
Servo servo;
Servo servo2;

//Subscriber Callback Functions right motor
void right_vel( const std_msgs::Float32& cmd_msg){
  float wr=(saturation(cmd_msg.data)*500)+1500;
  //servo.write(cmd_msg.data); //set servo angle, should be from 0-180
  servo.writeMicroseconds(wr);
}

//Subscriber Callback Functions left motor
void left_vel( const std_msgs::Float32& cmd_msg2){
  double wl=(saturation(cmd_msg2.data)*(-500))+1500;
  //servo2.write(cmd_msg2.data); //set servo angle, should be from 0-180
  servo2.writeMicroseconds(wl);
}

//Main Function ROS
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void read_sensors(){
float Right_vrad_l=0, Left_vrad_l=0;
float dt;
long Right_Nticks, Left_Nticks;

  if(count_enc<buf_size)
  {
    right_encoder_vec[count_enc] = _RightEncoderTicks;
    left_encoder_vec[count_enc]  = _LeftEncoderTicks;
    time_vector[count_enc]= micros();
    count_enc++;
  }
  else
  {

    for(int i=0;i<buf_size-1;i++)
      right_encoder_vec[i] = right_encoder_vec[i+1];
    for(int i=0;i<buf_size-1;i++)
      left_encoder_vec[i] = left_encoder_vec[i+1];
    for(int i=0;i<buf_size-1;i++)
      time_vector[i] = time_vector[i+1];
    //memcpy(right_encoder_vec,right_encoder_vec+sizeof(long),sizeof(long)*(buf_size-1));
    //memcpy(left_encoder_vec,left_encoder_vec+sizeof(long),sizeof(long)*(buf_size-1));
    //memcpy(time_vector,time_vector+sizeof(long),sizeof(long)*(buf_size-1));
    right_encoder_vec[buf_size-1] = _RightEncoderTicks;
    left_encoder_vec[buf_size-1]  = _LeftEncoderTicks;
    time_vector[buf_size-1] = micros();
  }

  Right_Nticks = right_encoder_vec[count_enc-1] - right_encoder_vec[count_enc-2];
  Left_Nticks = left_encoder_vec[count_enc-1] - left_encoder_vec[count_enc-2];

  D_S_r=((float)Right_Nticks/400)*(M_PI*0.1016);
  D_S_l=((float)Left_Nticks/400)*(M_PI*0.1016);

  x+=(D_S_r+D_S_l)/2*cos(theta);
  y+=(D_S_r+D_S_l)/2*sin(theta);
  theta+=(D_S_r-D_S_l)/0.365;
  
  enc_prev = count_enc - vel_window - 1;
  if(enc_prev<0)
    enc_prev = 0;

  Right_Nticks = right_encoder_vec[count_enc-1] - right_encoder_vec[enc_prev];
  Left_Nticks = left_encoder_vec[count_enc-1] - left_encoder_vec[enc_prev];

  dt=(float)(time_vector[count_enc-1] - time_vector[enc_prev])/1e+6;
  if(dt<0 || dt>1)
    dt = 0.15;

  Left_vrad_l=(((float)Left_Nticks*2*M_PI)/400)/dt; //velocity in radians per second
  Right_vrad_l=(((float)Right_Nticks*2*M_PI)/400)/dt; //velocity in radians per second   

 

//////////////////////////////////////////////////////////////////////////////////////////////////////

//Publishers
   Wl_msg.data=(Left_vrad_l);
   W_l.publish(&Wl_msg); 
   Wr_msg.data=(Right_vrad_l);
   W_r.publish(&Wr_msg); 

   odom.position.x=x;
   odom.position.y=y;
   odom.position.z=0;

   odom.orientation=tf::createQuaternionFromYaw(theta);
   Ard_odom.publish(&odom);

   /*counter_right.data=(_RightEncoderTicks);
   cnt_right.publish(&counter_right);
   counter_left.data=_LeftEncoderTicks;
   cnt_left.publish(&counter_left);*/

}

// read current sensor
void read_voltage(){
   value_l = analogRead(A0);//left motor read A0 port
   value_r = analogRead(A1);//right motor read A1 port
   x_l_new = value_l;
   y_l_new = 0.3*x_l_new+0.7*y_l_pre;
   x_r_new = value_r;
   y_r_new = 0.3*x_r_new+0.7*y_r_pre;

   //publish current sensor data
   voltage_l.data = float(y_l_new)/1024*5;
   y_l_pre = y_l_new;
   voltage_r.data = float(y_r_new)/1024*5;
   y_r_pre = y_r_new;
   current_l.data = (abs(voltage_l.data - 2.5))/0.185;
   current_r.data = (abs(voltage_r.data - 2.5))/0.185;
   crt_l.publish(&current_l);
   vtg_l.publish(&voltage_l);
   crt_r.publish(&current_r);
   vtg_r.publish(&voltage_r);     
  }

//Subscribers and callback functions
ros::Subscriber<std_msgs::Float32> sub("Vel_ctl/wr", right_vel);
ros::Subscriber<std_msgs::Float32> sub2("Vel_ctl/wl", left_vel);

//MAIN LOOP
void setup() 
{
//Initialize ROS node handler (subscribe and advertise topics)
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub2);
  nh.advertise(W_r);
  nh.advertise(W_l);
  nh.advertise(cnt_right);
  nh.advertise(cnt_left);
  nh.advertise(Ard_odom);

//Set up pins for H-Bridge
  servo.attach(10); //attach it to pin 10 for arduino mega
  servo2.attach(11); //attach it to pin 11 for arduino mega
  
// Port Configuration for encoders
  DDRD = DDRD | 0x00;
  PORTD = 0b00001111; 


//Set Interrupts for encoders -Left
  attachInterrupt(digitalPinToInterrupt(21), HandleLeftMotorInterruptA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(20), HandleLeftMotorInterruptB, CHANGE);

  
//Set Interrupts for encoders -Right
  attachInterrupt(digitalPinToInterrupt(19), HandleRightMotorInterruptA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(18), HandleRightMotorInterruptB, CHANGE);

  //Set interrupts for timer - only once! 
  TIMSK2 &= ~(1<<TOIE2);
  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));
  TCCR2B &= ~(1<<WGM22);
  TCCR2B |= (1<<CS21);
  TCCR2B &= ~(1<<CS20);
  TCCR2B &= ~(1<<CS22);
  ASSR &= ~(1<<AS2);
  TIMSK2 &= ~((1<<OCIE2A) | ~(1<<OCIE2B));
  TCNT2 = tcnt2; // defined as 130
  TIMSK2 |= (1<<TOIE2);

//Initialize Variables
  vel_window = 10;
  left_encoder_vec[0] = 0;
  right_encoder_vec[0] = 0;
  time_vector[0]=0;
  //current_time=micros();
  //previous_time=micros();
}

//MAIN PROGRAM
void loop()
{ 
  nh.spinOnce();
  read_sensors();
  read_voltage();
  delay(1);
}


// Interrupt service routines for the left motor's quadrature encoder
    //Channel A
void HandleLeftMotorInterruptA(){
  _LeftEncoderBSet = PIND & (1<<PD0);
  _LeftEncoderASet = PIND & (1<<PD1);
  
  _LeftEncoderTicks+=ParseEncoder_L();
  
  _LeftEncoderAPrev = _LeftEncoderASet;
  _LeftEncoderBPrev = _LeftEncoderBSet;
}

    //Channel B
void HandleLeftMotorInterruptB(){
  _LeftEncoderBSet = PIND & (1<<PD0);
  _LeftEncoderASet = PIND & (1<<PD1);
  
  _LeftEncoderTicks+=ParseEncoder_L();
  
  _LeftEncoderAPrev = _LeftEncoderASet;
  _LeftEncoderBPrev = _LeftEncoderBSet;
}

// Interrupt service routines for the right motor's quadrature encoder
    //Channel A
void HandleRightMotorInterruptA(){
  _RightEncoderBSet = PIND & (1<<PD2);
  _RightEncoderASet = PIND & (1<<PD3);
  
  _RightEncoderTicks+=ParseEncoder_R();
 
  _RightEncoderAPrev = _RightEncoderASet;
  _RightEncoderBPrev = _RightEncoderBSet;
}

    //Channel B
void HandleRightMotorInterruptB(){
  
  _RightEncoderBSet = PIND & (1<<PD2);
  _RightEncoderASet = PIND & (1<<PD3);
  
  _RightEncoderTicks+=ParseEncoder_R();
  
  _RightEncoderAPrev = _RightEncoderASet;
  _RightEncoderBPrev = _RightEncoderBSet;
}

// Logic for left encoder count
long ParseEncoder_L(){
  if(_LeftEncoderAPrev && _LeftEncoderBPrev){
    if(!_LeftEncoderASet && _LeftEncoderBSet) return 1;
    if(_LeftEncoderASet && !_LeftEncoderBSet) return -1;
  }else if(!_LeftEncoderAPrev && _LeftEncoderBPrev){
    if(!_LeftEncoderASet && !_LeftEncoderBSet) return 1;
    if(_LeftEncoderASet && _LeftEncoderBSet) return -1;
  }else if(!_LeftEncoderAPrev && !_LeftEncoderBPrev){
    if(_LeftEncoderASet && !_LeftEncoderBSet) return 1;
    if(!_LeftEncoderASet && _LeftEncoderBSet) return -1;
  }else if(_LeftEncoderAPrev && !_LeftEncoderBPrev){
    if(_LeftEncoderASet && _LeftEncoderBSet) return 1;
    if(!_LeftEncoderASet && !_LeftEncoderBSet) return -1;
  }
  return 0;
}

// Logic for right encoder count
long ParseEncoder_R(){
  if(_RightEncoderAPrev && _RightEncoderBPrev){
    if(!_RightEncoderASet && _RightEncoderBSet) return 1;
    if(_RightEncoderASet && !_RightEncoderBSet) return -1;
  }else if(!_RightEncoderAPrev && _RightEncoderBPrev){
    if(!_RightEncoderASet && !_RightEncoderBSet) return 1;
    if(_RightEncoderASet && _RightEncoderBSet) return -1;
  }else if(!_RightEncoderAPrev && !_RightEncoderBPrev){
    if(_RightEncoderASet && !_RightEncoderBSet) return 1;
    if(!_RightEncoderASet && _RightEncoderBSet) return -1;
  }else if(_RightEncoderAPrev && !_RightEncoderBPrev){
    if(_RightEncoderASet && _RightEncoderBSet) return 1;
    if(!_RightEncoderASet && !_RightEncoderBSet) return -1;
  }
  return 0;
}

//Saturation function (x>1 or x<-1)
float saturation (float x){
  if (x>1)
    return 1;
    else if (x<-1)
    return -1;
    else 
    return x;
}

//Timer Function
ISR(TIMER2_OVF_vect) {  
  //TCNT2 = tcnt2;  // reload the timer
  //t++;

  /*if(count_enc<buf_size)
  {
    right_encoder_vec[count_enc] = _RightEncoderTicks;
    left_encoder_vec[count_enc]  = _LeftEncoderTicks;
    count_enc++;
  }
  else
  {

    memcpy(right_encoder_vec,right_encoder_vec+sizeof(int),sizeof(int)*(buf_size-1));
    memcpy(left_encoder_vec,left_encoder_vec+sizeof(int),sizeof(int)*(buf_size-1));
    right_encoder_vec[buf_size-1] = _RightEncoderTicks;
    left_encoder_vec[buf_size-1]  = _LeftEncoderTicks;
  }

  Right_Nticks = right_encoder_vec[count_enc-1] - right_encoder_vec[count_enc-2];
  Left_Nticks = left_encoder_vec[count_enc-1] - left_encoder_vec[count_enc-2];

  D_S_r=((float)Right_Nticks/400)*(M_PI*0.1016);
  D_S_l=((float)Left_Nticks/400)*(M_PI*0.1016);

  x+=(D_S_r+D_S_l)/2*cos(theta);
  y+=(D_S_r+D_S_l)/2*sin(theta);
  theta+=(D_S_r-D_S_l)/0.365;
  
  enc_prev = count_enc - vel_window - 1;
  if(enc_prev<0)
    enc_prev = 0;

  Right_Nticks = right_encoder_vec[count_enc-1] - right_encoder_vec[enc_prev];
  Left_Nticks = left_encoder_vec[count_enc-1] - left_encoder_vec[enc_prev];

  dt = (float)(count_enc - enc_prev - 1)/100;

  Left_vrad_l=(((float)Left_Nticks*2*M_PI)/400)/dt; //velocity in radians per second
  Right_vrad_l=(((float)Right_Nticks*2*M_PI)/400)/dt; //velocity in radians per second   
  */
}
