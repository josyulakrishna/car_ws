#include <Wire.h>
#include <Adafruit_MCP4725.h>

#include <TimerThree.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <e2o/acc_prof.h>


/////////////////////////////////Pin initialization/////////////////////////////////////////

#define Motor_enable 22
#define Motor_clock 11
#define Motor_anticlock 12 

#define encoder0PinA 2     
#define encoder0PinB 3
#define encoder0Pinindex 19
#define index_offset 0

#define request_encoder 10

///////////////////////////mechanical constraint initialization///////////////////////////////

#define turns 2                    //////8192 (2048*2*2)
#define onerev 2048

//////////////////////////////Variable initialization/////////////////////////////////////////

#define pi 3.141592
#define R 0.27    // TO BE MEASURED
#define wb 1.3   // TO BE MEASURED

int timer_freq=10000;
double time_step=(double)timer_freq/1000000;
bool int_flag=0,spin_flag=0,data_flag=0;


volatile int encoder0Pos = 0;             
unsigned int tmp = 0, tmp_index = 0, Aold = 0, Bnew = 0, indexnew = 0, indexold = 0, index = 0;
int temp_encoder_pos=0, encoder_offset=0;

double kp=1, ki=0.004, kd=0.08;
double kp_acc=5, ki_acc=0.045, kd_acc=0.40;

//double kp=14, ki=0.3, kd=12;
//long int t=0;

double encoder_input=0.00;
unsigned long int motor_speed=0;
bool motor_direction=0;// 0 is clockwise and 1 is anticlockwise

double braking=100, acceleration=0, steering=0;
bool kill_switch=1;

double steering_abs_error=0, steering_diff_error=0, steering_cum_error=0,steering_old_error=0;
double bracc_abs_error=0, bracc_diff_error=0, bracc_cum_error=0,bracc_old_error=0;
double steering_pid_output=0,bracc_pid_ouput=0;
double tim[100];
double velocity[100];
double steering_angle[100];
int n=0;
int i=0,z=0;
unsigned long int tim_new=0,tim_old=0;
double vel_out=0;
double cur_vel=0;
double tar_vel=0;
double vel_err=0;
double prev_vel_err=0;
double vel_err_diff=0;
double accum_vel_err=0;

Adafruit_MCP4725 dac;
unsigned long int dac_count=0,prev_acceleration=0,dac_rate_count=0;

int dac_rate=200;


double del_ticks=0, inst_acc=0,old_vel=0,new_vel=0,old_time=0,new_time=0;

/////////////////////Ros setup///////////////////////////////////////

ros::NodeHandle nh1;
e2o::acc_prof acc_prof_data; 
ros::Publisher pub_val("acc_stat", &acc_prof_data);


///////////////////////////////Setup pins//////////////////////////////////////////////////////

void setup() 
{

  Timer3.initialize(timer_freq);
  Timer3.attachInterrupt(get_input); 
  
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(request_encoder, OUTPUT);
  digitalWrite(request_encoder,LOW);

  
  pinMode(Motor_enable, OUTPUT);  
  pinMode(Motor_clock, OUTPUT);
  pinMode(Motor_anticlock, OUTPUT);
  
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);
  
  pinMode(encoder0Pinindex, INPUT);
  digitalWrite(encoder0Pinindex, HIGH);
  
  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);
    // index pin on interrupt 1 (pin 21)
  attachInterrupt(4, doEncoderindex, CHANGE);
  // set up the Serial Connection
  
  

  dac.begin(0x62);

  
  //Serial.begin(57600);
  Serial2.begin(115200);
  
 
  nh1.initNode();
  nh1.advertise(pub_val);
  
  
  
}



/////////////////////////////////Loop and PID here //////////////////////

void loop()
{  

 if(millis() - dac_rate_count >= dac_rate)
  {
      if(dac_count>=2048)
      {
      dac_count = 2048;
      }

      if (new_time-old_time>10000)
      {
        dac_count=dac_count+20;
        old_time=millis();
      }
            
      
      dac.setVoltage(dac_count, false);;     
      dac_rate_count = millis();
      new_time=millis();
  } 
  
  
 
        
  
}



void velocity_calculation()
{
   digitalWrite(request_encoder,HIGH);
   char buffer[] = {' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};
   while (!Serial2.available()); // Wait for characters
   Serial2.readBytesUntil('/n', buffer, 17);
   String incomingValue = buffer;
   String alpha1 = incomingValue.substring(0,4);
   char alpha2[]={' ',' ',' ',' '};
   alpha1.toCharArray(alpha2,4);
   int alpha = atoi(alpha2);

   String ini_vel1 = incomingValue.substring(5,9);
   char ini_vel2[]={' ',' ',' ',' '};
   ini_vel1.toCharArray(ini_vel2,4);
   int ini_vel = atoi(ini_vel2);

   String fin_vel1 = incomingValue.substring(10);
   char fin_vel2[]={' ',' ',' ',' '};
   fin_vel1.toCharArray(fin_vel2,4);
   int fin_vel = atoi(fin_vel2);



   
 
   digitalWrite(request_encoder,LOW);
  
}



void doEncoderA() 
{
  Bnew^Aold ? encoder0Pos++ : encoder0Pos--;
  Aold = digitalRead(encoder0PinA);
}


////////////Encoder Interrupt on B changing state//////////

void doEncoderB() 
{
  Bnew = digitalRead(encoder0PinB);
  Bnew^Aold ? encoder0Pos++ : encoder0Pos--;
}

///////////Encoder interrupt on index changing state///////////

void doEncoderindex()
{
  indexnew=digitalRead(encoder0Pinindex);
  
  if((abs(encoder0Pos-temp_encoder_pos)>200) || index==0)
  {
    if (indexnew>indexold)
  
    {
      index++; 
      temp_encoder_pos = encoder0Pos;        
    }
  }
  indexold=indexnew;
}


//////////////////////////PID///////////////////////



void get_input(void)
{
//  acc_prof_data.acceleration = inst_acc;
//  acc_prof_data.dac_value = dac_count;
//  acc_prof_data.start_velocity = old_vel;
//  acc_prof_data.end_velocity = new_vel;
//  nh1.spinOnce();

}


