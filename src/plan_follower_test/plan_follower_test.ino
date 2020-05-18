#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <TimerThree.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <e2o/input.h>

/////////////////////////////////Pin initialization/////////////////////////////////////////

#define S_Motor_enable 22
#define S_Motor_clock 11
#define S_Motor_anticlock 12 

#define B_Motor_pwm 8
#define B_Motor_dir 9     //HIGH,up   LOW, DOWN

#define encoder0PinA 2     
#define encoder0PinB 3
#define encoder0Pinindex A8
#define index_offset 0

#define request_encoder 10

#define hallPinA 19
#define hallPinB 18

///////////////////////////mechanical constraint initialization///////////////////////////////

#define turns 2                    //////8192 (2048*2*2)
#define onerev 2048

#define pi 3.141592
#define R 0.27    // TO BE MEASURED
#define wb 1.3   // TO BE MEASURED

#define brake_delay1 (int)700
#define brake_delay2 (int)1600


//////////////////////////////Variable initialization/////////////////////////////////////////

int timer_freq=10000;
double time_step=(double)timer_freq/1000000;


bool int_flag=0, spin_flag=0, msg_flag=0;

unsigned int dac_flag = 0, brake_flag=0;

unsigned long long int brake_delay_count=0;

volatile int encoder0Pos = 0;             
unsigned int tmp = 0, tmp_index = 0, Aold = 0, Bnew = 0, indexnew = 0, indexold = 0, index = 0;
int temp_encoder_pos=0, encoder_offset=0;

volatile int hallPos = 0;             
unsigned int h_Aold = 0, h_Bnew = 0, hall_tmp ;
double hall_input=0.0;

double kp=1, ki=0.004, kd=0.08;
double kp_acc=5, ki_acc=0.045, kd_acc=0.40;

double encoder_input=0.00;
unsigned long int S_Motor_speed=0;
bool S_Motor_direction=0;// 0 is clockwise and 1 is anticlockwise

double braking=0, acceleration=0, steering=0;
bool kill_switch=0;

double steering_abs_error=0, steering_diff_error=0, steering_cum_error=0,steering_old_error=0;
double bracc_abs_error=0, bracc_diff_error=0, bracc_cum_error=0,bracc_old_error=0;
double steering_pid_output=0,bracc_pid_ouput=0;

double tim=0, velocity=0, steering_angle=0;
int mode=0;
int n=0, i=0, z=0;
unsigned long int tim_new=0, tim_old=0;
double vel_out=0, cur_vel=0, tar_vel=0, vel_err=0, prev_vel_err=0, vel_err_diff=0, accum_vel_err=0;


Adafruit_MCP4725 dac;
unsigned long int dac_count=0,prev_acceleration=0,dac_rate_count=0;

int dac_rate=200;

double del_ticks=0, mean_vel=0;


/////////////////////Ros setup///////////////////////////////////////

ros::NodeHandle nh1;
void get_commands(const e2o::input& msg)
{
    nh1.loginfo("hi");
    msg_flag=1;
    tim=msg.time;
    velocity=msg.velocity;
    steering_angle=msg.steering_angle;
    mode=msg.mode;

  steering = steering_angle*100/35;
  tar_vel = velocity;
  
//    steering_ang[i]= (float) atan(((lr+lf)/lr)*tan(asin((lr*v_d_theta[i][2])/(tim[i]*v_d_theta[i][0]))));
//    steering_ang[i]= (float) steering_ang[i]*(180/3.1415)*(135/35);
    
    if(mode==4)
      {
        braking = 80;
        velocity=0;
        steering=0;
      }  
    else
      braking = 0;  
    
}
 
ros::Subscriber<e2o::input> sub("cmd_msg", &get_commands);



///////////////////////////////Setup pins//////////////////////////////////////////////////////


void setup() 
{

  Timer3.initialize(timer_freq);
  Timer3.attachInterrupt(get_input); 
  
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(request_encoder, OUTPUT);
  digitalWrite(request_encoder,LOW);
  
  pinMode(S_Motor_enable, OUTPUT);  
  
  pinMode(S_Motor_clock, OUTPUT);
  pinMode(S_Motor_anticlock, OUTPUT);

  digitalWrite(S_Motor_enable, LOW);
  digitalWrite(S_Motor_clock, LOW);
  digitalWrite(S_Motor_anticlock, LOW);  
  
  pinMode(B_Motor_pwm, OUTPUT);
  pinMode(B_Motor_dir, OUTPUT);
  digitalWrite(B_Motor_pwm, LOW);
  digitalWrite(B_Motor_dir, LOW);
  
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);
  
  pinMode(encoder0Pinindex, INPUT);
  digitalWrite(encoder0Pinindex, HIGH);

  pinMode(hallPinA, INPUT);
  digitalWrite(hallPinA, HIGH);
  pinMode(hallPinB, INPUT);
  digitalWrite(hallPinB, HIGH);
  
  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);
    // index pin on interrupt 1 (pin 21)
  //attachInterrupt(4, doEncoderindex, CHANGE);
  // set up the Serial Connection

//  attachInterrupt(5, dohall_A, CHANGE);
//  attachInterrupt(4, dohall_B, CHANGE);
  


  dac.begin(0x60);

  //Serial.begin(57600);
  Serial2.begin(115200);
  
  
  nh1.initNode();
  nh1.subscribe(sub);
  nh1.spinOnce();
}


void loop(void)
{ 

  //if(!kill_switch_status())
  {
    if(msg_flag==1)
    {
      //msg_flag=0;
      
      //nh1.loginfo("msg flag");
      
      if(int_flag==1)
      {
        //nh1.spin();
        int_flag=0;
        get_encoder_hall_position();
        velocity_calculation();
        //nh1.loginfo("int flag");
        
//      Serial.print(steering);
//      Serial.print(',');
//      Serial.println(encoder_input);

      }
        
      act_control();

    }
    
    //nh1.spinOnce();
    
    if(spin_flag==1)
    {
        spin_flag=0;
        nh1.spinOnce();
    }
    
  }
//  else
//  {
//    Serial.println("kill!!"); 
//    
//    if(millis() - dac_rate_count >= dac_rate)
//    {
//      dac.setVoltage(0, false);
//      dac_rate_count = millis();
//    }
//    digitalWrite(S_Motor_enable, LOW);
//    digitalWrite(S_Motor_clock, LOW);
//    digitalWrite(S_Motor_anticlock, LOW);  
//    digitalWrite(B_Motor_dir, LOW);
//    analogWrite(B_Motor_pwm,0);
//    
//  }


}



void get_input()
{
   
    int_flag=1;
    spin_flag++;  
  
}


void velocity_calculation()
{
   digitalWrite(request_encoder,HIGH);
   char buffer1[] = {' ',' ',' ',' ',' ',' ',' '}; // Receive up to 4 bytes
   char buffer2[] = {' ',' ',' ',' ',' ',' ',' '}; // Receive up to 4 bytes
   char buffer3[] = {' ',' ',' ',' ',' ',' ',' '}; // Receive up to 4 bytes
   
   while (!Serial2.available()); // Wait for characters
   Serial2.readBytesUntil(',', buffer1, 7);
   int incomingValue1 = atoi(buffer1);
   
   Serial2.readBytesUntil(',', buffer1, 7);
   int incomingValue1 = atoi(buffer1);

   Serial2.readBytesUntil(',', buffer1, 7);
   int incomingValue1 = atoi(buffer1);
   
   
   //Serial.println(buffer);    
   mean_vel = (double)incomingValue/1000;
   digitalWrite(request_encoder,LOW);
   cur_vel=mean_vel;

   if(cur_vel>0) 
    nh1.loginfo("velocity ONNNN*****");
  
}

void act_control()
{ 
  
  steering_control();
  braking_control();
    
  if(millis() - dac_rate_count >= dac_rate)
  {
      
      if(tar_vel==cur_vel)
        nh1.loginfo("hi5");
      accelerate();
      //dac.setVoltage(1100,false);     
      dac_rate_count = millis();
  }  
}

void accelerate() 
{  
    nh1.loginfo("**********");
    
    vel_err=tar_vel-cur_vel;
    vel_err_diff=vel_err-prev_vel_err;

    if(abs(vel_err)>0.05)
    {
      accum_vel_err=accum_vel_err+vel_err;
    }
    
    vel_out=kp_acc*(vel_err)+ki*(accum_vel_err)+kd*(vel_err_diff);
    dac_count = map(vel_out, 0, 5.56, 900, 2048);
    
    if(dac_count>=1500)
      dac_count = 1500;
    
    dac.setVoltage(dac_count, false);
    
    prev_vel_err=vel_err;
    
  //  dac.setVoltage(50, false);

//    if (tar_vel==2)
//    {
//      dac.setVoltage(900,false);
//      nh1.loginfo("vel");
//    }
//



//    dac_count = map(acceleration, 0, 100, 700, 2048);
//    //Serial.println("hello");
//    dac.setVoltage(dac_count, false);

}

void braking_control()
{
   if((braking>-60 && braking<60) && brake_flag!=0)
   {
    int delay_rev=0;
      if(brake_flag==1)
      {
       delay_rev = brake_delay1;
       }
       else if(brake_flag==2)
       {
        delay_rev = brake_delay2;
       }
       if(brake_flag!=0)
       { 
        digitalWrite(B_Motor_dir, LOW);
        analogWrite(B_Motor_pwm,255);
       }
       else
      {
        digitalWrite(B_Motor_dir, HIGH);
        analogWrite(B_Motor_pwm,0);
      }
       if(millis()-brake_delay_count >= delay_rev)
      {
        brake_flag=0;
        
      }
      if(brake_flag==0)
      {
        brake_delay_count = millis();
      }

   }
   else if ((braking<-60) && brake_flag!=2)
   {
    acceleration = 0;
//      brake_delay_count = millis();
      if(brake_flag!=1)
      {
      digitalWrite(B_Motor_dir, HIGH);
      analogWrite(B_Motor_pwm,255);
      }
      else
      {
        digitalWrite(B_Motor_dir, HIGH);
      analogWrite(B_Motor_pwm,0);
      }
      if(millis()-brake_delay_count >= brake_delay1)
      {
        brake_flag=1;
        
      }
      if(brake_flag==1)
      {
        brake_delay_count = millis();
      }
      
   }
   else if ((braking>60) && brake_flag!=1)
   {
    acceleration = 0;
    if(brake_flag!=2)
    {
      digitalWrite(B_Motor_dir, HIGH);
      analogWrite(B_Motor_pwm,255);
    }
    else
    {
      digitalWrite(B_Motor_dir, HIGH);
      analogWrite(B_Motor_pwm,0);
    }
      if(millis()-brake_delay_count >= brake_delay2)
      {
        brake_flag=2;
        
      }
      if(brake_flag==2)
      {
        brake_delay_count = millis();
      }
   } 
   else if (brake_flag==0)
   {
      digitalWrite(B_Motor_dir, LOW);
      analogWrite(B_Motor_pwm,0);   
      brake_delay_count = millis();
   }
    

   
   
Serial.println(brake_flag);                  
      
} 


void steering_control()
{
    steering_pid();
    //S_Motor_speed=(abs(steering_pid_output)/(kp*100))*255;

    
    S_Motor_speed=(abs(steering_pid_output));
    
    if (S_Motor_speed>245)
    {
      S_Motor_speed=245;
    }
    
    //Serial.println(S_Motor_speed);
    if(S_Motor_direction==0)
    {
    digitalWrite(S_Motor_enable, HIGH);     
    digitalWrite(S_Motor_anticlock, LOW);
    analogWrite(S_Motor_clock, S_Motor_speed);  
    }

    else
    {
    //Serial.println(S_Motor_speed);
    digitalWrite(S_Motor_enable, HIGH);     
    digitalWrite(S_Motor_clock, LOW);
    analogWrite(S_Motor_anticlock, S_Motor_speed);   
    }

}




//////////////////////////PID///////////////////////

void steering_pid()
{
  
  double steering_error=(steering-encoder_input);
  //Serial.println(steering_abs_error);

  if (steering_error>0)
  {
    S_Motor_direction=0;
  }
  else
  {
    S_Motor_direction=1;
  }

  steering_abs_error = map(steering_error, -100, 100, -255, 255);
  
  steering_diff_error = steering_abs_error - steering_old_error;
  
  if(abs(steering_abs_error)>0.1)
  {
    steering_cum_error=steering_cum_error+steering_abs_error;
  }
  else 
  {
    steering_cum_error=0;
  }
  
  steering_pid_output=kp*steering_abs_error+kd*steering_diff_error+ki*(steering_cum_error);
  steering_old_error=steering_abs_error;

  if(steering_pid_output > 255)
     steering_pid_output = 255;
  else if(steering_pid_output < -255)
     steering_pid_output = -255;
  
    
    
}



/////////////encoder position calculation/////////

void get_encoder_hall_position()
{
    //Check each changes in position
  //nh1.loginfo("encoder function");
  if (tmp != encoder0Pos) 
  {
    //Serial.println(encoder0Pos, DEC);
    tmp = encoder0Pos;
    
    encoder_input = (encoder0Pos);//*(100/4096));
    encoder_input = -(encoder_input - encoder_offset);
    encoder_input = map(encoder_input, 0, 4096, 0, 100);
//    Serial.print("Encoder_input = ");
//    Serial.println(encoder_input);

  }
  
//  if (tmp_index!=index){
//    //Serial.print("index=");
//    //Serial.println(index);
//    tmp_index=index;
//    //encoder0Pos = 0;             /////////
//  }


//  if(hall_tmp != hallPos)
//  {
//    hall_tmp = hallPos;   
//    hall_input = map(hallPos, 0, 1365, 0, 100);
//  }

}


  
//////encoder Interrupt on A changing state///////////

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


// Interrupt on A changing state
void dohall_A() {
  h_Bnew^h_Aold ? hallPos++ : hallPos--;
  h_Aold = digitalRead(hallPinA);
}
// Interrupt on B changing state
void dohall_B() {
  h_Bnew = digitalRead(hallPinB);
  h_Bnew^h_Aold ? hallPos++ : hallPos--;
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



bool kill_switch_status()
{
//  if(sbus.getNormalizedChannel(5)>0)
//    kill_switch=1;
//  else
    
    if(mode==10)
    {
      kill_switch=1;
    }
    
  return kill_switch;    
} 





