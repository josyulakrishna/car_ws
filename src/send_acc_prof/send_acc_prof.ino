#include <EnableInterrupt.h>
#include <TimerThree.h>
#include <TimerOne.h> 
#define pi 3.141592
#define R 0.27    // TO BE MEASURED
#define wb 1.3   // TO BE MEASURED

#define encoder0PinA 18
#define encoder0PinB 19
#define encoder0Pinindex 20

#define encoder1PinA 3
#define encoder1PinB 2
#define encoder1Pinindex 21

#define request_encoder 10

bool flag=0, int_flag=0,alpha_flag=0;
int timer_freq=10000;
int acc_freq=10000;
double acc_time=(double)acc_freq/1000000;
double time_step=(double)timer_freq/1000000;
double thetao=0;
double theta=0;
double xo=0;
double x=0;
double yo=0;
double y=0;
double total_dist=0;
double old_vel=0;
double new_vel=0;
double alpha=0;
String data = "";

int del0=0,del1=0;
double theta0=0,theta1=0,omega0=0,omega1=0,vel0=0,vel1=0, mean_vel=0,mean_omega=0,dist0=0,dist1=0,distC=0;

double del_ticks=0;

//--------Encoder0------------//
volatile long int encoder0Pos = 0,prev0Pos = 0,temp_encoder_pos=0;
unsigned int tmp = 0;
unsigned int Aold = 0;
unsigned int Bnew = 0;
unsigned int indexnew = 0;
unsigned int indexold = 0;
unsigned int tmp_index0=0,index0 = 0;

//-----------Encoder1-----------//
volatile long int encoder1Pos = 0, prev1Pos = 0,temp_encoder_pos1=0 ;
unsigned int Aold1 = 0;
unsigned int Bnew1 = 0;
unsigned int indexnew1 = 0;
unsigned int indexold1 = 0;
unsigned int tmp_index1 = 0,index1=0;
////////////////////


double encoder_input=0,delf=0,beta=0;

void setup() 
{

  
  pinMode(request_encoder, INPUT_PULLUP);
  enableInterrupt(request_encoder, interruptFunction, RISING);
  
  
//  Timer3.initialize(timer_freq);
//  Timer3.attachInterrupt(calc_trajectory); 
  Timer3.initialize(acc_freq);
  Timer3.attachInterrupt(calc_alpha);

//----------- Encoder 0-----------------//
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA,HIGH);
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB,HIGH);
  pinMode(encoder0Pinindex, INPUT);
  digitalWrite(encoder0Pinindex,HIGH);

  enableInterrupt(encoder0PinA, doEncoderA0, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  enableInterrupt(encoder0PinB, doEncoderB0, CHANGE);
    // index pin on interrupt 1 (pin 21)
  enableInterrupt(encoder0Pinindex, doEncoderindex0, CHANGE);



//--------- Encoder 1 -------------------//
  pinMode(encoder1PinA, INPUT);
  digitalWrite(encoder1PinA,HIGH);
  pinMode(encoder1PinB, INPUT);
  digitalWrite(encoder1PinB,HIGH);
  pinMode(encoder1Pinindex, INPUT);
  digitalWrite(encoder1Pinindex,HIGH);

  
  enableInterrupt(encoder1PinA, doEncoderA1, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  enableInterrupt(encoder1PinB, doEncoderB1, CHANGE);
    // index pin on interrupt 1 (pin 21)
  enableInterrupt(encoder1Pinindex, doEncoderindex1, CHANGE);
 
  Serial.begin (230400);
  Serial3.begin (115200);
  
}

void loop() 
{

  

  if(int_flag==1)
  {
    int_flag=0;
    Serial.println("hello");
    String send_data = data;
    Serial3.println(send_data);
  }

///

 
    
  del0 = (encoder0Pos - prev0Pos); // del ticks     
  del1 = (encoder1Pos - prev1Pos);
  prev0Pos = encoder0Pos;
  prev1Pos = encoder1Pos;
  
  vel0 = ((double)del0/2048)*2*pi*R/time_step; 
  vel1 = ((double)del1/2048)*2*pi*R/time_step;  

  mean_vel=(vel0+vel1)/2;
 
  
}

void interruptFunction() 
{ 
   //////////////use flag//////////////////
   int_flag=1;
   
}


void calc_alpha()
{
  alpha_flag=0;

  new_vel = mean_vel;
  alpha = (new_vel-old_vel)/acc_time;
  alpha_flag=1;
  String new_vel_data = String(new_vel*1000);
  String old_vel_data = String(old_vel*1000);
  old_vel = mean_vel;
  String alpha_data = String(alpha*1000);
  data = String(new_vel_data + "," + old_vel_data + "," + alpha_data);



}


//----------- Encoder 0------------------//
// Interrupt on A changing state
void doEncoderA0()
{
  Bnew^Aold ? encoder0Pos++ : encoder0Pos--;
  Aold = digitalRead(encoder0PinA);
  
}

// Interrupt on B changing state
void doEncoderB0() 
{
  Bnew = digitalRead(encoder0PinB);
  Bnew^Aold ? encoder0Pos++ : encoder0Pos--;
}

void doEncoderindex0()
{
  indexnew=digitalRead(encoder0Pinindex);
  
  if((abs(encoder0Pos-temp_encoder_pos)>200) || index0==0)
  {
    if (indexnew>indexold)
  
    {
      index0++;
      temp_encoder_pos = encoder0Pos;
      
//      Serial.print("index0 = ");
//      Serial.println(index0); 
  
         
//        prev0Pos = 0;  
//        encoder0Pos=0;


      if(index0==1)
      {
        prev0Pos = 0;  
        encoder0Pos=0;
        
      }
      else if((index0-1)%10==0)
      {
        prev0Pos = 20480-encoder0Pos;
        encoder0Pos = 0;        
      }
              
    }
  }
  indexold=indexnew;
 
}

//----------- Encoder 1----------------//
// Interrupt on A changing state
void doEncoderA1() 
{
  Bnew1^Aold1 ? encoder1Pos++ : encoder1Pos--;
  Aold1 = digitalRead(encoder1PinA);
  
}
// Interrupt on B changing state
void doEncoderB1() 
{
  Bnew1 = digitalRead(encoder1PinB);
  Bnew1^Aold1 ? encoder1Pos++ : encoder1Pos--;
}

void doEncoderindex1()
{
  indexnew1=digitalRead(encoder1Pinindex);
  
  if((abs(encoder1Pos-temp_encoder_pos1)>200) || index1==0)
  {
    if (indexnew1>indexold1)
  
    {
      index1++; 
      temp_encoder_pos1= encoder1Pos;

//      Serial.print("index1 = "); //encoder1Pos = 0; 
//      Serial.println(index1);
      
//        prev1Pos = 0;  
//        encoder1Pos=0;
      
      
      if(index1==1)
      {
        prev1Pos = 0;  
        encoder1Pos=0;
        
      }
      else if((index1-1)%10==0)
      {
        prev1Pos = 20480-encoder1Pos;
        encoder1Pos = 0;        
      }
            
    }
  }
  indexold1=indexnew1;

}
