#include <Wire.h>
#include <Adafruit_MCP4725.h>

#include <TimerThree.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <trajectory_plot/input.h>


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


double del_ticks=0, mean_vel=0;

/////////////////////Ros setup///////////////////////////////////////

ros::NodeHandle nh1;
 void assign_val(const trajectory_plot::input& msg)
 {
    n=msg.sections;
    for (int i=0; i<n; i++)
    { 
    tim[i]=msg.time[i];
    velocity[i]=msg.velocity[i];
    steering_angle[i]=msg.steering_angle[i];
    }

    if(data_flag==0)
    {
      section_assign(0);
      data_flag=1;
    }
//    if(velocity[0]==2)
//    {
//      nh1.loginfo("yes");
//    }
 }
 
ros::Subscriber<trajectory_plot::input> sub("mlc_traj_control", &assign_val);



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
  nh1.subscribe(sub);
  
  
  
}



/////////////////////////////////Loop and PID here //////////////////////

void loop()
{  
  if(data_flag==1&&i<=n)
  {
    if(int_flag==1)
    {
      int_flag=0;
      get_encoder_position();
      velocity_calculation();
     
    }  

    
    
    tim_new = millis();

    if((tim_new-tim_old)>(tim[i]*1000))
    {
      i++;
      section_assign(i);
      tim_old = millis();    
    }

    
    
    act_control();

    if(spin_flag==10)
    {
      spin_flag=0;
      nh1.spinOnce();
    }

  }

  else
  {
    tim_old=millis();
    nh1.spinOnce();    

    if(millis() - dac_rate_count >= dac_rate)
    {
      dac.setVoltage(0,false);
      dac_rate_count = millis();
    }  
  
    digitalWrite(Motor_enable, LOW); 
    
  }
  
        
  
}

void act_control()
{
  steering_control();
    
  if(millis() - dac_rate_count >= dac_rate)
  {
      if(tar_vel==cur_vel)
        nh1.loginfo("hi");
      accelerate();
      //dac.setVoltage(1100,false);     
      dac_rate_count = millis();
  }  
}

void velocity_calculation()
{
   digitalWrite(request_encoder,HIGH);
   char buffer[] = {' ',' ',' ',' ',' ',' ',' '}; // Receive up to 4 bytes
   while (!Serial2.available()); // Wait for characters
   Serial2.readBytesUntil('\n', buffer, 7);
   int incomingValue = atoi(buffer);
   //Serial.println(buffer);    
   mean_vel = (double)incomingValue/1000;
   digitalWrite(request_encoder,LOW);
   cur_vel=mean_vel;
  
}

void section_assign(int k)
 {

//      char buffer1[50];
//      int n;
     
      
      steering=steering_angle[k];
      tar_vel=velocity[k];
      nh1.loginfo("yes");

 }



void steering_control()
{
    steering_pid();
    //motor_speed=(abs(steering_pid_output)/(kp*100))*255;
    
    motor_speed=(abs(steering_pid_output));
    
    if (motor_speed>245)
    {
      motor_speed=245;
    }
    
    //Serial.println(motor_speed);
    if(motor_direction==0)
    {
    digitalWrite(Motor_enable, HIGH);     
    digitalWrite(Motor_anticlock, LOW);
    analogWrite(Motor_clock, motor_speed);  
    }

    else
    {
    //Serial.println(motor_speed);
    digitalWrite(Motor_enable, HIGH);     
    digitalWrite(Motor_clock, LOW);
    analogWrite(Motor_anticlock, motor_speed);   
    }

}

void accelerate() 
{

  
    
    vel_err=tar_vel-cur_vel;
    vel_err_diff=vel_err-prev_vel_err;

    if(abs(vel_err)>0.05)
    {
      accum_vel_err=accum_vel_err+vel_err;
    }
    
    vel_out=kp_acc*(vel_err)+ki*(accum_vel_err)+kd*(vel_err_diff);
    dac_count = map(vel_out, 0, 5.56, 900, 2048);
    
    if(dac_count>=2048)
      dac_count = 2048;
    
    dac.setVoltage(dac_count, false);
    
    prev_vel_err=vel_err;
    
  //  dac.setVoltage(50, false);

//    if (tar_vel==2)
//    {
//      dac.setVoltage(900,false);
//      nh1.loginfo("vel");
//    }
// 
}
//////////////////////////PID///////////////////////

void steering_pid()
{
  
  double steering_error=(steering-encoder_input);
  //Serial.println(steering_abs_error);

  if (steering_error>0)
  {
    motor_direction=0;
  }
  else
  {
    motor_direction=1;
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

//void bracc_pid()
//{
//  
//  
//}


/////////////encoder position calculation/////////

void get_encoder_position()
{
    //Check each changes in position
  if (tmp != encoder0Pos) {
    //Serial.println(encoder0Pos, DEC);
    tmp = encoder0Pos;
    
    encoder_input = (encoder0Pos);//*(100/4096));
    encoder_input = -(encoder_input - encoder_offset);
    encoder_input = map(encoder_input, 0, 4096, 0, 100);
//    Serial.print("Encoder_input = ");
//    Serial.println(encoder_input);

  }
  
  if (tmp_index!=index){
    //Serial.print("index=");
    //Serial.println(index);
    tmp_index=index;
    //encoder0Pos = 0;             /////////
  }

}

/////////steering offset calculation/////////////

void get_steering_offset()
{     
      bool offset_flag=0;
      
      while((abs(encoder0Pos)<1024)&&index==0)
      {
        //analogWrite(Motor_pwm, 192);          // Run in half speed      
        digitalWrite(Motor_enable, HIGH);     
        analogWrite(Motor_clock, 192);    
        digitalWrite(Motor_anticlock, LOW);    
        get_encoder_position();  
//        Serial.print("encoderpos");
//        Serial.println(encoder0Pos);
      }

      digitalWrite(Motor_enable, LOW);     
      if(index==1)
         {
          encoder_offset = encoder0Pos - index_offset;
          offset_flag=1;
         }
      
      //Serial.print("exit1 loop: ");
      //Serial.println(index);
      delay(2000);  
      
      //if(index==0)
      {
        
        while(index==0)
        {
          //analogWrite(Motor_pwm, 192);          // Run in half speed      
          digitalWrite(Motor_enable, HIGH);
          analogWrite(Motor_anticlock, 192);
          digitalWrite(Motor_clock, LOW);         
          get_encoder_position();  
//          Serial.print("encoderpos");
//          Serial.println(encoder0Pos);
        }
      }

    if(offset_flag==0)
      encoder_offset = encoder0Pos - index_offset;
    
    digitalWrite(Motor_anticlock, LOW);       
    digitalWrite(Motor_clock, LOW);       
      
    //digitalWrite(Motor_enable, LOW);       
    //get_encoder_position();  
    //Serial.print("encoder_offset=");
    //Serial.println(encoder_offset);
           

    digitalWrite(Motor_anticlock, LOW);
    digitalWrite(Motor_clock, LOW);
    Serial.print("encoder_offset=");
    Serial.println(encoder0Pos);

    
  
  //delay(1);

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




void get_input(void)
{
   
    int_flag=1;
    spin_flag++;  
  
}


