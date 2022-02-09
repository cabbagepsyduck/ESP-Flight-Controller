
/***********************************************************************
Description: Application for QUAD controller using ESP8266 WwiFi module.

Version: 1.0
Revision record: Initial version forQuad controller
Date: 01.10.2021

************************************************************************/

#include<Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
WiFiUDP UDP;

//////////////////// Define wifi access point credential
////////////////////////////////////////////////////////
#ifndef STASSID
#define STASSID "FTPL"      
#define STAPSK  "singh321"
#endif

///////////////////////////////// Define network details
////////////////////////////////////////////////////////
WiFiUDP Udp;
unsigned int localPort = 8787;      // local port to listen on

IPAddress local_IP(192,168,1,50);
IPAddress gateway(192,168,1,1);
IPAddress Subnet(255,255,0,0);
IPAddress primaryDNS(8,8,8,8);
IPAddress secondaryDNS(8,8,4,4);

#define UDP_TX_PACKET_MAX_SIZE 32 //increase UDP size
bool CommandLoss = true;

//////////////////////////////// Define global variables
////////////////////////////////////////////////////////
unsigned char  CmdBuffer[UDP_TX_PACKET_MAX_SIZE+1];       // WiFi to UART ( Command buffer)
unsigned char  ReplyBuffer[UDP_TX_PACKET_MAX_SIZE+1];     // UART to WiFi ( Telemetry buffer)
unsigned char LinkRate = 125; // Start with link fail condition

//////////////////////// Ground cmmmand global variables
////////////////////////////////////////////////////////
unsigned short MotorCmd[7]; // 1.125 mSec PWM
int input_PITCH = 1024;
int input_ROLL = 1024;
int input_YAW = 1024;
int input_THROTTLE = 1024;
unsigned char DataBuffer[24];

//////////////////////////////////// Motor PID parameter
////////////////////////////////////////////////////////
float  Kgain = 1.0 ;   // Loop gain parameter
float  Ti = 3.0;        // Integrator time constant
float  Td = 0.2;        // Differentiator time constant
float  delT=0.1;        // Update time interval

int Motor1_PID;
/////////////////////// Global variables
char packet[7];
boolean recvState;

//-----------------------------------------------------------------------//   
int ESCout_1 ,ESCout_2 ,ESCout_3 ,ESCout_4;

int state1,state2,state3,state4;

float Pitch, Roll; 
//-----------------------------------------------------------------------// 
float gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temperature, acc_total_vector;
float angle_pitch, angle_roll, angle_yaw;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;
float elapsedTime;
long Time, timePrev, time2;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;

//-----------------------------------------------------------------------// 
float pitch_PID, roll_PID, yaw_PID;
float roll_desired_angle, pitch_desired_angle, yaw_desired_angle; 
float roll_error, roll_previous_error, pitch_error, pitch_previous_error, yaw_error;
float roll_pid_p, roll_pid_d, roll_pid_i, pitch_pid_p, pitch_pid_i, pitch_pid_d, yaw_pid_p, yaw_pid_i;


double twoX_kp=5;      
double twoX_ki=0.003;
double twoX_kd=2;     
double yaw_kp=3;    
double yaw_ki=0.002;

///////////////////////////////////// Setup I2C sensor module ()
////////////////////////////////////////////////////////////////
void InitI2CModule(void)
{

  // Configure IIC port
  Wire.begin();    
  Wire.setClock(400000);
  Wire.endTransmission();  

  Wire.beginTransmission(0x68);                                        
  Wire.write(0x6B);                                                    
  Wire.write(0x00);                                                  
  Wire.endTransmission();  
         
  Wire.beginTransmission(0x68);   // Setup accelerometer (2g)                                   
  Wire.write(0x1C);                                                   
  Wire.write(0x00);                                                   
  Wire.endTransmission();         

  Wire.beginTransmission(0x68);  // Configure LPF                                      
  Wire.write(0x1A);                                                 
  Wire.write(0x00);                                                   
  Wire.endTransmission();  
  
  Wire.beginTransmission(0x68);  // Gyroscope setting (250 DPS)                                      
  Wire.write(0x1B);                                                 
  Wire.write(0x00);                                                   
  Wire.endTransmission();                                            
}


///////////////////// Setup and initialise application framework
////////////////////////////////////////////////////////////////
void setup() {

  // Setup serial port
  Serial.begin(115200, SERIAL_8N1);     // Set UART baud & attribute
  Serial.setTimeout(24);                // Set for 40 mSec packet
  Serial.println("\nSerial port initialised");
  
  // Set pin direction
  pinMode(D5,OUTPUT); // Motor-1 PWM output
  pinMode(D6,OUTPUT); // Motor-2 PWM output
  pinMode(D7,OUTPUT); // Motor-3 PWM output
  pinMode(D8,OUTPUT); // Motor-4 PWM output 
  
  //analogWriteRange(2222);
  //analogWriteFreq(60); // Min supportd frequency 100 Hz 

  pinMode(LED_BUILTIN, OUTPUT);
  delay(300);
  Serial.printf("DIO configured \n");

  InitI2CModule();  // Initialise I2C
  
  // Establish static IP address
  if(!WiFi.config(local_IP, gateway, Subnet, primaryDNS, secondaryDNS))
  {
    // Keep loopint till connection is established
    delay(50);
    Serial.println("\nSTA Failed");
  }
  else Serial.println("\nSTA connected");

  WiFi.mode(WIFI_STA);          // Set WiFi to station mode
  WiFi.begin(STASSID, STAPSK);  // Start WiFi
  Serial.println("\nWiFi set to STA mode");
  
  // Establish wifi with access point
  Serial.println("\nWaiting to connect AP");
  while (WiFi.status() != WL_CONNECTED) 
  {
    Serial.print('.');
    delay(500);
  }
  Serial.println("\nAP Connected.. ");
  
  Udp.begin(localPort);
  Serial.println("\nUDP Started.. ");

  //UDP.begin(9999);


  
}


/////////////////// OFP main control function for turn, move etc
////////////////////////////////////////////////////////////////
void OFP_PID_Loop()
{

  roll_desired_angle  = 3*((float)input_ROLL  / (float)10 - (float)5);
  pitch_desired_angle = 3*((float)input_PITCH / (float)10 - (float)5);
  //yaw_desired_angle =0;
  
  roll_error  = angle_roll_output  - roll_desired_angle;
  pitch_error = angle_pitch_output - pitch_desired_angle;  
  yaw_error = angle_yaw - yaw_desired_angle;  
    
  roll_pid_p  = twoX_kp * roll_error;
  pitch_pid_p = twoX_kp * pitch_error;
  yaw_pid_p   = yaw_kp  * yaw_error;
  
  if(-3 < roll_error < 3) { roll_pid_i  = roll_pid_i  + (twoX_ki * roll_error);  }
  if(-3 < pitch_error < 3){ pitch_pid_i = pitch_pid_i + (twoX_ki * pitch_error); }
  if(-3 < yaw_error < 3)  { yaw_pid_i   = yaw_pid_i   + (yaw_ki  * yaw_error);   }
  
  roll_pid_d  = twoX_kd * ((roll_error  - roll_previous_error)  / elapsedTime);
  pitch_pid_d = twoX_kd * ((pitch_error - pitch_previous_error) / elapsedTime);
  
  roll_PID    = roll_pid_p  + roll_pid_i  + roll_pid_d;
  pitch_PID   = pitch_pid_p + pitch_pid_i + pitch_pid_d;
  yaw_PID     = yaw_pid_p   + yaw_pid_i;
  
  if(roll_PID < -400) { roll_PID=-400; }
  else if(roll_PID > 400) { roll_PID=400; }
  
  if(pitch_PID < -400) { pitch_PID=-400; }
  else if(pitch_PID > 400) { pitch_PID=400; }
  
  if(yaw_PID < -400) { yaw_PID=-400; }
  else if(yaw_PID > 400) { yaw_PID=400; }
  
  ESCout_1 = input_THROTTLE - roll_PID - pitch_PID - yaw_PID;
  if(ESCout_1 > 2000) ESCout_1 = 2000;
  else if(ESCout_1 < 1100) ESCout_1 = 1100;
    
  ESCout_2 = input_THROTTLE + roll_PID - pitch_PID + yaw_PID;
  if(ESCout_2 > 2000) ESCout_2 = 2000;
  else if(ESCout_2 < 1100) ESCout_2 = 1100;
  
  ESCout_3 = input_THROTTLE + roll_PID + pitch_PID - yaw_PID;
  if(ESCout_3 > 2000) ESCout_3 = 2000;
  else if(ESCout_3 < 1100) ESCout_3 = 1100;
   
  ESCout_4 = input_THROTTLE - roll_PID + pitch_PID + yaw_PID;
  if(ESCout_4 > 2000) ESCout_4 = 2000;
  else if(ESCout_4 < 1100) ESCout_4 = 1100;
  
  roll_previous_error  = roll_error;
  pitch_previous_error = pitch_error;
  
//-----------------------------------------------------------------------//
  while((micros() - Time) < 1000);
  state1 = 1; state2 = 1; state3 = 1; state4 = 1;
  while(state1 == 1 || state2 == 1 || state3 == 1 || state4 == 1){
    time2 = micros();
    if((time2 - Time) >= ESCout_1 && state1 == 1){ GPOC = (1 << 14); state1=0;}
    if((time2 - Time) >= ESCout_2 && state2 == 1){ GPOC = (1 << 12);state2=0;}
    if((time2 - Time) >= ESCout_3 && state3 == 1){ GPOC = (1 << 13);state3=0;}
    if((time2 - Time) >= ESCout_4 && state4 == 1){ GPOC = (1 << 15);state4=0;}
}
//-----------------------------------------------------------------------//
  if(!recvState){
    int packetSize = UDP.parsePacket();
    if (packetSize) {
      int len = UDP.read(packet, 6);
      packet[len] = '\0';    
  if(String(packet[0]) == "a"){
  input_ROLL = int(packet[1]);
  input_PITCH = int(packet[2]); 
  input_THROTTLE = 1000 + int(packet[3]);
  input_YAW = int(packet[4]);
  }
  else if(String(packet[0]) == "b"){
  input_ROLL = int(packet[1]);
  input_PITCH = int(packet[2]); 
  input_THROTTLE = 1000 + int(packet[3])*100 + int(packet[4]);
  input_YAW = int(packet[5]);
  }
  if(String(packet[0]) == "1"){
  twoX_kp = (float)int(packet[1])/(float)100;
  twoX_ki = (float)int(packet[2])/(float)1000; 
  twoX_kd = (float)int(packet[3])/(float)100;
  }  
  else if(String(packet[0]) == "2"){
  twoX_kp = (float)(float)(int(packet[1])*100 + int(packet[2]))/(float)100;
  twoX_ki = (float)(float)(int(packet[3])*100 + int(packet[4]))/(float)1000; 
  twoX_kd = (float)(float)(int(packet[5])*100 + int(packet[6]))/(float)100;
  }    
  else if(String(packet[0]) == "3"){
  twoX_kp = (float)(int(packet[1])*100 + int(packet[2]))/(float)100;
  twoX_ki = (float)int(packet[3])/(float)1000; 
  twoX_kd = (float)int(packet[4])/(float)100;
  }  
  else if(String(packet[0]) == "4"){
  twoX_kp = (float)int(packet[1])/(float)100;
  twoX_ki = (float)(int(packet[2])*100 + int(packet[3]))/(float)1000; 
  twoX_kd = (float)int(packet[4])/(float)100;
  }  
  else if(String(packet[0]) == "5"){
  twoX_kp = (float)int(packet[1])/(float)100;
  twoX_ki = (float)int(packet[2])/(float)1000; 
  twoX_kd = (float)(int(packet[3])*100 + int(packet[4]))/(float)100;
  }    
  else if(String(packet[0]) == "6"){
  twoX_kp = (float)(int(packet[1])*100 + int(packet[2]))/(float)100;
  twoX_ki = (float)(int(packet[3])*100 + int(packet[4]))/(float)1000; 
  twoX_kd = (float)int(packet[5])/(float)100;
  }  
  else if(String(packet[0]) == "7"){
  twoX_kp = (float)int(packet[1])/(float)100;
  twoX_ki = (float)(int(packet[2])*100 + int(packet[3]))/(float)1000; 
  twoX_kd = (float)(int(packet[4])*100 + int(packet[5]))/(float)100;
  } 
  else if(String(packet[0]) == "8"){
  twoX_kp = (float)(int(packet[1])*100 + int(packet[2]))/(float)100;
  twoX_ki = (float)int(packet[3])/(float)1000; 
  twoX_kd = (float)(int(packet[4])*100 + int(packet[5]))/(float)100;
  }
  Serial.print(input_ROLL);Serial.print(" ");
  Serial.print(input_THROTTLE);Serial.print(" ");
  Serial.print(twoX_kp);Serial.print(" ");
  Serial.print(twoX_ki,3);Serial.print(" ");
  Serial.print(twoX_kd);Serial.println();
  }
  }
  else if(recvState){
  UDP.beginPacket(UDP.remoteIP(), UDP.remotePort());
  UDP.print(Time - timePrev);
  UDP.endPacket();
  }
  recvState = !recvState;
  //-----------------------------------------------------------------------//
  Serial.print(angle_roll_output);Serial.print("  ");
  Serial.print(angle_yaw);Serial.print("  ");
  Serial.print(angle_pitch_output);Serial.print(" | ");
  Serial.print(roll_desired_angle);Serial.print("  ");
  Serial.print(pitch_desired_angle);
  Serial.println();
  
}

///////////////// Process IMU data and filter for inherent noise
////////////////////////////////////////////////////////////////
void ProcessImuData(void)
{
static float RawRoll, RawPitch;
static float Alpha=0.5;

  RawPitch = (180.0/3.141)*atan(acc_y/(sqrt(acc_x*acc_x+acc_z*acc_z)));
  RawRoll  = (180.0/3.141)*atan(-(acc_x/acc_z));

  Pitch = RawPitch* Alpha + (1.0 - Alpha)*Pitch;
  Roll =  RawRoll * Alpha + (1.0 - Alpha)*Roll;

  //Serial.print(Roll); Serial.print(" "); Serial.print(Pitch); Serial.print(" "); Serial.print(Motor1_PID); Serial.print("\n"); 

  //Serial.print(acc_x); Serial.print(" "); Serial.print(acc_y); Serial.print(" "); Serial.print(acc_z); Serial.print("\n"); 
}



///////////////////////// Implement individual motor PID control 
////////////////////////////////////////////////////////////////
void Motor_1_PID(float seterr)
{
// Controller state
static float  integral;    // Summation of setpoint errors
static float  deriv;        // Previous setpoint error  

// Operating variables
static float change; 
static float pidout2,pidout3;


  // Proportional + Integral response
  pidout2 = seterr + (integral * delT / Ti);
  integral += seterr;

  // Proportional + Integral + Derivative response
  change = seterr - deriv;
  pidout3 = pidout2 + (change * Td / delT);
  deriv = seterr;

  // drive controller output
  Motor1_PID = -(pidout3*Kgain);

  // Limit output
  if(Motor1_PID < -25) Motor1_PID = -25;
  else if(Motor1_PID > 25) Motor1_PID = 25;

 //Serial.print(seterr); Serial.print(" "); Serial.print(Kgain); Serial.print(" "); Serial.print(Ti); Serial.print(" "); Serial.print(Motor1_PID); Serial.print("\n"); 
 
return;
}
  


////////////// Implement QUAD PID control for QUAD stabilization
////////////////////////////////////////////////////////////////
void QUAD_Motr_PID(void)
{
  // PID Tuning parameter
  Motor_1_PID(0-Pitch);
  
}



/////////////////// Read 9 axis IMU data from IIC port at 250 Hz
////////////////////////////////////////////////////////////////
bool ReadImuData(void)
{
static bool DataStatus = true;
static unsigned char i;

  // Setup read request 
  Wire.beginTransmission(0x68);                                       
  Wire.write(0x3B);                                                  
  Wire.endTransmission();  

  // Read sensor register
  Wire.requestFrom(0x68,14);                                        
  while(Wire.available() < 14);  

  for (i=0; i<14; i++) DataBuffer[i] = Wire.read();
  DataBuffer[14] = 0x55;
  DataBuffer[15] = 0x66;
  //Serial.write(DataBuffer, 16);
  
  acc_x = (short)( (DataBuffer[0] << 8) | DataBuffer[1] ) / 16384.0;                               
  acc_y = (short)( (DataBuffer[2] << 8) | DataBuffer[3] ) / 16384.0;                               
  acc_z = (short)( (DataBuffer[4] << 8) | DataBuffer[5] ) / 16384.0; 
                                  
  temperature = (DataBuffer[6] << 8) | DataBuffer[7];
                           
  gyro_x = (DataBuffer[8]  << 8) | DataBuffer[9];                               
  gyro_y = (DataBuffer[10] << 8) | DataBuffer[11];                                 
  gyro_z = (DataBuffer[12] << 8) | DataBuffer[13];   
                                            
  gyro_x_cal /= 2000;                                                 
  gyro_y_cal /= 2000;                                                 
  gyro_z_cal /= 2000;
 

  return(DataStatus);
}


unsigned short MotorOffsetOffset = 1125;
////// Based on quad requirement generate motor throttle
////////////////////////////////////////////////////////
void GenerateMotorThrottle(void)
{ 
  MotorCmd[0] = input_THROTTLE + MotorOffsetOffset + Motor1_PID;
  MotorCmd[1] = input_THROTTLE + MotorOffsetOffset;
  MotorCmd[2] = input_THROTTLE + MotorOffsetOffset;
  MotorCmd[3] = input_THROTTLE + MotorOffsetOffset;  

  MotorCmd[5] = MotorCmd[0];  // Assume max (Put max value in index 5)
  if(MotorCmd[1] > MotorCmd[5]) MotorCmd[5] = MotorCmd[1];  
  if(MotorCmd[2] > MotorCmd[5]) MotorCmd[5] = MotorCmd[2]; 
  if(MotorCmd[3] > MotorCmd[5]) MotorCmd[5] = MotorCmd[3]; 
  MotorCmd[5] = MotorCmd[5] + 2;  // FOr processing
}

///////////////// Write pwm data at port to drive motors
////////////////////////////////////////////////////////
void UpdatePWM_Data(void)
{
static long MotorCmdLimit[7], Tmicros, i; 

  noInterrupts(); // Disable all interrupt
  Tmicros = micros();
  
  if(Tmicros < 0xFFFF8000)  // Do not process roll over section)
  {   
    for(i=0; i<6; i++) MotorCmdLimit[i] = MotorCmd[i] + Tmicros;
   
    digitalWrite(D5, HIGH); 
    digitalWrite(D6, HIGH);
    digitalWrite(D7, HIGH);
    digitalWrite(D8, HIGH);
  
    do
    {
      Tmicros = micros();
      if(Tmicros > MotorCmdLimit[0]) digitalWrite(D5, LOW);
      if(Tmicros > MotorCmdLimit[1]) digitalWrite(D6, LOW);
      if(Tmicros > MotorCmdLimit[2]) digitalWrite(D7, LOW);
      if(Tmicros > MotorCmdLimit[3]) digitalWrite(D8, LOW);
      
    } while (Tmicros < MotorCmdLimit[5]);
  }
  interrupts(); // Enable all interrupt
}

////// Polled mode => Receive command data from UDP port
////////////////////////////////////////////////////////
void Read_UDP_Data(void) 
{
static unsigned char i,n, ESum, OSum, FrId, PktCnt;
static bool Header, Footer, Checksum, FrID, Athenticity;
static unsigned long LossCount = 0;

  LossCount++;
  
  if (Udp.parsePacket()) 
  {     
      // read the packet into packetBufffer
      n = Udp.read(CmdBuffer, UDP_TX_PACKET_MAX_SIZE);
      if(n>32) n=32;  

      // Calculate packet checksum
      ESum = 0; OSum = 0;
      for(i=0; i<30; i+=2)
      {
        ESum ^= CmdBuffer[i];
        OSum ^= CmdBuffer[i+1];
      }

      // Verify header
      if( (CmdBuffer[0]  == 0xAA) && (CmdBuffer[1]  == 0xBB) ) Header = true; else Header = false;
      
      // Verify footer
      if( (CmdBuffer[28] == 0xCC) && (CmdBuffer[29] == 0xDD) ) Footer = true; else Footer = false;
    
      // Verify checksum
      if( (CmdBuffer[30] == ESum) && (CmdBuffer[31] == OSum) ) Athenticity = true; else Athenticity = false;
             
      // Verify frameID
      if(CmdBuffer[2] == FrId) FrID = true; else FrID = false;

      // Verify command authenticity
      if(Header && Footer && FrID && Athenticity)            
      {   
        LossCount = 0;                                             
        PktCnt++;
        
        input_PITCH    = CmdBuffer[3] | (CmdBuffer[4]  << 8);
        input_ROLL     = CmdBuffer[5] | (CmdBuffer[6]  << 8);
        input_YAW      = CmdBuffer[7] | (CmdBuffer[8]  << 8);
        input_THROTTLE = CmdBuffer[9] | (CmdBuffer[10] << 8);

        Kgain = CmdBuffer[19]/50.0;   // Loop gain parameter
        Ti = CmdBuffer[21]/50.0;      // Integrator time constant
        Td = CmdBuffer[23]/50.0;      // Differentiator time constant
      }
      
      FrId = CmdBuffer[2] + 1;  // Get next FrID     
  }
    
  if(LossCount > 10) CommandLoss = true;
  else CommandLoss = false;
}

/////////////////////////// LED status display indication module
////////////////////////////////////////////////////////////////
void BlinkLED(unsigned char Schedular) 
{
static bool LEDOn=true;

  if(CommandLoss) // Link fail
  {
    if(Schedular < 4) LEDOn = false;
    else LEDOn = true; 
  }
  else LEDOn = false;// Link healthy

  digitalWrite(LED_BUILTIN, LEDOn); // Blink LED  
}


//////// Application main schedular to handle all the processing
////////////////////////////////////////////////////////////////
void loop() 
{
static unsigned char currentMillis, LastMills;
static unsigned long Performance=0;
static bool DataFlag;  

  currentMillis = (millis() >> 4) & 0x07;  // Keep polling timer value (16 mSec resolution)
  Performance++;
  
  if(currentMillis != LastMills)  //  mSec time has lapsed
  {   
    //Serial.print(Performance); Serial.print("\n"); 
    
    LastMills = currentMillis; // reassign
    Performance = 0;
            
    Read_UDP_Data();
      
    DataFlag = ReadImuData();     
    if(DataFlag) ProcessImuData();
    DataFlag = false;


     QUAD_Motr_PID();
     
     GenerateMotorThrottle();
     UpdatePWM_Data();
     
     BlinkLED(currentMillis);  
  }    
}
