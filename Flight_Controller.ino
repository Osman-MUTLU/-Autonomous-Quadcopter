#include "Wire.h"
#include <Servo.h>

/* ARDUINO NANO QUADCOPTER SCHEMA https://electronoobs.com/images/Robotica/tut_5/flight_controller_schematic.png */

/* RECEIVER values*/
unsigned long timer_1, timer_2, timer_3, timer_4,timer_5,timer_6, current_time;
int receiver_input[7];
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4,last_channel_5, last_channel_6;

/* Timers */ 
unsigned long loop_timer;
/* Motors*/
Servo M1,M2,M3,M4;
int m1_pow =0;
int m2_pow =0;
int m3_pow =0;
int m4_pow =0;
int throttle = 1500;
int minThrottle = 1200;
boolean start = false;
/*  GYRO DEĞİŞKENLERİ MPU6050  */
long gyroX , gyroY, gyroZ;
int cal_int;
float rotX, rotY, rotZ;
float calX,calY,calZ;
float gyro_pitch_input,gyro_yaw_input,gyro_roll_input;
int counter = 0;

/* ROLL PID */

float pid_roll_kp=1.3; //DEFAULT 1.3
float pid_roll_ki=0.05; //DEFAULT 0.05
float pid_roll_kd=15;  //DEFAULT 15

/* PITCH PID */

float pid_pitch_kp=1.3; //DEFAULT 1.3
float pid_pitch_ki=0.05; //DEFAULT 0.05
float pid_pitch_kd=15;  //DEFAULT 15

/* YAW PID */

float pid_yaw_kp=4.0; //DEFAULT 4.0
float pid_yaw_ki=0.02; //DEFAULT 0.02
float pid_yaw_kd=0;  //DEFAULT 0

/* PID VERIABLES */
float pid_roll_error,pid_roll_error_prev,pid_roll_error_mem;
float pid_roll_output;
float pid_roll_setpoint=0;
float pid_pitch_error,pid_pitch_error_prev,pid_pitch_error_mem;
float pid_pitch_output;
float pid_pitch_setpoint=0;
float pid_yaw_error,pid_yaw_error_prev,pid_yaw_error_mem;
float pid_yaw_output;
float pid_yaw_setpoint=0;


void setup() {
  /*SENSOR SIGNAL*/
  Wire.begin();
  /*SERIAL MONITOR*/
  Serial.begin(9600);
  /* RECEIVER SIGNALS */
  PCICR |= (1 << PCIE2);                                                    //Set PCIE0 to enable PCMSK2 scan.
  PCMSK2 |= (1 << PCINT21);                                                  //Set PCINT0 (digital input 5) to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT20);                                                  //Set PCINT1 (digital input 4)to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT18);                                                  //Set PCINT2 (digital input 2)to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT22);                                                  //Set PCINT3 (digital input 6)to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT23);                                                  //Set PCINT0 (digital input 7) to trigger an interrupt on state change.
  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                   //Set PCINT0 (digital input 8) to trigger an interrupt on state change.

  /* ARDUINO MOTOR CONTROL WITH ESC https://www.youtube.com/watch?v=uOQk8SJso6Q */
  M1.attach(3,1000,2000);  // Top Left
  M2.attach(10,1000,2000); // Top Right
  M3.attach(9,1000,2000);  // Bottom Right
  M4.attach(11,1000,2000); // Bottom Left
  
  setupMPU();
  loop_timer = micros(); 
}

void loop() {
  recordGyroRegisters();
  read_pwm();
  // printMPU();
  // printReceiver();
  PID();
  Motor_Control_Algorithm();
}


/* RECEIVER READING */
/* Pin Change Interruptins ISR for reading PWM signals https://www.youtube.com/watch?v=ZDtRWmBMCmw */
void printReceiver(){
  Serial.print(" PITCH = ");
  Serial.print(receiver_input[1]); // CH1
  Serial.print("    ROLL = ");
  Serial.print(receiver_input[2]); // CH2
  Serial.print("    Throttle = "); 
  Serial.print(receiver_input[3]); // CH3
  Serial.print("    YAW = ");
  Serial.print(receiver_input[4]); // CH4
  Serial.print("    AUX1 = ");
  Serial.print(receiver_input[5]); // CH5
  Serial.print("    AUX2 = ");
  Serial.print(receiver_input[6]); // CH6
  Serial.println();
}
void read_pwm(){
  if(receiver_input[1]<1000) receiver_input[1] = 1000;
  else if(receiver_input[1]>2000) receiver_input[1] = 2000;
  if(receiver_input[2]<1000) receiver_input[2] = 1000;
  else if(receiver_input[2]>2000) receiver_input[2] = 2000;
  if(receiver_input[3]<1000) receiver_input[3] = 1000;
  else if(receiver_input[3]>2000) receiver_input[3] = 2000;
  if(receiver_input[4]<1000) receiver_input[4] = 1000;
  else if(receiver_input[4]>2000) receiver_input[4] = 2000;
  if(receiver_input[5]<1000) receiver_input[5] = 1000;
  else if(receiver_input[5]>2000) receiver_input[5] = 2000;
  if(receiver_input[6]<1000) receiver_input[6] = 1000;
  else if(receiver_input[6]>2000) receiver_input[6] = 2000;
}
ISR(PCINT2_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PIND & B00100000){                                                     //Is input 5 high?
    if(last_channel_1 == 0){                                                //Input 5 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1){                                             //Input 5 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    receiver_input[1] = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if(PIND & B00010000 ){                                                    //Is input 4 high?
    if(last_channel_2 == 0){                                                //Input 4 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if(last_channel_2 == 1){                                             //Input 4 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    receiver_input[2] = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if(PIND & B00000100 ){                                                    //Is input 2 high?
    if(last_channel_3 == 0){                                                //Input 2 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if(last_channel_3 == 1){                                             //Input 2 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input[3] = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }
  //Channel 4=========================================
  if(PIND & B01000000 ){                                                    //Is input 6 high?
    if(last_channel_4 == 0){                                                //Input 6 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_4 == 1){                                             //Input 6 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    receiver_input[4] = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
  //Channel 5=========================================
  if(PIND & B10000000 ){                                                    //Is input 7 high?
    if(last_channel_5 == 0){                                                //Input 7 changed from 0 to 1.
      last_channel_5 = 1;                                                   //Remember current input state.
      timer_5 = current_time;                                               //Set timer_5 to current_time.
    }
  }
  else if(last_channel_5 == 1){                                             //Input 7 is not high and changed from 1 to 0.
    last_channel_5 = 0;                                                     //Remember current input state.
    receiver_input[5] = current_time - timer_5;                             //Channel 5 is current_time - timer_5.
  }
}
ISR(PCINT0_vect){
  //Channel 6=========================================
  if(PINB & B00000001 ){                                                    //Is input 8 high?
    if(last_channel_6 == 0){                                                //Input 8 changed from 0 to 1.
      last_channel_6 = 1;                                                   //Remember current input state.
      timer_6 = current_time;                                               //Set timer_6 to current_time.
    }
  }
  else if(last_channel_6 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    last_channel_6 = 0;                                                     //Remember current input state.
    receiver_input[6] = current_time - timer_6;                             //Channel 6 is current_time - timer_6.
  }
}

/* MOTOR MIXING ALGORTIHM https://www.youtube.com/watch?v=hGcGPUqB67Q&list=PLPNM6NzYyzYqMYNc5e4_xip-yEu1jiVrr&index=1 */ 
void Motor_Control_Algorithm(){
  m1_pow = throttle - pid_roll_output - pid_pitch_output + pid_yaw_output;
  m2_pow = throttle + pid_roll_output - pid_pitch_output - pid_yaw_output;
  m3_pow = throttle + pid_roll_output + pid_pitch_output + pid_yaw_output;
  m4_pow = throttle - pid_roll_output + pid_pitch_output - pid_yaw_output;
  // Arka poz m3,m4 + pitch
  // Sağ poz 2,3 + roll
  // Saatin tersi poz 1,3 + yaw
  
  if(start){
    if(m1_pow>2000){
      m1_pow=2000;
    }
    else if(m1_pow<minThrottle){
      m1_pow=minThrottle;
    }
    
    if(m2_pow>2000){
      m2_pow=2000;
    }
    else if(m2_pow<minThrottle){
      m2_pow=minThrottle;
    }
    
    if(m3_pow>2000){
      m3_pow=2000;
    }
    else if(m3_pow<minThrottle){
      m3_pow=minThrottle;
    }
    
    if(m4_pow>2000){
      m4_pow=2000;
    }
    else if(m4_pow<minThrottle){
      m4_pow=minThrottle;
    }
    while(micros() - loop_timer < 4000); 
    loop_timer = micros(); 
    M1.writeMicroseconds(m1_pow);
    M2.writeMicroseconds(m2_pow);
    M3.writeMicroseconds(m3_pow);
    M4.writeMicroseconds(m4_pow);
  }
  else{
    while(micros() - loop_timer < 4000); 
    loop_timer = micros(); 
    M1.writeMicroseconds(1000);
    M2.writeMicroseconds(1000);
    M3.writeMicroseconds(1000);
    M4.writeMicroseconds(1000);
  }
}

/* PID Control video: (YMFC-3D by Joop Brokking) https://www.youtube.com/watch?v=JBvnB0279-Q */
void PID(){
  /* ROLL PID */
  pid_roll_error = gyro_roll_input - pid_roll_setpoint;
  pid_roll_error_mem += pid_roll_error;
  if(pid_roll_error_mem >400) pid_roll_error_mem = 400;
  if(pid_roll_error_mem <-400) pid_roll_error_mem = -400;
  pid_roll_output = pid_roll_error*pid_roll_kp + pid_roll_error_mem * pid_roll_ki + (pid_roll_error - pid_roll_error_prev)*pid_roll_kd;
  if(pid_roll_output >400) pid_roll_output=400;
  if(pid_roll_output <-400) pid_roll_output=-400;
  pid_roll_error_prev = pid_roll_error;

  /* PITCH PID */
  pid_pitch_error = gyro_pitch_input - pid_pitch_setpoint;
  pid_pitch_error_mem += pid_pitch_error;
  if(pid_pitch_error_mem >400) pid_pitch_error_mem = 400;
  if(pid_pitch_error_mem <-400) pid_pitch_error_mem = -400;
  pid_pitch_output = pid_pitch_error*pid_pitch_kp + pid_pitch_error_mem * pid_pitch_ki + (pid_pitch_error - pid_pitch_error_prev)*pid_pitch_kd;
  if(pid_pitch_output >400) pid_pitch_output=400;
  if(pid_pitch_output <-400) pid_pitch_output=-400;
  pid_pitch_error_prev = pid_pitch_error;

  /* YAW PID */
  pid_yaw_error = gyro_yaw_input - pid_yaw_setpoint;
  pid_yaw_error_mem += pid_yaw_error;
  if(pid_yaw_error_mem >400) pid_yaw_error_mem = 400;
  if(pid_yaw_error_mem <-400) pid_yaw_error_mem = -400;
  pid_yaw_output = pid_yaw_error*pid_yaw_kp + pid_yaw_error_mem * pid_yaw_ki + (pid_yaw_error - pid_yaw_error_prev)*pid_yaw_kd;
  if(pid_yaw_output >400) pid_yaw_output=400;
  if(pid_yaw_output <-400) pid_yaw_output=-400;
  pid_yaw_error_prev = pid_yaw_error;

}
  
void printMPU(){
  counter++;
  if(counter%100==0){
    Serial.print("PITCH =   ");
    Serial.print(gyro_pitch_input);// Ön negatif(-), Arka pozitif(+)
    Serial.print("    Roll =   ");
    Serial.print(gyro_roll_input); // Sol negatif(-), Sağ pozitif(+)
    Serial.print("    Z =   ");
    Serial.print(gyro_yaw_input); // Saat yönü negatif(-), Saatin tersi (+)
    Serial.println();
    counter=0;
  }
}

/* MPU650 READING DATA AND CALCULATING ERRORS https://www.youtube.com/watch?v=UxABxSADZ6U  */
void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
  Serial.print("Reading MPU 6050 .");
  for(cal_int = 0;cal_int<2000;cal_int++){
    recordGyroRegisters();
    if(cal_int %200 == 0)Serial.print(".");
    calX += rotX;
    calY += rotY; 
    calZ += rotZ;
    delayMicroseconds(1000);
    delay(3);
  }
  Serial.println();
  calX /= 2000;
  calY /= 2000;
  calZ /= 2000;
}

void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
  if(cal_int == 2000){
    rotX -= calX;
    rotY -= calY; 
    rotZ -= calZ;
  }
}
void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((rotX) * 0.3);
  gyro_roll_input = (gyro_roll_input * 0.7) + ((rotY) * 0.3);
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((rotZ) * 0.3);
}
