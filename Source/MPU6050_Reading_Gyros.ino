#include "Wire.h"
long gyroX , gyroY, gyroZ;
int cal_int;
float rotX, rotY, rotZ;
float calX,calY,calZ;
float gyro_pitch_input,gyro_yaw_input,gyro_roll_input;
int counter = 0;


void setup() {
  // put your setup code here, to run once:
  /*SENSOR SIGNAL*/
  /*  X = PITCH
      Y = ROLL
      Z = YAW
      
    If you want to change the directions of the movement direction, 
    change the rotations.
    Go to the bottom of the code...
  */
   
  Wire.begin();
  /*SERIAL MONITOR*/
  Serial.begin(9600);
  setupMPU();
}

void loop() {
  // put your main code here, to run repeatedly:
  recordGyroRegisters();
  printMPU();
}
void printMPU(){
  counter++;
  if(counter%100==0){
    Serial.print("PITCH =   ");
    Serial.print(gyro_pitch_input);// Ön negatif(-), Arka pozitif(+)
    Serial.print("    Roll =   ");
    Serial.print(gyro_roll_input); // Sol negatif(-), Sağ pozitif(+)
    Serial.print("    yaw =   ");
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

  /* If you want to change the directions of the movement direction, 
     change the rotations.!!! */
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((rotX / 65.5) * 0.3);
  gyro_roll_input = (gyro_roll_input * 0.7) + ((rotY / 65.5) * 0.3);
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((rotZ / 65.5) * 0.3);
}
