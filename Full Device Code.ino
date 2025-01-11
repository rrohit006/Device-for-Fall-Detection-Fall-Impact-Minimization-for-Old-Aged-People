#include <Wire.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(9, 10);// SIM800L TXD and RXD pin to arduino pin 9 and 10

const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
boolean fall = false; //stores if a fall has occurred
boolean trigger1 = false; //stores if first trigger (lower threshold) has occurred
boolean trigger2 = false; //stores if second trigger (upper threshold) has occurred
boolean trigger3 = false; //stores if third trigger (orientation change) has occurred
byte trigger1count = 0; //stores the counts past since trigger 1 was set true
byte trigger2count = 0; //stores the counts past since trigger 2 was set true
byte trigger3count = 0; //stores the counts past since trigger 3 was set true
int angleChange = 0;

const int LED = 2;
const int Buzzer = 3;
const int Airbag = 4;

void setup() 
{ 
  mySerial.begin(9600);
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.println("Wrote to IMU");

  pinMode(LED, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(Airbag, OUTPUT);
  
}

void loop() 
{
  mpu_read();
  ax = (AcX - 2050) / 16384.00;
  ay = (AcY - 77) / 16384.00;
  az = (AcZ - 1947) / 16384.00;
  gx = (GyX + 270) / 131.07;
  gy = (GyY - 351) / 131.07;
  gz = (GyZ + 136) / 131.07;
  
  // calculating Amplitude vector for 3 axes
  float Raw_Amp = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
  int Amp = Raw_Amp * 10;  // Multiplied by 10 because values are between 0 to 1
  Serial.println(Amp);



  angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
  Serial.println(angleChange);
  if (Amp >= 11 && (angleChange >= 150) && (angleChange <= 400)) 
  { // if orientation changes remain between 0-10 degrees
    Serial.println(angleChange);

    digitalWrite(LED, HIGH);
    tone(Buzzer, 4000);
    digitalWrite(Airbag, HIGH);
    delay(1500);
    digitalWrite(Airbag, LOW);
    delay(2000);
    digitalWrite(Airbag, HIGH);
    delay(100);
    digitalWrite(Airbag, LOW);
    delay(20000);
    digitalWrite(Airbag, HIGH);
    delay(1500);
    digitalWrite(Airbag, LOW);

    mySerial.println("AT+CMGF=1");    //To send SMS in Text Mode
    delay(100);
    mySerial.println("AT+CMGS=\"+8801736666420\"\r");  // the phone number you are using
    delay(100);
    mySerial.println("ALERT!! Fall Detected!! Please Provide Emergency Assistance");//the content of the message
    delay(100);
    mySerial.println((char)26);//the stopping character
    delay(100);
      
    digitalWrite(LED, LOW);
    noTone(Buzzer);

  }


  
  if (Amp <= 2 && trigger2 == false) 
  { // if AM breaks lower threshold (0.4g)
    trigger1 = true;
    Serial.println("TRIGGER 1 ACTIVATED");
  }
  
  if (trigger1 == true) 
  {
    trigger1count++;
    if (Amp >= 12) 
    { // if AM breaks upper threshold (3g)
      trigger2 = true;
      Serial.println("TRIGGER 2 ACTIVATED");
      trigger1 = false; 
      trigger1count = 0;
    }
  }
  
  if (trigger2 == true) 
  {
    trigger2count++;
    angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
    Serial.println(angleChange);
    if (angleChange >= 30 && angleChange <= 400) 
    { // if orientation changes by between 80-100 degrees
      trigger3 = true; 
      trigger2 = false; 
      trigger2count = 0;
      Serial.println(angleChange);
      Serial.println("TRIGGER 3 ACTIVATED");
    }
  }
  
  if (trigger3 == true) 
  {
    trigger3count++;
    if (trigger3count >= 10) 
    {
      angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
      Serial.println(angleChange);
      if ((angleChange >= 0) && (angleChange <= 10)) 
      { // if orientation changes remain between 0-10 degrees
        fall = true; 
        trigger3 = false; 
        trigger3count = 0;
        Serial.println(angleChange);
      }
      else 
      { // user regained normal orientation
        trigger3 = false; 
        trigger3count = 0;
        Serial.println("TRIGGER 3 DEACTIVATED");
      }
    }
  }
  
  if (fall == true) 
  { // in event of a fall detection
    Serial.println("FALL DETECTED");
    fall = false;
  }
  
  if (trigger2count >= 6) 
  { // allow 0.5s for orientation change
    trigger2 = false; 
    trigger2count = 0;
    Serial.println("TRIGGER 2 DEACTIVATED");
  }
  
  if (trigger1count >= 6) 
  { // allow 0.5s for AM to break upper threshold
    trigger1 = false; 
    trigger1count = 0;
    Serial.println("TRIGGER 1 DEACTIVATED");
  }
  
  delay(100);
}

void mpu_read() 
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);  // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}
