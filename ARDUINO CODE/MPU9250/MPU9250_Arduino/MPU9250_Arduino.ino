#include <Wire.h>
#include "I2C.h"

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
void setup(){
  Wire.begin();
  I2CwriteByte(MPU9250_ADDRESS, 0x6B, 0); // PWR_MGMT_1 register. Set to zero (wakes up the MPU-6050)
  Serial.begin(9600);
}

void loop(){ 
 uint8_t DatAcelGyroTemp[14];
 I2Cread(MPU9250_ADDRESS, 0x3B, 14, DatAcelGyroTemp);
 AcX =(DatAcelGyroTemp[0] << 8 | DatAcelGyroTemp[1]); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
 AcY =(DatAcelGyroTemp[2] << 8 | DatAcelGyroTemp[3]); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
 AcZ =(DatAcelGyroTemp[4] << 8 | DatAcelGyroTemp[5]); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
 Tmp =(DatAcelGyroTemp[6] << 8 | DatAcelGyroTemp[7]); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
 GyX =(DatAcelGyroTemp[8] << 8 | DatAcelGyroTemp[9]); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
 GyY =(DatAcelGyroTemp[10] << 8 | DatAcelGyroTemp[11]); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
 GyZ =(DatAcelGyroTemp[12] << 8 | DatAcelGyroTemp[13]); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

// Serial.print("AcX = "); Serial.print(AcX);
// Serial.print(" | AcY = "); Serial.print(AcY);
// Serial.print(" | AcZ = "); Serial.print(AcZ);
// Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53); //equation for temperature in degrees C from datasheet
// Serial.print(" | GyX = "); Serial.print(GyX);
// Serial.print(" | GyY = "); Serial.print(GyY);
// Serial.print(" | GyZ = "); Serial.println(GyZ);
String Cadena;
Cadena = String(AcX) + " " + String(AcY) + " "+String(AcZ);
Serial.println(Cadena);
 delay(333);
}
