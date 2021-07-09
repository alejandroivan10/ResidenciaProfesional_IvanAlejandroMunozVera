#include<Wire.h>

#define Addr_Accl 0x19  // BMX055 Accl I2C address is 0x19(25)
#define Addr_Gyro 0x69  // BMX055 Gyro I2C address is 0x69(105)
#define Addr_Mag 0x13  // BMX055 Mag I2C address is 0x13(19)

String sAUX;
uint8_t data[0x55];

//Funcion auxiliar lectura
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
   Wire.beginTransmission(Address);
   Wire.write(Register);
   Wire.endTransmission();
 
   Wire.requestFrom(Address, Nbytes);
   uint8_t index = 0;
   while (Wire.available())
      Data[index] = Wire.read();
//      Serial.print("NewValue = "+String(Data[index]));
//      Serial.print("  Addr = "+String(Address)+"  Indx = ");
//      Serial.println(index);
      index++;
}
 
// Funcion auxiliar de escritura
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
   Wire.beginTransmission(Address);
   Wire.write(Register);
   Wire.write(Data);
   Wire.endTransmission();
}

void I2CwriteNBytes(uint8_t Address, uint8_t Register, uint8_t* Data, uint8_t Nbytes)
{
   Wire.beginTransmission(Address);
   Wire.write(Register);
   for(int i=0; i<Nbytes; i++){
    Wire.write(Data[i]);
   }
   Wire.endTransmission(true);
}


void setup()
{
  // Initialise I2C communication as MASTER
  Wire.begin();
  Serial.begin(115200);
  //Wire.setClock(100000);
  
  I2CwriteByte(Addr_Accl, 0x0F, 0x03);  // Range = +/- 2g
  I2CwriteByte(Addr_Accl, 0x10, 0x0F);  // Bandwidth = 1000 Hz
  I2CwriteByte(Addr_Accl, 0x11, 0x00);  // Normal mode, Sleep duration = 0.5ms
  
  I2CwriteByte(Addr_Gyro, 0x0F, 0x04);  // Full scale = +/- 125 degree/s
  I2CwriteByte(Addr_Gyro, 0x10, 0x02);  // ODR = 100 Hz  *Value=130
  I2CwriteByte(Addr_Gyro, 0x11, 0x00);  // Normal mode, Sleep duration = 2ms
  
  I2CwriteByte(Addr_Mag, 0x4B, 0x83);  // Soft reset
  I2CwriteByte(Addr_Mag, 0x4E, 0x84);  // X, Y, Z-Axis enabled
  I2CwriteByte(Addr_Mag, 0x51, 0x04); // No. of Repetitions for X-Y Axis = 9
  I2CwriteByte(Addr_Mag, 0x52, 0x0F); // No. of Repetitions for Z-Axis = 15
  I2CwriteByte(Addr_Mag, 0x4C, 0x00);  // Normal Mode, ODR = 10 Hz
  delay(1000);
  
  I2Cread(Addr_Accl, 0x00, 1, &data[0]);
  sAUX = "CHIP_ID_ACC =  "+String(data[0]);
  Serial.println(sAUX);
  I2Cread(Addr_Gyro, 0x00, 1, &data[0]);
  sAUX = "CHIP_ID_GYR =  "+String(data[0]);
  Serial.println(sAUX);
  I2Cread(Addr_Mag, 0x40, 1, &data[0]);
  sAUX = "CHIP_ID_MAG =  "+String(data[0]);
  Serial.println(sAUX);
  Serial.println("\nAcelerometro: ");
  for(uint8_t Reg=0;Reg<=0x36;Reg++){
    I2Cread(Addr_Accl, Reg, 1, &data[Reg]);
    sAUX = "Registro #"+String(Reg)+" =  "+String(data[Reg]);
    Serial.println(sAUX);
  }
  Serial.println("\nGiroscopio: ");
  for(uint8_t Reg=0;Reg<=0x35;Reg++){
    I2Cread(Addr_Gyro, Reg, 1, &data[Reg]);
    sAUX = "Registro #"+String(Reg)+" =  "+String(data[Reg]);
    Serial.println(sAUX);
  } 
  Serial.println("\nMagnetometro: ");
  for(uint8_t Reg=0x40;Reg<=0x52;Reg++){
    I2Cread(Addr_Mag, Reg, 1, &data[Reg-0x40]);
    sAUX = "Registro #"+String(Reg)+" =  "+String(data[Reg-0x40]);
    Serial.println(sAUX);
  } 

  I2CwriteByte(Addr_Accl, 0x3A, 201);
  I2Cread(Addr_Accl, 0x3A, 1, &data[0]);
  sAUX = "Registro 0x3A =  "+String(data[0]);
  Serial.println(sAUX);
}

int xAccl, yAccl, zAccl, xGyro, yGyro, zGyro, xMag, yMag, zMag;
void loop()
{
  for (int i = 0; i < 6; i++) {I2Cread(Addr_Accl, 2+i, 1, &data[i]);}
      // Convert the data to 12-bits
  xAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
  if (xAccl > 2047)
    {xAccl -= 4096;}
  yAccl = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
  if (yAccl > 2047)
    {yAccl -= 4096;}
  zAccl = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
  if (zAccl > 2047)
    {zAccl -= 4096;}

  for (int i = 0; i < 6; i++)  {I2Cread(Addr_Gyro, 2+i, 1, &data[i]);}
      // Convert the data
  xGyro = (data[1] * 256) + data[0];
  if (xGyro > 32767)
    {xGyro -= 65536;}
  yGyro = (data[3] * 256) + data[2];
  if (yGyro > 32767)
    {yGyro -= 65536;}
  zGyro = (data[5] * 256) + data[4];
  if (zGyro > 32767)
    {zGyro -= 65536;}

  for (int i = 0; i < 6; i++)  {I2Cread(Addr_Mag, 66+i, 1, &data[i]);}
      // Convert the data
  xMag = ((data[1] * 256) + (data[0] & 0xF8)) / 8;
  if (xMag > 4095)
    {xMag -= 8192;}
  yMag = ((data[3] * 256) + (data[2] & 0xF8)) / 8;
  if (yMag > 4095)
    {yMag -= 8192;}
  zMag = ((data[5] * 256) + (data[4] & 0xFE)) / 2;
  if (zMag > 16383)
    {zMag -= 32768;}

  // Output data to serial monitor
  Serial.print("Acceleration in X-Axis : ");
  Serial.println(xAccl);
  Serial.print("Acceleration in Y-Axis : ");
  Serial.println(yAccl);
  Serial.print("Acceleration in Z-Axis : ");
  Serial.println(zAccl);
  Serial.print("X-Axis of rotation : ");
  Serial.println(xGyro);
  Serial.print("Y-Axis of rotation : ");
  Serial.println(yGyro);
  Serial.print("Z-Axis of rotation : ");
  Serial.println(zGyro);
  Serial.print("Magnetic field in X-Axis : ");
  Serial.println(xMag);
  Serial.print("Magnetic field in Y-Axis : ");
  Serial.println(yMag);
  Serial.print("Magnetic filed in Z-Axis : ");
  Serial.println(zMag);Serial.println();
  delay(200);
}

////////////////////////////////////////////////////////////////////////////////////
