//GND - GND
//VCC - VCC
//SDA - Pin A4
//SCL - Pin A5
 
#include <Wire.h>

/*  DIRECCIONES I2C (A CONTINUCIÓN)  */
#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C


/*  MASCARAS E INFORMACION (A CONTINUACIÓN)  */
#define   GYRO_FSCALE_MASK        0b11100111   // Register 27-0x1B [4:3 bit]
#define   GYRO_FSCALE_250_DPS     0x00
#define   GYRO_FSCALE_500_DPS     0x08
#define   GYRO_FSCALE_1000_DPS    0x10
#define   GYRO_FSCALE_2000_DPS    0x18
#define   ACC_FSCALE_MASK       0b11100111  //Register 28-0x1C [4:3 bit]
#define   ACC_FSCALE_2_G        0x00  
#define   ACC_FSCALE_4_G        0x08
#define   ACC_FSCALE_8_G        0x10
#define   ACC_FSCALE_16_G       0x18

#define   MAG_FSCALE_MASK       0b11101111  //Register 0xA [4 bit]
#define   MAG_FSCALE_14_bit     0x00
#define   MAG_FSCALE_16_bit     0x08
#define   MAG_MODE_MASK       ob11110000  //Register 0xA [3:0 bit]
#define   MAG_MODE_2          0x06


/*  REGISTROS IMPORTANTES (A COTINUACIÓN)  */
#define   ACC_GYRO_DATA   0x3B   //Leer 14 registros (big endian): 
           //accel(1-6) Temp(7-8) gyro(8-14)    Primer byte HIGH, 2nd byte LOW
#define   MAG_DATA        0x03   //Leer 7 registros(Two complement big endian):
              //Mag(1-6) Status(7)        Primer byte LOW, 2nd byte HIGH

 
//Funcion auxiliar lectura
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
   Wire.beginTransmission(Address);
   Wire.write(Register);
   Wire.endTransmission();
 
   Wire.requestFrom(Address, Nbytes);
   uint8_t index = 0;
   while (Wire.available())
      Data[index++] = Wire.read();
}
 
 
// Funcion auxiliar de escritura
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
   Wire.beginTransmission(Address);
   Wire.write(Register);
   Wire.write(Data);
   Wire.endTransmission();
}
 
 
void setup()
{
   Wire.begin();
   Serial.begin(115200);
 
   // Configurar acelerometro
   I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FSCALE_2_G);
   // Configurar giroscopio
   I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FSCALE_250_DPS);
   // Configurar magnetometro
   I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);
   I2CwriteByte(MAG_ADDRESS, 0x0A, MAG_MODE_2);
}
 
 
void loop()
{
   // ---  Lectura acelerometro y giroscopio --- 
   uint8_t Acc_Gyro[14];
   I2Cread(MPU9250_ADDRESS, ACC_GYRO_DATA, 14, Acc_Gyro);
 
   // Convertir registros acelerometro
   int16_t ax = (Acc_Gyro[0] << 8 | Acc_Gyro[1]);
   int16_t ay = (Acc_Gyro[2] << 8 | Acc_Gyro[3]);
   int16_t az = Acc_Gyro[4] << 8 | Acc_Gyro[5];
 
   // Convertir registros giroscopio
   int16_t gx = (Acc_Gyro[8] << 8 | Acc_Gyro[9]);
   int16_t gy = (Acc_Gyro[10] << 8 | Acc_Gyro[11]);
   int16_t gz = Acc_Gyro[12] << 8 | Acc_Gyro[13];
 
 
 
   // ---  Lectura del magnetometro --- 
   uint8_t ST1;
   do
   {
      I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
   } while (!(ST1 & 0x01));
 
   uint8_t Mag[7];
   I2Cread(MAG_ADDRESS, MAG_DATA, 7, Mag);
 
 
   // Convertir registros magnetometro
   int16_t mx = (Mag[3] << 8 | Mag[2]);
   int16_t my = (Mag[1] << 8 | Mag[0]);
   int16_t mz = (Mag[5] << 8 | Mag[4]);
 
 
   // --- Mostrar valores ---
 
   // Acelerometro
//   Serial.print(ax, DEC);
//   Serial.print("\t");
//   Serial.print(ay, DEC);
//   Serial.print("\t");
//   Serial.print(az, DEC);
//   Serial.print("\t");
// 
//   // Giroscopio
//   Serial.print(gx, DEC);
//   Serial.print("\t");
//   Serial.print(gy, DEC);
//   Serial.print("\t");
//   Serial.print(gz, DEC);
//   Serial.print("\t");
 
 
   // Magnetometro
   Serial.print(mx + 200, DEC);
   Serial.print(" ");
   Serial.print(my - 70, DEC);
   Serial.print(" ");
   Serial.println(mz - 700, DEC);
   
   delay(1000);    
}
