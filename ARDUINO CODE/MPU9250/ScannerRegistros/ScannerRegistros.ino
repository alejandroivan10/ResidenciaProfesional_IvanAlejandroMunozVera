#include <Wire.h>

/*  DIRECCIONES I2C (A CONTINUCIÓN)  */
#define    MPU9250_ADDR            0x68
#define    MAG_ADDRESS             0x0C


#define   MAG_FSCALE_MASK       0b11101111  //Register 0xA [4 bit]
#define   MAG_FSCALE_14_bit     0x00
#define   MAG_FSCALE_16_bit     0x08
#define   MAG_MODE_MASK       ob11110000  //Register 0xA [3:0 bit]
#define   MAG_MODE_2          0x06


/*  REGISTROS IMPORTANTES (A COTINUACIÓN)  */
#define GYRO_SELF_TEST   0X00  //Leer 3 registros: X Y Z
#define ACC_SELF_TEST    0X0D  //Leer 3 registros: X Y Z
#define ACC_GYRO_DATA    0x3B   //Leer 14 registros (big endian): 
           //accel(1-6) Temp(7-8) gyro(8-14)    Primer byte HIGH, 2nd byte LOW
#define ACC_GYRO_CONFIG  0x1A   //Configuraciones generales
#define ACC_CONFIG_1     0x1C   //Accel-Config Register #1   
#define ACC_GYRO_RESET   0x6B   //Reset to "0" all Registers 
#define   MAG_DATA         0x03  //Leer 7 registros(Two complement big endian):
           //Mag(1-6) Status(7)        Primer byte LOW, 2nd byte HIGH
#define   MAG_CNTL_1       0x0A  //Registro #1 de configuración del sensor
#define   MAG_ASTC         0x0C  // SELF-TEST Register
#define   MAG_STATUS_1     0x02  //Status register #1 (Data ready - Data Overrun)
#define   MAG_ASAX         0x10  //Sensitivity adjudment values. Leer 3 registros


String sAUX;
//float MAG_FS = 0.6; float ACC_FS = 2*9.81; float GYR_FS = 250;

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
int I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
   Wire.beginTransmission(Address);
   Wire.write(Register);
   Wire.write(Data);
   if(Wire.endTransmission() != 0) return -1;
   else return 0;
}




uint8_t Mag[7];        //Magnetometer DATA
float AX,AY,AZ,GX,GY,GZ,MX,MY,MZ; //9 GDL´s DATA
uint8_t ST1;
const uint8_t ASA[3] = {180, 181, 170};        //Magnetometer sensivility adjudment 
float Mag_ADJ[3];

uint8_t Data;
void setup() {
   Wire.begin();
   Serial.begin(115200);
   //I2CwriteByte(MAG_ADDRESS, MAG_CNTL_1+1, 0x01); //Soft Reset
   delay(5000);

   
//   I2CwriteByte(MAG_ADDRESS, MAG_CNTL_1, 0x00);  //Set -> Power Down Mode
//   delay(50);
//      /* Procedimiento para realizar SELF-TEST Magnetometro(Ver abajo) */
//   I2CwriteByte(MAG_ADDRESS, MAG_ASTC, 0x40); //Magnetic Fiel for SelfTest starts
//   delay(50);
//   I2Cread(MAG_ADDRESS, MAG_DATA, 7, Mag);
//   if(I2CwriteByte(MAG_ADDRESS, MAG_CNTL_1, 0b00001000) == -1){  //Set -> SELF-TEST Mode
//     Serial.println("error");
//   }
//   for(uint8_t Reg=0; Reg<=19; Reg++){
//     I2Cread(MAG_ADDRESS, Reg, 1, &Data);
//     sAUX = "Registro #"+String(Reg)+" =  "+String(Data);
//     Serial.println(sAUX);
//   }
//   Mag_ADJ[0] = ((ASA[0]-128)/256.0f + 1);
//   Mag_ADJ[1] = ((ASA[1]-128)/256.0f + 1);
//   Mag_ADJ[2] = ((ASA[2]-128)/256.0f + 1);
//   
//   do{
//      I2Cread(MAG_ADDRESS, MAG_STATUS_1, 1, &ST1);
//   } while (!(ST1 & 0x01));
//   
//   I2Cread(MAG_ADDRESS, MAG_DATA, 7, Mag);
//   I2CwriteByte(MAG_ADDRESS, MAG_ASTC, 0x00);
//   I2CwriteByte(MAG_ADDRESS, MAG_CNTL_1, 0x00);  //Set -> Power Down Mode
//   delay(50);
//   int16_t mx = (Mag[1] << 8 | Mag[0]);
//   int16_t my = (Mag[3] << 8 | Mag[2]);
//   int16_t mz = (Mag[5] << 8 | Mag[4]);
//   sAUX = "NO SCALE SELF-TEST: "+String(mx)+" "+String(my)+" "+String(mz);
//   Serial.println(sAUX);
//   MX = Mag_ADJ[0]*(float)mx;
//   MY = Mag_ADJ[1]*(float)my;
//   MZ = Mag_ADJ[2]*(float)mz;
//   sAUX = "SCALE SELF-TEST: "+String(MX)+" "+String(MY)+" "+String(MZ);
//   Serial.println(sAUX);
//
//
//   
//   I2CwriteByte(MAG_ADDRESS, MAG_CNTL_1, 0x16);  //Set continuous
//   delay(50);
//   do{
//      I2Cread(MAG_ADDRESS, MAG_STATUS_1, 1, &ST1);
//   } while (!(ST1 & 0x01));
//   I2Cread(MAG_ADDRESS, MAG_DATA, 7, Mag);
//   mx = (Mag[1] << 8 | Mag[0]);
//   my = (Mag[3] << 8 | Mag[2]);
//   mz = (Mag[5] << 8 | Mag[4]);
//   MX = Mag_ADJ[0]*(float)mx;
//   MY = Mag_ADJ[1]*(float)my;
//   MZ = Mag_ADJ[2]*(float)mz;
//   sAUX = "NO SELF-TEST: "+String(MX)+" "+String(MY)+" "+String(MZ);
//   Serial.println(sAUX);
}

void loop() {
  delay(500);
  uint8_t Data;
  for(uint8_t Reg=0; Reg<=126; Reg++){
    I2Cread(MPU9250_ADDR, Reg, 1, &Data);
    sAUX = "Registro #"+String(Reg)+" =  "+String(Data);
    Serial.println(sAUX);
  }
  
  while(true){
    }
}
