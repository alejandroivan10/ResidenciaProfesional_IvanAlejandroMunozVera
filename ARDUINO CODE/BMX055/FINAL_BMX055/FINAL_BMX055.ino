//GND - GND
//VCC - VCC
//SDA - Pin A4
//SCL - Pin A5
//PS  - VCC (I2C)

//SDA(MOSI) - Pin D11
//AD0(MISO) - Pin D12
//SCL(SCK)  - Pin D13
//CSB1(CS)  - Pin D7 (arbitrario)
//CSB2(CS)  - Pin D8 (arbitrario)
//CSB3(CS)  - Pin D9 (arbitrario)
//PS - GND (SPI)

/*
 * INDISPENSABLES:
 *  + Definir un protocolo de comunicación: #define "SPI_PROTOCOL" o "I2C_PROTOCOL"
 *  + Definir un sensor: #define "BMX055"
*/
#define I2C_PROTOCOL
#define BMX055
#define SPEED_TEST
uint8_t CSB1 = 7;  // Chip Select
uint8_t CSB2 = 8;
uint8_t CSB3 = 9;

#if defined(I2C_PROTOCOL)
  #undef SPI_PROTOCOL
  #include <Wire.h>
  #define MARGread(Addr,CS,Reg,nBytes,Data) I2Cread(Addr,Reg,nBytes,Data)
  #define MARGwriteByte(Addr,CS,Reg,Data) I2CwriteByte(Addr,Reg,Data)
  #define MARGwriteNBytes(Addr,CS,Reg,nBytes,Data) I2CwriteNBytes(Addr,Reg,nBytes,Data)
  
  #define MAGread(Addr,CS,Reg,nBytes,Data) I2Cread(Addr,Reg,nBytes,Data)
  #define MAGwriteByte(Addr,CS,Reg,Data) I2CwriteByte(Addr,Reg,Data)
  #define MAGwriteNBytes(Addr,CS,Reg,nBytes,Data) I2CwriteNBytes(Addr,Reg,nBytes,Data)
#elif defined(SPI_PROTOCOL)
  #undef I2C_PROTOCOL
  #include <SPI.h>
  #define MARGread(Addr,CS,Reg,nBytes,Data) SPIread(CS,Reg,nBytes,Data)
  #define MARGwriteByte(Addr,CS,Reg,Data) SPIwriteByte(CS,Reg,Data)
  #define MARGwriteNBytes(Addr,CS,Reg,nBytes,Data) SPIwriteNBytes(CS,Reg,nBytes,Data)
  
  #define MAGread(Addr,CS,Reg,nBytes,Data) SPIreadMg(Addr,CS,Reg,nBytes,Data)
  #define MAGwriteByte(Addr,CS,Reg,Data) SPIwriteByteMg(Addr,CS,Reg,Data)
  #define MAGwriteNBytes(Addr,CS,Reg,nBytes,Data) SPIwriteNBytesMg(Addr,CS,Reg,nBytes,Data)
#else
 #warning "No Protocol has been selected"
#endif


#include "Complementos.h"

#define Addr_Accl 0x19  // BMX055 Accl I2C address is 0x19(25)
#define Addr_Gyro 0x69  // BMX055 Gyro I2C address is 0x69(105)
#define Addr_Mag  0x13  // BMX055 Mag I2C address is 0x13(19)

String sAUX;
uint8_t data[0x55];

/////////////////   Complementos.h   //////////////////////

uint8_t Nmuestras = 3;
Mean Ac_mean(Nmuestras), Gy_mean(Nmuestras), Ma_mean(Nmuestras);

void setup()
{
  Serial.begin(115200);
#if defined(I2C_PROTOCOL)
  // Initialise I2C communication as MASTER
  Wire.begin();
  Wire.setClock(1000000);
  //Serial.println("Protocolo I2C Seleccionado");
#elif defined(SPI_PROTOCOL)
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  pinMode(CSB1, OUTPUT);
  digitalWrite(CSB1, HIGH);
  pinMode(CSB2, OUTPUT);
  digitalWrite(CSB2, HIGH);
  pinMode(CSB3, OUTPUT);
  digitalWrite(CSB3, HIGH);
  //Serial.println("Protocolo SPI Seleccionado");
#endif  
  delay(10);


// SPEED_TEST code
///*
  uint8_t nBytes = 2;
#if defined(SPEED_TEST) 
//#error "Uncommented Section"
  uint8_t Dato[2] = {58,74};
  uint8_t Data[2];
  uint32_t t_SPIw;
  uint32_t t_SPIr;
  uint32_t t_I2Cw;
  uint32_t t_I2Cr;
  uint8_t Addr = Addr_Accl;
  uint8_t Reg = 0x38;
  
 #if defined(SPI_PROTOCOL)
  while(true){
    uint8_t index = 0;
    t_SPIw = micros();
    while (index < nBytes){
      digitalWrite(CSB1, LOW);
      SPI.transfer(Reg+index & 0b01111111); //Primer bit en "0" para Escritura
      SPI.transfer(Dato[index++]);
      digitalWrite(CSB1, HIGH);
    }
    t_SPIw = micros() - t_SPIw;
    t_SPIr = micros();
    for(uint8_t Reg=0x38; Reg<=0x3C; Reg++){       //Reg<=125
      Data[0]=66;
      nBytes = 1;
        digitalWrite(CSB1, LOW);
        SPI.transfer(Reg | 0b10000000); //Primer bit en "1" para Lectura
        uint8_t index = 0;
        while (index < nBytes){
          Data[index++] = SPI.transfer(0x00);
        }
        digitalWrite(CSB1, HIGH); 
      sAUX = "Registro #"+String(Reg)+" =  "+String(Data[0]);
      Serial.println(sAUX);       
    }
    t_SPIr = micros() - t_SPIr;
    sAUX = "Time_Wr = "+String(t_SPIw)+"    Time_Rd = "+String(t_SPIr);
    Serial.println(sAUX);
    Serial.println();
    delay(10000);
  }
  
 #elif defined(I2C_PROTOCOL)
  while(true){
    nBytes = 2;
    t_I2Cw = micros();
    for(int i=0; i<nBytes; i++){
      Wire.beginTransmission(Addr);
      Wire.write(Reg+i);
      Wire.write(Dato[i]); 
      Wire.endTransmission(true);   
    }
    t_I2Cw = micros() - t_I2Cw;
    nBytes = 1;
    for(uint8_t Reg=0x38; Reg<=0x3C; Reg++){       //Reg<=125
      Data[0]=66;
        Wire.beginTransmission(Addr);
        Wire.write(Reg);
        Wire.endTransmission(true);
        t_I2Cr = micros();
        Wire.requestFrom(Addr, nBytes);
        t_I2Cr = micros() - t_I2Cr;
        uint8_t index = 0;
        while (Wire.available()){
           Data[index++] = Wire.read();
        }
      sAUX = "Registro #"+String(Reg)+" =  "+String(Data[0]);
      Serial.println(sAUX);
    }
    sAUX = "Time_Wr = "+String(t_I2Cw)+"    Time_Rd = "+String(t_I2Cr);
    Serial.println(sAUX);
    Serial.println();
    delay(10000);
  }
 #endif
#endif
//*/

    
  MARGwriteByte(Addr_Accl, CSB1, 0x0F, 0x03);  // Range = +/- 2g
  MARGwriteByte(Addr_Accl, CSB1, 0x10, 0x0F);  // Bandwidth = 1000 Hz
  MARGwriteByte(Addr_Accl, CSB1, 0x11, 0x00);  // Normal mode, Sleep duration = 0.5ms
  
  MARGwriteByte(Addr_Gyro, CSB2, 0x0F, 0x03);  // Full scale = +/- 250 degree/s
  MARGwriteByte(Addr_Gyro, CSB2, 0x10, 0x02);  // ODR = 1000 Hz  *Value=130
  MARGwriteByte(Addr_Gyro, CSB2, 0x11, 0x00);  // Normal mode, Sleep duration = 2ms
  
  MARGwriteByte(Addr_Mag, CSB3, 0x4B, 0x83);  // Soft reset
  MARGwriteByte(Addr_Mag, CSB3, 0x4E, 0x84);  // X, Y, Z-Axis enabled
  MARGwriteByte(Addr_Mag, CSB3, 0x51, 0x04);  // No. of Repetitions for X-Y Axis = 9
  MARGwriteByte(Addr_Mag, CSB3, 0x52, 0x0F);  // No. of Repetitions for Z-Axis = 15

  //Corroborate that the past code was written on the chip
///*
  for(uint8_t Reg=0x0F; Reg<=0x11; Reg++){
      MARGread(Addr_Accl, CSB1, Reg, 1, &data[0]);
      sAUX = "Registro #"+String(Reg)+" =  "+String(data[0]);
      Serial.println(sAUX);
  }
  for(uint8_t Reg=0x0F; Reg<=0x11; Reg++){
      MARGread(Addr_Gyro, CSB2, Reg, 1, &data[0]);
      sAUX = "Registro #"+String(Reg)+" =  "+String(data[0]);
      Serial.println(sAUX);
  }
  for(uint8_t Reg=0x4B; Reg<=0x52; Reg++){
      MARGread(Addr_Mag, CSB3, Reg, 1, &data[0]);
      sAUX = "Registro #"+String(Reg)+" =  "+String(data[0]);
      Serial.println(sAUX);
  }
  Serial.println();
  while(true){}
//*/

  do{
    MARGwriteByte(Addr_Mag, CSB3, 0x4C, 0x00);  // Normal Mode, ODR = 10 Hz
    MARGread(Addr_Mag, CSB3, 0x4C, 1, &data[0]);
    Serial.println(data[0]);
    delay(10);
  }while(data[0] != 0 );
  
//  MARGread(Addr_Accl, CSB1, 0x00, 1, &data[0]);
//  sAUX = "CHIP_ID_ACC =  "+String(data[0]);
//  Serial.println(sAUX);
//  MARGread(Addr_Gyro, CSB2, 0x00, 1, &data[0]);
//  sAUX = "CHIP_ID_GYR =  "+String(data[0]);
//  Serial.println(sAUX);
//  MARGread(Addr_Mag, CSB3, 0x40, 1, &data[0]);
//  sAUX = "CHIP_ID_MAG =  "+String(data[0]);
//  Serial.println(sAUX);
//  Serial.println("\nAcelerometro: ");
//  for(uint8_t Reg=0;Reg<=0x36;Reg++){
//    MARGread(Addr_Accl, CSB1, Reg, 1, &data[Reg]);
//    sAUX = "Registro #"+String(Reg)+" =  "+String(data[Reg]);
//    Serial.println(sAUX);
//  }
//  Serial.println("\nGiroscopio: ");
//  for(uint8_t Reg=0;Reg<=0x35;Reg++){
//    MARGread(Addr_Gyro, CSB2, Reg, 1, &data[Reg]);
//    sAUX = "Registro #"+String(Reg)+" =  "+String(data[Reg]);
//    Serial.println(sAUX);
//  } 
//  Serial.println("\nMagnetometro: ");
//  for(uint8_t Reg=0x40;Reg<=0x52;Reg++){
//    MARGread(Addr_Mag, CSB3, Reg, 1, &data[Reg-0x40]);
//    sAUX = "Registro #"+String(Reg)+" =  "+String(data[Reg-0x40]);
//    Serial.println(sAUX);
//  } 
}






int ax, ay, az, gx, gy, gz, mx, my, mz;
int16_t mx_ant, my_ant, mz_ant; uint8_t CSSV = 0;; //Count Stagnant Sensor Values
uint32_t t_act, t_ant;
uint32_t t_p;
float t_delta;

float AX,AY,AZ,GX,GY,GZ,MX,MY,MZ;
float AR,AR_ant,delta_AR, GR,GR_ant,delta_GR, MR,MR_ant,delta_MR;
float Acc_Adj[3]  = {1.0025,0.9964,1.0015}, 
      Gyr_Adj[3]  = {1,1,1}, 
      Mag_Adj[3]  = {1.05,1.05,1.06}; 
int8_t Acc_Bias[3] = {15,-24,3},  //{0.145,-0.235,0.025}  * 104.38323
      Gyr_Bias[3] = {0,0,0},
      Mag_Bias[3] = {19,-86,-65};  //{8.15,-26.84,19.1} * 3.1496063
float ACC_FS = 1024,   //  [ LSB per gravity (LSB/g) ] -- 2g
      GYR_FS = 131.2,  //  [ LSB per deg/seg (LSB/(deg/seg)) ] -- 250 deg/seg
      MAG_FS_XY = 0.3174, MAG_FS_Z = 0.1526; //  [ uT per LSB (uT/LSB) ]

void loop()
{
  t_ant = t_act;
  t_act = micros(); t_p = t_act;
  t_delta = float(t_act-t_ant)/1e6;
  
  for (int i = 0; i < 6; i++) {MARGread(Addr_Accl, CSB1, 2+i, 1, &data[i]);}
      // Convert the data to 12-bits
  ax = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
  if (ax > 2047)    {ax -= 4096;}
  ay = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
  if (ay > 2047)    {ay -= 4096;}
  az = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
  if (az > 2047)    {az -= 4096;}

  for (int i = 0; i < 6; i++)  {MARGread(Addr_Gyro, CSB2, 2+i, 1, &data[i]);}
      // Convert the data
  gx = (data[1] * 256) + data[0];
  if (gx > 32767)    {gx -= 65536;}
  gy = (data[3] * 256) + data[2];
  if (gy > 32767)    {gy -= 65536;}
  gz = (data[5] * 256) + data[4];
  if (gz > 32767)    {gz -= 65536;}

  for (int i = 0; i < 6; i++)  {MARGread(Addr_Mag, CSB3, 66+i, 1, &data[i]);}
      // Convert the data
  mx = ((data[1] * 256) + (data[0] & 0xF8)) / 8;
  if (mx > 4095)    {mx -= 8192;}
  my = ((data[3] * 256) + (data[2] & 0xF8)) / 8;
  if (my > 4095)    {my -= 8192;}
  mz = ((data[5] * 256) + (data[4] & 0xFE)) / 2;
  if (mz > 16383)   {mz -= 32768;}
    
           // Convertir registros acelerometro
  AX = float(ax+Acc_Bias[0])*9.81*Acc_Adj[0]/ACC_FS;
  AY = float(ay+Acc_Bias[1])*9.81*Acc_Adj[1]/ACC_FS;
  AZ = float(az+Acc_Bias[2])*9.81*Acc_Adj[2]/ACC_FS;
  Ac_mean.setNewValues(AX,AY,AZ);
  AX = Ac_mean.getMeanX(); AY = Ac_mean.getMeanY(); AZ = Ac_mean.getMeanZ();
  AR_ant = AR;   AR = sqrt(pow(AX,2)+pow(AY,2)+pow(AZ,2)); 
  delta_AR = AR - AR_ant;

           // Convertir registros giroscopio
  GX = float(gx+Gyr_Bias[0])*10*Gyr_Adj[0]/GYR_FS;
  GY = float(gy+Gyr_Bias[1])*10*Gyr_Adj[1]/GYR_FS;
  GZ = float(gz+Gyr_Bias[2])*10*Gyr_Adj[2]/GYR_FS;
  Gy_mean.setNewValues(GX,GY,GZ);
  GX = Gy_mean.getMeanX(); GY = Gy_mean.getMeanY(); GZ = Gy_mean.getMeanZ();
  GR_ant = GR;   GR = sqrt(pow(GX,2)+pow(GY,2)+pow(GZ,2));
  delta_GR = GR - GR_ant;

           // Convertir registros magnetometro
  MX = -float(my+Mag_Bias[1])*Mag_Adj[1]*MAG_FS_XY;
  MY = float(mx+Mag_Bias[0])*Mag_Adj[0]*MAG_FS_XY;
  MZ = float(mz+Mag_Bias[2])*Mag_Adj[2]*MAG_FS_XY;
  Ma_mean.setNewValues(MX,MY,MZ);// MX -= 15.0f
  MX = Ma_mean.getMeanX(); MY = Ma_mean.getMeanY(); MZ = Ma_mean.getMeanZ();
  // OFFSET: X = -38 a 41 uT   Y = -24 a 55 uT   Z = -44 a 39 uT
  MR_ant = MR;   MR = sqrt(pow(MX,2)+pow(MY,2)+pow(MZ,2));
  delta_MR = MR - MR_ant;

  if(mx_ant == mx & my_ant == my & mz_ant == mz){
     CSSV++;
     if(CSSV >= 10){
       //MARG_Reset();
       delay(1);
       //MARG_setUp();
       CSSV = 0;
     }
   }else{
     CSSV = 0;
   }
   mx_ant = mx;  my_ant = my;  mz_ant = mz;
  /* IMPRESIÓN DE VALORES POR PANTALLA  */
               // Acelerometro
  sAUX = "Ac=["+String(AX)+" "+String(AY)+" "+String(AZ)+"] ";
  Serial.print(sAUX);
//  sAUX = "  Magnitud:  "+String(AR)+"  Diff: "+String(100*delta_AR/t_delta);
//  Serial.println(sAUX);
//                // Giroscopio
  sAUX = "Gy=["+String(GX)+" "+String(GY)+" "+String(GZ)+"] ";
  Serial.print(sAUX);
//  sAUX = "  Magnitud:  "+String(GR)+"  Diff: "+String(100*delta_GR/t_delta);
//  Serial.println(sAUX);
                 // Magnetometro
  sAUX = "Mg=["+String(MX)+" "+String(MY)+" "+String(MZ)+"]";
  Serial.print(sAUX);
//  sAUX = "  Magnitud:  "+String(MR)+"  Diff: "+String(100*delta_MR/t_delta);
//  Serial.println(sAUX);  
  Serial.print(" "); Serial.print(t_act);
  Serial.println();
  delay(100);
}

////////////////////////////////////////////////////////////////////////////////////
