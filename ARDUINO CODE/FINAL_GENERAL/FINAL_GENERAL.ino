//GND - GND
//VCC - VCC
//SDA - Pin A4
//SCL - Pin A5

//SDA(MOSI) - Pin D11
//AD0(MISO) - Pin D12
//SCL(SCK)  - Pin D13
//NCS(CS)   - Pin D7 (arbitrario)

/*
 * INDISPENSABLES:
 *  + Definir un protocolo de comunicación: #define "SPI_PROTOCOL" o "I2C_PROTOCOL"
 *  + Definir un sensor: #define "MPU9250" o "BMX055"
*/
#define I2C_PROTOCOL
#define MPU9250
//#define MAG_PROOF

uint8_t CS = 7;  // Chip Select


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
 #error "No Protocol has been selected"
#endif





#include "Complementos.h"
#include "RegistersAndMask.h"
#include "SetUp.h"

/* VARIABLES GLOBALES */
uint8_t Acc_Gyro[14];  //Accel-Gyro DATA
uint8_t Mag[7];        //Magnetometer DATA
float AX, AY, AZ, GX, GY, GZ, MX, MY, MZ; //9 GDL´s DATA
float AR, GR, MR, AR_ant, GR_ant, MR_ant, delta_AR, delta_GR, delta_MR;
int16_t Acc_Bias[3], Gyr_Bias[3], Mag_Bias[3]; //Sumar Offset
float Acc_Adj[3], Gyr_Adj[3], Mag_Adj[3]; //Multiplicar Offset
uint8_t ST1;           //Magnetometer "Status" value

String sAUX;
float MAG_FS = 0.15;
float ACC_FS = 0.59875e-3; //   (9.81 * 2) / 2^15
float GYR_FS = 7.6294e-3;  //    250 / 2^15

bool flag = false;

/////////////////   Complementos.h   //////////////////////

uint8_t Nmuestras = 3;
Mean Ac_mean(Nmuestras), Gy_mean(Nmuestras), Ma_mean(Nmuestras);

void setup()
{
  Serial.begin(115200);
#if defined(I2C_PROTOCOL)
  Wire.begin();
  Wire.setClock(400000);
  //Serial.println("Protocolo I2C Seleccionado");
#elif defined(SPI_PROTOCOL)
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  //Serial.println("Protocolo SPI Seleccionado");
#endif
  delay(10);


#ifdef SPI_PROTOCOLt
  uint8_t Dato[2] = {77,39};
  uint8_t Data[2];
  while(true){
    SPIwriteNBytes(CS, 21, 2, Dato);  //Gyro_DLPF => 5 Hz
    for(uint8_t Reg=1; Reg<=125; Reg++){
      Data[0]=66;
      SPIread(CS, Reg, 1, Data);
      sAUX = "Registro #"+String(Reg)+" =  "+String(Data[0]);
      Serial.println(sAUX);
    }
    Serial.println();
    delay(10000);
  }
#endif

#if defined(MAG_PROOF)
  MARG_Reset();
  delay(100);
  MARG_setUp();
  uint8_t Dato[2] = {77,39};
  uint8_t Data[2];
  delay(100);
  while(true){
    //MAGwriteByte(MAG_ADDRESS, CS, 0x0C, Dato); //Gyro_DLPF => 5 Hz
    for(uint8_t Reg=0; Reg<=127; Reg++){
      Data[0]=67;
      MARGread(Reg,CS,0x01,1,Data);
      //MAGread(MAG_ADDRESS, CS, Reg, 1, Data);
      //sAUX = "Registro #"+String(Reg)+" =  "+String(Data[0]);
      if(Data[0] != 67){
        sAUX = "Respuesta de la dirección: "+String(Reg);
        Serial.println(sAUX);
      }
    }
    Serial.println();
    delay(10000);
  }
#endif
  
  
  MARG_Reset();
  MARG_setUp();
  // Configurar magnetometro
  //MARGwriteByte(MPU9250_ADDR, CS, 0x37, 0x02); //Registro INT_PIN_CFG

  /* INTERRUPCIÓN POR DESBORDAMIENTO DEL TIMER 2 */
  SREG = (SREG & 0b01111111); //Desabilitar interrupciones
  TIMSK2 = TIMSK2 | 0b00000001; //Habilita la interrupción por desbordamiento
  TCCR2B = 0b00000011; //Preescaler para frecuencia de 250kHz
  SREG = (SREG & 0b01111111) | 0b10000000; //Habilitar interrupciones //Desabilitar interrupciones
  delay(10);



  //   /* Procedimiento para obtener el offset Acel-Giro */
  //   MARGwriteByte(MPU9250_ADDR, CS, 28, ACC_FSCALE_16_G);
  //   MARGwriteByte(MPU9250_ADDR, CS, 27, GYRO_FSCALE_1000_DPS);
  //   MARGwriteByte(MPU9250_ADDR, CS, ACC_GYRO_CONFIG, 0x06);  //Gyro_DLPF => 92 Hz
  //   MARGwriteByte(MPU9250_ADDR, CS, ACC_CONFIG_1+1, 0x06);  //Acc_DLPF => 92 Hz
  //   int32_t AX_OS=0, AY_OS=0, AZ_OS=0, GX_OS=0, GY_OS=0, GZ_OS=0;
  //   bool Offs_Finish = false, Off_Error = false;
  //   uint16_t i_Offs = 0;
  //   uint8_t ErrorAX, ErrorAY, ErrorGX, ErrorGY, ErrorGZ;
  //   //while(!Off_Error){
  //     Offs_Finish = false;
  //     while(!Offs_Finish){
  //       if(flag){
  //         flag = 0;
  //         MARGread(MPU9250_ADDR, CS, ACC_GYRO_DATA, 14, Acc_Gyro);
  //                // Convertir registros acelerometro
  //         int16_t ax = (Acc_Gyro[0] << 8 | Acc_Gyro[1]);
  //         int16_t ay = (Acc_Gyro[2] << 8 | Acc_Gyro[3]);
  //         int16_t az = (Acc_Gyro[4] << 8 | Acc_Gyro[5]);
  //         int16_t gx = (Acc_Gyro[8] << 8 | Acc_Gyro[9]);
  //         int16_t gy = (Acc_Gyro[10] << 8 | Acc_Gyro[11]);
  //         int16_t gz = (Acc_Gyro[12] << 8 | Acc_Gyro[13]);
  //         i_Offs++;
  //         if(i_Offs <= 50){
  //           AX_OS += ax;     AY_OS += ay;     AZ_OS += az;
  //           GX_OS += gx;     GY_OS += gy;     GZ_OS += gz;
  //         }else{
  //           int16_t AXOFF = AX_OS / 200;
  //           int16_t AYOFF = AY_OS / 200;
  //           int16_t AZOFF = AZ_OS / 200;
  //           int16_t GXOFF = GX_OS / 200;
  //           int16_t GYOFF = GY_OS / 200;
  //           int16_t GZOFF = GZ_OS / 200;
  //           sAUX = "ACC: "+String(AXOFF)+" "+String(AYOFF)+" "+String(AZOFF);
  //           Serial.println(sAUX);
  //           sAUX = "GYR: "+String(GXOFF)+" "+String(GYOFF)+" "+String(GZOFF);
  //           Serial.println(sAUX);
  //           sAUX = "ACC_GRAV: "+String(float(AXOFF)/ACC_SENS)+" "+String(float(AYOFF)/ACC_SENS)+" "+String(float(AZOFF)/ACC_SENS);
  //           Serial.println(sAUX);
  //           sAUX = "GYR_DPS: "+String(float(GXOFF)/GYRO_SENS)+" "+String(float(GYOFF)/GYRO_SENS)+" "+String(float(GZOFF)/GYRO_SENS);
  //           Serial.println(sAUX);
  //           if(AZOFF > 0) {AZOFF -= ACC_SENS;}
  //           else {AZOFF += ACC_SENS;}
  //           Acc_Gyro[0] = (-GXOFF >> 8) & 0xFF;
  //           Acc_Gyro[1] = (-GXOFF)      & 0xFF;
  //           Acc_Gyro[2] = (-GYOFF >> 8) & 0xFF;
  //           Acc_Gyro[3] = (-GYOFF)      & 0xFF;
  //           Acc_Gyro[4] = (-GZOFF >> 8) & 0xFF;
  //           Acc_Gyro[5] = (-GZOFF)      & 0xFF;
  //           //MARGwriteNBytes(MPU9250_ADDR, CS, GYRO_OFFSET, 6, Acc_Gyro);
  //           MARGread(MPU9250_ADDR, CS, ACC_OFFSET, 2, Acc_Gyro);
  //           Acc_Bias[0] = (Acc_Gyro[0] << 8 | Acc_Gyro[1]);
  //           MARGread(MPU9250_ADDR, CS, ACC_OFFSET+3, 2, Acc_Gyro);
  //           Acc_Bias[1] = (Acc_Gyro[0] << 8 | Acc_Gyro[1]);
  //           MARGread(MPU9250_ADDR, CS, ACC_OFFSET+6, 2, Acc_Gyro);
  //           Acc_Bias[2] = (Acc_Gyro[0] << 8 | Acc_Gyro[1]);
  //           sAUX = "AC_BIAS: "+String(Acc_Bias[0])+" "+String(Acc_Bias[1])+" "+String(Acc_Bias[2]);
  //           Serial.println(sAUX);
  //           Acc_Bias[0] -= AXOFF;
  //           Acc_Bias[1] -= AYOFF;
  //           Acc_Bias[2] -= AZOFF;
  //           Offs_Finish = true;
  //           delay(1000);
  //         }
  //       }
  //     }
  //   //} // Gyr_Bias = {20, 6, 7}. Error = {-2.20,-0.35,-0.14}.
  // Gyr_Bias = {106, 20, 8}. Error = {0,0,0}
  Gyr_Bias[0] = 104; Gyr_Bias[1] = 18; Gyr_Bias[2] = 23;   // OFFSET Giroscopio
  Acc_Gyro[0] = (Gyr_Bias[0] >> 8) & 0xFF;
  Acc_Gyro[1] = (Gyr_Bias[0])      & 0xFF;
  Acc_Gyro[2] = (Gyr_Bias[1] >> 8) & 0xFF;
  Acc_Gyro[3] = (Gyr_Bias[1])      & 0xFF;
  Acc_Gyro[4] = (Gyr_Bias[2] >> 8) & 0xFF;
  Acc_Gyro[5] = (Gyr_Bias[2])      & 0xFF;
  MARGwriteNBytes(MPU9250_ADDR, CS, GYRO_OFFSET, 6, Acc_Gyro);
  Acc_Bias[0] = 5406; Acc_Bias[1] = -5410; Acc_Bias[2] = 8058;  //OFFSET Acelerometro
  Acc_Gyro[0] = (Acc_Bias[0] >> 8) & 0xFF;
  Acc_Gyro[1] = (Acc_Bias[0])      & 0xFF;
  MARGwriteNBytes(MPU9250_ADDR, CS, ACC_OFFSET, 2, Acc_Gyro);
  Acc_Gyro[0] = (Acc_Bias[1] >> 8) & 0xFF;
  Acc_Gyro[1] = (Acc_Bias[1])      & 0xFF;
  MARGwriteNBytes(MPU9250_ADDR, CS, ACC_OFFSET + 3, 2, Acc_Gyro);
  Acc_Gyro[0] = (Acc_Bias[2] >> 8) & 0xFF;
  Acc_Gyro[1] = (Acc_Bias[2])      & 0xFF;
  MARGwriteNBytes(MPU9250_ADDR, CS, ACC_OFFSET + 6, 2, Acc_Gyro);


  //   /* Procedimiento para obtener el ajuste de SENSIBILIDAD Magnetometro */
  //   MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_1, 0x00);  //Set -> Power Down Mode
  //   delay(10);
  //   MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_1, 0x0F);  //Set -> Fuse ROM access Mode
  //   delay(10);
  //   MAGread(MAG_ADDRESS, CS, MAG_ASAX, 3, ASA);  //Get sensivility
  //   sAUX = "Sensitivity: "+String(ASA[0])+" "+String(ASA[1])+" "+String(ASA[2]);
  //   Serial.println(sAUX);
  //   MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_1, 0x00);  //Set -> Power Down Mode
  const uint8_t ASA[3] = {180, 181, 170}; //Magnetometer sensivility adjudment
  Mag_Adj[0] = ((ASA[0] - 128) / 256.0f + 1);
  Mag_Adj[1] = ((ASA[1] - 128) / 256.0f + 1);
  Mag_Adj[2] = ((ASA[2] - 128) / 256.0f + 1);

  {
    //   /* Procedimiento para realizar SELF-TEST Acel-Giro (Ver abajo) */
    //   MARGwriteByte(MPU9250_ADDR, CS, ACC_GYRO_CONFIG, 0x06);  //Gyro_DLPF => 92 Hz
    //   MARGwriteByte(MPU9250_ADDR, CS, ACC_CONFIG_1+1, 0x06);  //Acc_DLPF => 92 Hz
    //   MARGwriteByte(MPU9250_ADDR, CS, 28, ACC_FSCALE_2_G);
    //   MARGwriteByte(MPU9250_ADDR, CS, 27, GYRO_FSCALE_250_DPS);
    //   int32_t AX_OS=0, AY_OS=0, AZ_OS=0, GX_OS=0, GY_OS=0, GZ_OS=0;
    //   int32_t AX_ST_OS=0, AY_ST_OS=0, AZ_ST_OS=0, GX_ST_OS=0, GY_ST_OS=0, GZ_ST_OS=0;
    //   uint16_t i_ST = 0;
    //   bool ST_Finish = false, ST_active = false; //Norm_active = false;
    //   while(!ST_Finish){
    //     if(flag == 1){
    //       flag = 0;
    //       MARGread(MPU9250_ADDR, CS, ACC_GYRO_DATA, 14, Acc_Gyro);
    //              // Convertir registros acelerometro
    //       int16_t ax = (Acc_Gyro[0] << 8 | Acc_Gyro[1]);
    //       int16_t ay = (Acc_Gyro[2] << 8 | Acc_Gyro[3]);
    //       int16_t az = Acc_Gyro[4] << 8 | Acc_Gyro[5];
    //       int16_t gx = (Acc_Gyro[8] << 8 | Acc_Gyro[9]);
    //       int16_t gy = (Acc_Gyro[10] << 8 | Acc_Gyro[11]);
    //       int16_t gz = Acc_Gyro[12] << 8 | Acc_Gyro[13];
    //       i_ST++;
    //       if(i_ST <= 200){
    //         AX_OS += ax;     AY_OS += ay;     AZ_OS += az;
    //         GX_OS += gx;     GY_OS += gy;     GZ_OS += gz;
    //       }else if(i_ST <= 400){
    //         if(!ST_active){
    //           MARGwriteByte(MPU9250_ADDR, CS, 0x1B, 0b11100000); //Set Gyro SELF-TEST en X, Y y Z
    //           MARGwriteByte(MPU9250_ADDR, CS, 0x1C, 0b11100000); //Set Acc SELF-TEST en X, Y y Z
    //           ST_active = true;
    //           delay(30);
    //         }
    //         AX_ST_OS += ax;     AY_ST_OS += az;     AZ_ST_OS += ay;
    //         GX_ST_OS += gx;     GY_ST_OS += gz;     GZ_ST_OS += gy;
    //       }else{
    //         MARGwriteByte(MPU9250_ADDR, CS, 0x1B, 0b00000000); //Clear Gyro SELF-TEST en X, Y y Z
    //         MARGwriteByte(MPU9250_ADDR, CS, 0x1C, 0b00000000); //Clear Acc SELF-TEST en X, Y y Z
    //         delay(30);
    //         int32_t AXST = (AX_ST_OS - AX_OS) / 200;
    //         int32_t AYST = (AY_ST_OS - AY_OS) / 200;
    //         int32_t AZST = (AZ_ST_OS - AZ_OS) / 200;
    //         int32_t GXST = (GX_ST_OS - GX_OS) / 200;
    //         int32_t GYST = (GY_ST_OS - GY_OS) / 200;
    //         int32_t GZST = (GZ_ST_OS - GZ_OS) / 200;
    //         sAUX = String(AX_ST_OS)+" "+String(AY_ST_OS)+" "+String(AZ_ST_OS);
    //         Serial.println(sAUX);
    //         sAUX = String(AX_OS)+" "+String(AY_OS)+" "+String(AZ_OS);
    //         Serial.println(sAUX);
    //         sAUX = String(AXST)+" "+String(AYST)+" "+String(AZST);
    //         Serial.println(sAUX);
    //         sAUX = String(GX_ST_OS)+" "+String(GY_ST_OS)+" "+String(GZ_ST_OS);
    //         Serial.println(sAUX);
    //         sAUX = String(GX_OS)+" "+String(GY_OS)+" "+String(GZ_OS);
    //         Serial.println(sAUX);
    //         sAUX = String(GXST)+" "+String(GYST)+" "+String(GZST);
    //         Serial.println(sAUX);
    //         Serial.println();
    //         ST_Finish = true;
    //         uint8_t G_ST[3], A_ST[3];
    //         MARGread(MPU9250_ADDR, CS, ACC_SELF_TEST, 3, A_ST);
    //         MARGread(MPU9250_ADDR, CS, GYRO_SELF_TEST, 3, G_ST);
    //         int16_t AXST_OTP = 2620*pow(1.01, (A_ST[0]-1));
    //         int16_t AYST_OTP = 2620*pow(1.01, (A_ST[1]-1));
    //         int16_t AZST_OTP = 2620*pow(1.01, (A_ST[2]-1));
    //         int16_t GXST_OTP = 2620*pow(1.01, (G_ST[0]-1));
    //         int16_t GYST_OTP = 2620*pow(1.01, (G_ST[1]-1));
    //         int16_t GZST_OTP = 2620*pow(1.01, (G_ST[2]-1));
    //         sAUX = String(AXST_OTP)+" "+String(AYST_OTP)+" "+String(AZST_OTP);
    //         Serial.println(sAUX);
    //         sAUX = String(GXST_OTP)+" "+String(GYST_OTP)+" "+String(GZST_OTP);
    //         Serial.println(sAUX);
    //         sAUX = String(float(AXST)/AXST_OTP)+" "+String(float(AYST)/AYST_OTP)+" "+String(float(AZST)/AZST_OTP);
    //         Serial.println(sAUX);
    //         sAUX = String(float(GXST)/GXST_OTP)+" "+String(float(GYST)/GYST_OTP)+" "+String(float(GZST)/GZST_OTP);
    //         Serial.println(sAUX);
    //         delay(10000);
    //       }
    //     }
    //   }
    //   delay(10);


    //   /* Procedimiento para realizar SELF-TEST Magnetometro(Ver abajo) */
    //   MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_1, 0x10);  //Set -> Power Down Mode
    //   MAGwriteByte(MAG_ADDRESS, CS, MAG_ASTC, 0x40); //Magnetic Fiel for SelfTest starts
    //   delay(10);
    //   MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_1, 0x18);  //Set -> SELF-TEST Mode
    //   do{
    //      MAGread(MAG_ADDRESS, CS, MAG_STATUS_1, 1, &ST1);
    //   } while (!(ST1 & 0x01));
    //   MAGread(MAG_ADDRESS, CS, MAG_DATA, 7, Mag);
    //   MAGwriteByte(MAG_ADDRESS, CS, MAG_ASTC, 0x00);
    //   MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_1, 0x10);  //Set -> Power Down Mode
    //   delay(50);
    //   int16_t mx = (Mag[1] << 8 | Mag[0]);
    //   int16_t my = (Mag[3] << 8 | Mag[2]);
    //   int16_t mz = (Mag[5] << 8 | Mag[4]);
    //   sAUX = "NO SCALE SELF-TEST: "+String(mx)+" "+String(my)+" "+String(mz);
    //   Serial.println(sAUX);
    //   MX = Mag_Adj[0]*(float)mx;
    //   MY = Mag_Adj[1]*(float)my;
    //   MZ = Mag_Adj[2]*(float)mz;
    //   sAUX = "SELF-TEST: "+String(MX)+" "+String(MY)+" "+String(MZ);
    //   Serial.println(sAUX);
  }
  Acc_Bias[0] = -95; Acc_Bias[1] = -415; Acc_Bias[2] = -50; // MAG_FS = 0.6; -80,-400,-167
  Gyr_Bias[0] = 0; Gyr_Bias[1] = 0; Gyr_Bias[2] = 0; // ACC_FS = 0.59875e-3
  Mag_Bias[0] = -8; Mag_Bias[1] = -87; Mag_Bias[2] = -14; // GYR_FS = 7.6294e-3; -26,-83,10
  Acc_Adj[0] = 1; Acc_Adj[1] = 1; Acc_Adj[2] = 1;
  Gyr_Adj[0] = 1; Gyr_Adj[1] = 1; Gyr_Adj[2] = 1;
  Mag_Adj[0] *= 0.97; Mag_Adj[1] *= 0.94; Mag_Adj[2] *= 0.97;   // Mag_Adj : ASA-->{1.20313, 1.20703, 1.1641}


#ifdef SPI_PROTOCOL
 #ifdef MPU9250
//Indispensable para usar "SPI" sobre el Magnétometro MPU9250
  MARGwriteByte(MPU9250_ADDR, CS, MPU_INT_PIN_CFG, 0x12); 
  MARGwriteByte(MPU9250_ADDR, CS, MPU_USER_CTRL, 0x30);    // I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
  MARGwriteByte(MPU9250_ADDR, CS, MPU_I2C_MST_CTRL, 0x0D);    // I2C configuration multi-master  IIC 400KHz


  MARGwriteByte(MPU9250_ADDR, CS, MPU_I2C_SLV0_ADDR, MAG_ADDRESS);  // Set the I2C slave addres of AK8963 and set for write. 
  MARGwriteByte(MPU9250_ADDR, CS, MPU_I2C_SLV0_REG, MAG_CNTL_2); // I2C slave 0 register address from where to begin data transfer
  MARGwriteByte(MPU9250_ADDR, CS, MPU_I2C_SLV0_DO, 0x01);   // Reset AK8963
  MARGwriteByte(MPU9250_ADDR, CS, MPU_I2C_SLV0_CTRL, 0x81); // Enable I2C and set 1 byte

  MARGwriteByte(MPU9250_ADDR, CS, MPU_I2C_SLV0_REG, MAG_CNTL_1); // I2C slave 0 register address from where to begin data transfer
  MARGwriteByte(MPU9250_ADDR, CS, MPU_I2C_SLV0_DO, 0x12);   // Register value to 8Hz continuous measurement in 16bit
  MARGwriteByte(MPU9250_ADDR, CS, MPU_I2C_SLV0_CTRL, 0x81);  //Enable I2C and set 1 byte

  
//  while(false){
//    MAGread(MAG_ADDRESS, CS, MAG_DATA, 7, Mag);
//    for(int Reg=0; Reg<=6; Reg++){
//      //MARGread(MPU9250_ADDR, CS, Reg, 1, &Data);
//      sAUX = "Registro #"+String(Reg)+" =  "+String(Mag[Reg]);
//      Serial.println(sAUX);
//    }
//    Serial.println();
//    delay(1000);
//  }
 #endif
#endif

  //   uint8_t Data;
  //   for(uint8_t Reg=0; Reg<=19; Reg++){
  //     MAGread(MAG_ADDRESS, CS, Reg, 1, &Data);
  //     sAUX = "Registro #"+String(Reg)+" =  "+String(Data);
  //     Serial.println(sAUX);
  //   }
}

int16_t mx_ant, my_ant, mz_ant; uint8_t CSSV = 0; //Count Stagnant Sensor Values
uint32_t t_act, t_ant;
uint32_t t_p;
float t_delta;
void loop()
{
  t_ant = t_act;
  t_act = micros(); t_p = t_act;
  t_delta = float(t_act - t_ant) / 1e6;
  /* LECTURA DE VALORES SENSORES*/
  //SPI.setClockDivider(SPI_CLOCK_DIV2);
  MARGread(MPU9250_ADDR, CS, ACC_GYRO_DATA, 14, Acc_Gyro);
  //SPI.setClockDivider(SPI_CLOCK_DIV16);
  // Convertir registros acelerometro
  int16_t ax = (Acc_Gyro[0] << 8 | Acc_Gyro[1]);
  int16_t ay = (Acc_Gyro[2] << 8 | Acc_Gyro[3]);
  int16_t az = Acc_Gyro[4] << 8 | Acc_Gyro[5];
  AX = float(ax + Acc_Bias[0]) * Acc_Adj[0] * ACC_FS;
  AY = float(ay + Acc_Bias[1]) * Acc_Adj[1] * ACC_FS;
  AZ = float(az + Acc_Bias[2]) * Acc_Adj[2] * ACC_FS;
  Ac_mean.setNewValues(AX, AY, AZ);
  AX = Ac_mean.getMeanX(); AY = Ac_mean.getMeanY(); AZ = Ac_mean.getMeanZ();
  AR_ant = AR;   AR = sqrt(pow(AX, 2) + pow(AY, 2) + pow(AZ, 2));
  delta_AR = AR - AR_ant;
  // Convertir registros giroscopio
  int16_t gx = (Acc_Gyro[8] << 8 | Acc_Gyro[9]);
  int16_t gy = (Acc_Gyro[10] << 8 | Acc_Gyro[11]);
  int16_t gz = Acc_Gyro[12] << 8 | Acc_Gyro[13];
  GX = float(gx + Gyr_Bias[0]) * Gyr_Adj[0] * GYR_FS;
  GY = float(gy + Gyr_Bias[1]) * Gyr_Adj[1] * GYR_FS;
  GZ = float(gz + Gyr_Bias[2]) * Gyr_Adj[2] * GYR_FS;
  Gy_mean.setNewValues(GX, GY, GZ);
  GX = Gy_mean.getMeanX(); GY = Gy_mean.getMeanY(); GZ = Gy_mean.getMeanZ();
  GR_ant = GR;   GR = sqrt(pow(GX, 2) + pow(GY, 2) + pow(GZ, 2));
  delta_GR = GR - GR_ant;
  // ---  Lectura del magnetometro ---
  //sAUX = "Tiempo 1:  "+String(micros() - t_p);   Serial.print(sAUX);
  uint32_t t_MgStat = millis();
  //ST1 = 0;
  do {
    MAGread(MAG_ADDRESS, CS, MAG_STATUS_1, 1, &ST1);
    if(millis() - t_MgStat > 100){
      MARG_Reset();
      MARG_setUp();
      t_MgStat = millis();
      Serial.println(t_MgStat);
    }
    ST1 = 1;
  } while (!(ST1 & 0x01));
  //sAUX = "   Tiempo 2:  "+String(micros() - t_p);   Serial.print(sAUX);
  MAGread(MAG_ADDRESS, CS, MAG_DATA, 7, Mag);
  // Convertir registros magnetometro
  int16_t mx = (Mag[3] << 8 | Mag[2]);
  int16_t my = (Mag[1] << 8 | Mag[0]);
  int16_t mz = (Mag[5] << 8 | Mag[4]);
  MX = float(mx + Mag_Bias[1]) * Mag_Adj[1] * MAG_FS;
  MY = float(my + Mag_Bias[0]) * Mag_Adj[0] * MAG_FS;
  MZ = float(-mz + Mag_Bias[2]) * Mag_Adj[2] * MAG_FS;
  Ma_mean.setNewValues(MX, MY, MZ); // MX -= 15.0f
  MX = Ma_mean.getMeanX(); MY = Ma_mean.getMeanY(); MZ = Ma_mean.getMeanZ();
  // OFFSET: X = -38 a 41 uT   Y = -24 a 55 uT   Z = -44 a 39 uT
  MR_ant = MR;   MR = sqrt(pow(MX, 2) + pow(MY, 2) + pow(MZ, 2));
  delta_MR = MR - MR_ant;

  if ((mx_ant == mx) && (my_ant == my) && (mz_ant == mz)) {
    CSSV++;
    if (CSSV >= 10) {
      MARG_Reset();
      delay(1);
      MARG_setUp();
      CSSV = 0;
    }
  } else {
    CSSV = 0;
  }
  mx_ant = mx;  my_ant = my;  mz_ant = mz;

  //sAUX = "   Tiempo 3:  "+String(micros() - t_p);   Serial.println(sAUX);
  /* IMPRESIÓN DE VALORES POR PANTALLA  */
  // Acelerometro
  sAUX = "Ac=[" + String(AX) + " " + String(AY) + " " + String(AZ) + "] ";
  Serial.print(sAUX);
  //   sAUX = "  Magnitud:  "+String(AR)+"  Diff: "+String(100*delta_AR/t_delta);
  //   Serial.println(sAUX);
  // Giroscopio
  sAUX = "Gy=[" + String(GX) + " " + String(GY) + " " + String(GZ) + "] ";
  Serial.print(sAUX);
  //   sAUX = "  Magnitud:  "+String(GR)+"  Diff: "+String(100*delta_GR/t_delta);
  //   Serial.println(sAUX);
  // Magnetometro
  sAUX = "Mg=[" + String(MX) + " " + String(MY) + " " + String(MZ) + "]";
  Serial.print(sAUX);
  //   sAUX = "  Magnitud:  "+String(MR)+"  Diff: "+String(100*delta_MR/t_delta);
  //   Serial.println(sAUX);

  //sAUX = "Tiempo 4:  "+String(micros() - t_p);   Serial.println(sAUX);
  Serial.print(" "); Serial.print(t_act);
  Serial.println();
  
      MARG_Reset();
      MARG_setUp();
  delay(15);
  //   uint8_t Data;
  //  for(uint8_t Reg=0; Reg<=19; Reg++){
  //    MAGread(MAG_ADDRESS, CS, Reg, 1, &Data);
  //    sAUX = "Registro #"+String(Reg)+" =  "+String(Data);
  //    Serial.println(sAUX);
  //  }
}


ISR(TIMER2_OVF_vect) {
  flag = 1;
}
