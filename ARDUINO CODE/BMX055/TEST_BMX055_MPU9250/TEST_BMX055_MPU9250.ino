//GND - GND
//VCC - VCC
//SDA - Pin A4
//SCL - Pin A5
 
#include <Wire.h>

/*  DIRECCIONES I2C (A CONTINUCIÓN)  */
#define    MPU9250_ADDR            0x68
#define    MAG_ADDRESS             0x0C


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
#define   MAG_FSCALE_16_bit     0x10
#define   MAG_MODE_MASK       ob11110000  //Register 0xA [3:0 bit]
#define   MAG_MODE_2          0x06


/*  REGISTROS IMPORTANTES (A COTINUACIÓN)  */
#define GYRO_SELF_TEST   0X00  //Leer 3 registros: X Y Z
#define ACC_SELF_TEST    0X0D  //Leer 3 registros: X Y Z
#define GYRO_OFFSET      0x13  //Primer byte LOW, 2nd byte HIGH
#define ACC_OFFSET       0x77  //Primer byte LOW, 2nd byte HIGH.
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

/*     CONTANTES GLOBALES     */
#define  ACC_SENS   2048   //  32768 LSB / 16 G
#define  GYRO_SENS  33    //  32768 LSB / 1000 deg/seg

/* VARIABLES GLOBALES */
uint8_t Acc_Gyro[14];  //Accel-Gyro DATA
uint8_t Mag[7];        //Magnetometer DATA
float AX,AY,AZ,GX,GY,GZ,MX,MY,MZ; //9 GDL´s DATA
float AR,GR,MR,AR_ant,GR_ant,MR_ant,delta_AR,delta_GR,delta_MR;
int16_t Acc_Bias[3], Gyr_Bias[3], Mag_Bias[3]; //Sumar Offset
float Acc_Adj[3], Gyr_Adj[3], Mag_Adj[3]; //Multiplicar Offset
const uint8_t ASA[3] = {180, 181, 170}; //Magnetometer sensivility adjudment
uint8_t ST1;           //Magnetometer "Status" value

String sAUX;
float MAG_FS = 0.15; 
float ACC_FS = 0.59875e-3; //   (9.81 * 2) / 2^15
float GYR_FS = 7.6294e-3;  //    250 / 2^15

bool flag = false;

//Funcion auxiliar lectura
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
   Wire.beginTransmission(Address);
   Wire.write(Register);
   Wire.endTransmission();
 
   Wire.requestFrom(Address, Nbytes);
   uint8_t index = 0;
   while (Wire.available()){
      Data[index++] = Wire.read();
   }
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

class Index{
public:
  Index(uint8_t _NumOfIndices){
    NumOfIndices = _NumOfIndices;
    idx = 0;
  }
  uint8_t getIndex(){return idx;
  }
  operator++(){
    if(idx >= NumOfIndices - 1) idx = 0;
    else idx++;
  }
private:
  uint8_t idx;
  uint8_t NumOfIndices;
};
 
class Mean{
public:
  Mean(uint8_t _NumOfValues);
  ~Mean(){
    delete[] val[0];
    delete[] val[1];
    delete[] val[2];
    delete val;
    delete idx;
  }
  void setNewValues(float NewX, float NewY, float NewZ);
  float getMeanX(){return mean[0];}
  float getMeanY(){return mean[1];}
  float getMeanZ(){return mean[2];}
private:  
  Index *idx;
  float** val;
  float mean[3];
  uint8_t NumOfValues;
};
Mean::Mean(uint8_t _NumOfValues){
  NumOfValues = _NumOfValues;
  float NOV = float(NumOfValues);
  val = new float*[3];
  val[0] = new float[NumOfValues]; 
  val[1] = new float[NumOfValues];
  val[2] = new float[NumOfValues];
  idx = new Index(NumOfValues);
  mean[0] = 0.0; mean[1] = 0.0; mean[2] = 0.0;
  for(int i=0;i<NumOfValues;i++){
    val[0][i] = 0.0f;  
    val[1][i] = 0.0f;  
    val[2][i] = 0.0f;
  }
}
void Mean::setNewValues(float NewX, float NewY, float NewZ){
  uint8_t ix = idx->getIndex();
  float NOV = NumOfValues;
  mean[0] = (mean[0]*NOV - val[0][ix] + NewX)/NOV;
  mean[1] = (mean[1]*NOV - val[1][ix] + NewY)/NOV;
  mean[2] = (mean[2]*NOV - val[2][ix] + NewZ)/NOV;
  val[0][ix] = NewX;
  val[1][ix] = NewY;
  val[2][ix] = NewZ;
  (*idx)++;
}

uint8_t Nmuestras = 10;
Mean Ac_mean(Nmuestras), Gy_mean(Nmuestras), Ma_mean(Nmuestras);


/* ---------------------------- PROGRAM STARTS ------------------------------- */

#define   MAG_MODE_MASK       ob11110000  //Register 0xA [3:0 bit]
#define   MAG_MODE_2          0x06

#define MPU_FIFO_EN      0x23   //   
#define MPU_USR_CNTL     0x6A   // 


void setup()
{
   Wire.begin();
   Serial.begin(115200);
   I2CwriteByte(MPU9250_ADDR, ACC_GYRO_RESET, 0x80); //Reset Accel-Gyro Registers
   I2CwriteByte(MAG_ADDRESS, MAG_CNTL_1+1, 0x01);   //Soft Reset Magnetometer Registers
   // Configurar magnetometro
   I2CwriteByte(MPU9250_ADDR, 0x37, 0x02); //Registro INT_PIN_CFG
              /* INTERRUPCIÓN POR DESBORDAMIENTO DEL TIMER 2 */
   SREG = (SREG & 0b01111111); //Desabilitar interrupciones
   TIMSK2 = TIMSK2|0b00000001; //Habilita la interrupción por desbordamiento
   TCCR2B = 0b00000011; //Preescaler para frecuencia de 250kHz
   SREG = (SREG & 0b01111111) | 0b10000000; //Habilitar interrupciones //Desabilitar interrupciones
   delay(1000);
   I2CwriteByte(MPU9250_ADDR, ACC_GYRO_CONFIG, 0x06);  //Gyro_DLPF => 5 Hz
   I2CwriteByte(MPU9250_ADDR, ACC_CONFIG_1+1, 0x06);  //Acc_DLPF => 5 Hz


//   /* Procedimiento para obtener el offset Acel-Giro */
//   I2CwriteByte(MPU9250_ADDR, 28, ACC_FSCALE_16_G);
//   I2CwriteByte(MPU9250_ADDR, 27, GYRO_FSCALE_1000_DPS);
//   I2CwriteByte(MPU9250_ADDR, ACC_GYRO_CONFIG, 0x06);  //Gyro_DLPF => 92 Hz
//   I2CwriteByte(MPU9250_ADDR, ACC_CONFIG_1+1, 0x06);  //Acc_DLPF => 92 Hz
//   int32_t AX_OS=0, AY_OS=0, AZ_OS=0, GX_OS=0, GY_OS=0, GZ_OS=0;
//   bool Offs_Finish = false, Off_Error = false;
//   uint16_t i_Offs = 0;
//   uint8_t ErrorAX, ErrorAY, ErrorGX, ErrorGY, ErrorGZ;
//   //while(!Off_Error){
//     Offs_Finish = false;
//     while(!Offs_Finish){
//       if(flag){
//         flag = 0;
//         I2Cread(MPU9250_ADDR, ACC_GYRO_DATA, 14, Acc_Gyro);
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
//           //I2CwriteNBytes(MPU9250_ADDR, GYRO_OFFSET, Acc_Gyro, 6);
//           I2Cread(MPU9250_ADDR, ACC_OFFSET, 2, Acc_Gyro);
//           Acc_Bias[0] = (Acc_Gyro[0] << 8 | Acc_Gyro[1]);
//           I2Cread(MPU9250_ADDR, ACC_OFFSET+3, 2, Acc_Gyro);
//           Acc_Bias[1] = (Acc_Gyro[0] << 8 | Acc_Gyro[1]);
//           I2Cread(MPU9250_ADDR, ACC_OFFSET+6, 2, Acc_Gyro);
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
         // Gyr_Bias = {106, 20, 8}. Error = {0,0,0}. 
   Gyr_Bias[0] = 104; Gyr_Bias[1] = 18; Gyr_Bias[2] = 23;   // OFFSET Giroscopio
   Acc_Gyro[0] = (Gyr_Bias[0] >> 8) & 0xFF;
   Acc_Gyro[1] = (Gyr_Bias[0])      & 0xFF;
   Acc_Gyro[2] = (Gyr_Bias[1] >> 8) & 0xFF;
   Acc_Gyro[3] = (Gyr_Bias[1])      & 0xFF;
   Acc_Gyro[4] = (Gyr_Bias[2] >> 8) & 0xFF;
   Acc_Gyro[5] = (Gyr_Bias[2])      & 0xFF;
   I2CwriteNBytes(MPU9250_ADDR, GYRO_OFFSET, Acc_Gyro, 6);
   Acc_Bias[0] = 5406; Acc_Bias[1] = -5410; Acc_Bias[2] = 8058;  //OFFSET Acelerometro
   Acc_Gyro[0] = (Acc_Bias[0] >> 8) & 0xFF;
   Acc_Gyro[1] = (Acc_Bias[0])      & 0xFF;
   I2CwriteNBytes(MPU9250_ADDR, ACC_OFFSET, Acc_Gyro, 2);
   Acc_Gyro[0] = (Acc_Bias[1] >> 8) & 0xFF;
   Acc_Gyro[1] = (Acc_Bias[1])      & 0xFF;
   I2CwriteNBytes(MPU9250_ADDR, ACC_OFFSET+3, Acc_Gyro, 2);
   Acc_Gyro[0] = (Acc_Bias[2] >> 8) & 0xFF;
   Acc_Gyro[1] = (Acc_Bias[2])      & 0xFF;
   I2CwriteNBytes(MPU9250_ADDR, ACC_OFFSET+6, Acc_Gyro, 2);
   

//   /* Procedimiento para obtener el ajuste de SENSIBILIDAD Magnetometro */
//   I2CwriteByte(MAG_ADDRESS, MAG_CNTL_1, 0x00);  //Set -> Power Down Mode
//   delay(10);
//   I2CwriteByte(MAG_ADDRESS, MAG_CNTL_1, 0x0F);  //Set -> Fuse ROM access Mode
//   delay(10);
//   I2Cread(MAG_ADDRESS, MAG_ASAX, 3, ASA);  //Get sensivility
//   sAUX = "Sensitivity: "+String(ASA[0])+" "+String(ASA[1])+" "+String(ASA[2]);
//   Serial.println(sAUX);
//   I2CwriteByte(MAG_ADDRESS, MAG_CNTL_1, 0x00);  //Set -> Power Down Mode 
   Mag_Adj[0] = ((ASA[0]-128)/256.0f + 1);
   Mag_Adj[1] = ((ASA[1]-128)/256.0f + 1);
   Mag_Adj[2] = ((ASA[2]-128)/256.0f + 1);


//   /* Procedimiento para realizar SELF-TEST Acel-Giro (Ver abajo) */
//   I2CwriteByte(MPU9250_ADDR, ACC_GYRO_CONFIG, 0x06);  //Gyro_DLPF => 92 Hz
//   I2CwriteByte(MPU9250_ADDR, ACC_CONFIG_1+1, 0x06);  //Acc_DLPF => 92 Hz
//   I2CwriteByte(MPU9250_ADDR, 28, ACC_FSCALE_2_G);
//   I2CwriteByte(MPU9250_ADDR, 27, GYRO_FSCALE_250_DPS);
//   int32_t AX_OS=0, AY_OS=0, AZ_OS=0, GX_OS=0, GY_OS=0, GZ_OS=0;
//   int32_t AX_ST_OS=0, AY_ST_OS=0, AZ_ST_OS=0, GX_ST_OS=0, GY_ST_OS=0, GZ_ST_OS=0;
//   uint16_t i_ST = 0;
//   bool ST_Finish = false, ST_active = false; //Norm_active = false;
//   while(!ST_Finish){
//     if(flag == 1){
//       flag = 0;
//       I2Cread(MPU9250_ADDR, ACC_GYRO_DATA, 14, Acc_Gyro);
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
//           I2CwriteByte(MPU9250_ADDR, 0x1B, 0b11100000); //Set Gyro SELF-TEST en X, Y y Z
//           I2CwriteByte(MPU9250_ADDR, 0x1C, 0b11100000); //Set Acc SELF-TEST en X, Y y Z
//           ST_active = true;
//           delay(30);
//         }
//         AX_ST_OS += ax;     AY_ST_OS += az;     AZ_ST_OS += ay;
//         GX_ST_OS += gx;     GY_ST_OS += gz;     GZ_ST_OS += gy; 
//       }else{
//         I2CwriteByte(MPU9250_ADDR, 0x1B, 0b00000000); //Clear Gyro SELF-TEST en X, Y y Z
//         I2CwriteByte(MPU9250_ADDR, 0x1C, 0b00000000); //Clear Acc SELF-TEST en X, Y y Z 
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
//         I2Cread(MPU9250_ADDR, ACC_SELF_TEST, 3, A_ST);
//         I2Cread(MPU9250_ADDR, GYRO_SELF_TEST, 3, G_ST);
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
//   I2CwriteByte(MAG_ADDRESS, MAG_CNTL_1, 0x10);  //Set -> Power Down Mode
//   I2CwriteByte(MAG_ADDRESS, MAG_ASTC, 0x40); //Magnetic Fiel for SelfTest starts
//   delay(10);
//   I2CwriteByte(MAG_ADDRESS, MAG_CNTL_1, 0x18);  //Set -> SELF-TEST Mode
//   do{
//      I2Cread(MAG_ADDRESS, MAG_STATUS_1, 1, &ST1);
//   } while (!(ST1 & 0x01));
//   I2Cread(MAG_ADDRESS, MAG_DATA, 7, Mag);
//   I2CwriteByte(MAG_ADDRESS, MAG_ASTC, 0x00);
//   I2CwriteByte(MAG_ADDRESS, MAG_CNTL_1, 0x10);  //Set -> Power Down Mode
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

   Acc_Bias[0] = -95; Acc_Bias[1] = -415; Acc_Bias[2] = -50; // MAG_FS = 0.6; -80,-400,-167
   Gyr_Bias[0] = 0; Gyr_Bias[1] = 0; Gyr_Bias[2] = 0; // ACC_FS = 0.59875e-3
   Mag_Bias[0] = -8; Mag_Bias[1] = -87; Mag_Bias[2] = -14; // GYR_FS = 7.6294e-3; -26,-83,10
   Acc_Adj[0] = 1; Acc_Adj[1] = 1; Acc_Adj[2] = 1; 
   Gyr_Adj[0] = 1; Gyr_Adj[1] = 1; Gyr_Adj[2] = 1; 
   Mag_Adj[0] *= 0.97; Mag_Adj[1] *= 0.94; Mag_Adj[2] *= 0.97; //0.9886,.9965,.9156


   delay(500);
   // Configurar acelerometro
   I2CwriteByte(MPU9250_ADDR, 28, ACC_FSCALE_2_G);
   I2CwriteByte(MPU9250_ADDR, ACC_CONFIG_1+1, 0x06);  //Acc_DLPF => 5 Hz
   // Configurar giroscopio
   I2CwriteByte(MPU9250_ADDR, 27, GYRO_FSCALE_250_DPS);
   I2CwriteByte(MPU9250_ADDR, ACC_GYRO_CONFIG, 0x02);  //Gyro_DLPF => 5 Hz
   // Configurar magnetometro
   I2CwriteByte(MPU9250_ADDR, 0x37, 0x02); //Registro INT_PIN_CFG
   
   I2CwriteByte(MAG_ADDRESS, MAG_CNTL_1, MAG_MODE_2 | MAG_FSCALE_16_bit);
   delay(1000);
//   uint8_t Data;
//   for(uint8_t Reg=0; Reg<=19; Reg++){
//     I2Cread(MAG_ADDRESS, Reg, 1, &Data);
//     sAUX = "Registro #"+String(Reg)+" =  "+String(Data);
//     Serial.println(sAUX);
//   }
  Serial.println("SetUp Ready");
}
 
uint32_t t_act, t_ant;
uint32_t t_p;
float t_delta;
void loop()
{
   t_ant = t_act;
   t_act = micros(); t_p = t_act;
   t_delta = float(t_act-t_ant)/1e6;
   /* LECTURA DE VALORES SENSORES*/
   I2Cread(MPU9250_ADDR, ACC_GYRO_DATA, 14, Acc_Gyro);
          // Convertir registros acelerometro
   int16_t ax = (Acc_Gyro[0] << 8 | Acc_Gyro[1]);
   int16_t ay = (Acc_Gyro[2] << 8 | Acc_Gyro[3]);
   int16_t az = Acc_Gyro[4] << 8 | Acc_Gyro[5];
   AX = float(ax+Acc_Bias[0])*Acc_Adj[0]*ACC_FS;
   AY = float(ay+Acc_Bias[1])*Acc_Adj[1]*ACC_FS;
   AZ = float(az+Acc_Bias[2])*Acc_Adj[2]*ACC_FS;
   Ac_mean.setNewValues(AX,AY,AZ);
   AX = Ac_mean.getMeanX(); AY = Ac_mean.getMeanY(); AZ = Ac_mean.getMeanZ();
   AR_ant = AR;   AR = sqrt(pow(AX,2)+pow(AY,2)+pow(AZ,2)); 
   delta_AR = AR - AR_ant;
           // Convertir registros giroscopio
   int16_t gx = (Acc_Gyro[8] << 8 | Acc_Gyro[9]);
   int16_t gy = (Acc_Gyro[10] << 8 | Acc_Gyro[11]);
   int16_t gz = Acc_Gyro[12] << 8 | Acc_Gyro[13];
   GX = float(gx+Gyr_Bias[0])*Gyr_Adj[0]*GYR_FS;
   GY = float(gy+Gyr_Bias[1])*Gyr_Adj[1]*GYR_FS;
   GZ = float(gz+Gyr_Bias[2])*Gyr_Adj[2]*GYR_FS;
   Gy_mean.setNewValues(GX,GY,GZ);
   GX = Gy_mean.getMeanX(); GY = Gy_mean.getMeanY(); GZ = Gy_mean.getMeanZ();
   GR_ant = GR;   GR = sqrt(pow(GX,2)+pow(GY,2)+pow(GZ,2));
   delta_GR = GR - GR_ant;
           // ---  Lectura del magnetometro --- 
   //sAUX = "Tiempo 1:  "+String(micros() - t_p);   Serial.print(sAUX);
   do{ I2Cread(MAG_ADDRESS, MAG_STATUS_1, 1, &ST1);
   } while (!(ST1 & 0x01));
   //sAUX = "   Tiempo 2:  "+String(micros() - t_p);   Serial.print(sAUX);
   I2Cread(MAG_ADDRESS, MAG_DATA, 7, Mag);
           // Convertir registros magnetometro
   int16_t mx = (Mag[3] << 8 | Mag[2]);
   int16_t my = (Mag[1] << 8 | Mag[0]);
   int16_t mz = (Mag[5] << 8 | Mag[4]);
   MX = float(mx+Mag_Bias[1])*Mag_Adj[1]*MAG_FS;
   MY = float(my+Mag_Bias[0])*Mag_Adj[0]*MAG_FS;
   MZ = float(-mz+Mag_Bias[2])*Mag_Adj[2]*MAG_FS;
   Ma_mean.setNewValues(MX,MY,MZ);// MX -= 15.0f
   MX = Ma_mean.getMeanX(); MY = Ma_mean.getMeanY(); MZ = Ma_mean.getMeanZ();
   // OFFSET: X = -38 a 41 uT   Y = -24 a 55 uT   Z = -44 a 39 uT
   MR_ant = MR;   MR = sqrt(pow(MX,2)+pow(MY,2)+pow(MZ,2));
   delta_MR = MR - MR_ant;
   
   //sAUX = "   Tiempo 3:  "+String(micros() - t_p);   Serial.println(sAUX);
   /* IMPRESIÓN DE VALORES POR PANTALLA  */
                // Acelerometro
   sAUX = String(AX)+" "+String(AY)+" "+String(AZ)+" ";
   Serial.print(sAUX);
//   sAUX = "  Magnitud:  "+String(AR)+"  Diff: "+String(100*delta_AR/t_delta);
//   Serial.println(sAUX);
                 // Giroscopio
//   sAUX = "GYR: "+String(GX)+" "+String(GY)+" "+String(GZ);
//   Serial.print(sAUX);
//   sAUX = "  Magnitud:  "+String(GR)+"  Diff: "+String(100*delta_GR/t_delta);
//   Serial.println(sAUX);
                  // Magnetometro
   sAUX = String(MX)+" "+String(MY)+" "+String(MZ);
   Serial.print(sAUX);
//   sAUX = "  Magnitud:  "+String(MR)+"  Diff: "+String(100*delta_MR/t_delta);
//   Serial.println(sAUX); 
     
   //sAUX = "Tiempo 4:  "+String(micros() - t_p);   Serial.println(sAUX);
   Serial.println();
   
   delay(150);  
//   uint8_t Data;
//  for(uint8_t Reg=0; Reg<=19; Reg++){
//    I2Cread(MAG_ADDRESS, Reg, 1, &Data);
//    sAUX = "Registro #"+String(Reg)+" =  "+String(Data);
//    Serial.println(sAUX);
//  }  
}


ISR(TIMER2_OVF_vect){   
  flag = 1;
}
