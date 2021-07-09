//GND - GND
//VCC - VCC
//SDA - Pin A4
//SCL - Pin A5

//SDA(MOSI) - Pin D11
//AD0(MISO) - Pin D12
//SCL(SCK)  - Pin D13
//NCS(CS)   - Pin D7 (arbitrario)

/*
   INDISPENSABLES:
    + Definir un protocolo de comunicación: #define "SPI_PROTOCOL" o "I2C_PROTOCOL"
    + Definir un sensor: #define "MPU9250" o "BMX055"
*/
#define I2C_PROTOCOL
#define MPU9250
uint8_t CS = 7;  // Chip Select


#if defined(I2C_PROTOCOL)
#undef SPI_PROTOCOL
#include <Wire.h>
#define MARGread(Addr, CS, Reg, nBytes, Data) I2Cread(Addr, Reg, nBytes, Data)
#define MARGwriteByte(Addr, CS, Reg, Data) I2CwriteByte(Addr, Reg, Data)
#define MARGwriteNBytes(Addr, CS, Reg, nBytes, Data) I2CwriteNBytes(Addr, Reg, nBytes, Data)
#define MAGread(Addr, CS, Reg, nBytes, Data) I2Cread(Addr, Reg, nBytes, Data)
#define MAGwriteByte(Addr, CS, Reg, Data) I2CwriteByte(Addr, Reg, Data)
#define MAGwriteNBytes(Addr, CS, Reg, nBytes, Data) I2CwriteNBytes(Addr, Reg, nBytes, Data)

#elif defined(SPI_PROTOCOL)
#undef I2C_PROTOCOL
#include <SPI.h>
#define MARGread(Addr, CS, Reg, nBytes, Data) SPIread(CS, Reg, nBytes, Data)
#define MARGwriteByte(Addr, CS, Reg, Data) SPIwriteByte(CS, Reg, Data)
#define MARGwriteNBytes(Addr, CS, Reg, nBytes, Data) SPIwriteNBytes(CS, Reg, nBytes, Data)
#define MAGread(Addr, CS, Reg, nBytes, Data) SPIreadMg(Addr, CS, Reg, nBytes, Data)
#define MAGwriteByte(Addr, CS, Reg, Data) SPIwriteByteMg(Addr, CS, Reg, Data)
#define MAGwriteNBytes(Addr, CS, Reg, nBytes, Data) SPIwriteNBytesMg(Addr, CS, Reg, nBytes, Data)

#else
#error "No Protocol has been selected"
#endif

#include "Complementos.h"
#include "RegistersAndMask.h"



/* VARIABLES GLOBALES */
uint8_t Acc_Gyro[14];                      //Accel-Gyro DATA
uint8_t Mag[7];                            //Magnetometer DATA
float AX, AY, AZ, GX, GY, GZ, MX, MY, MZ;  //9 GDL´s DATA
float AR, GR, MR, AR_ant, GR_ant, MR_ant, delta_AR, delta_GR, delta_MR;
int32_t AX_OS = 0, AY_OS = 0, AZ_OS = 0, GX_OS = 0, GY_OS = 0, GZ_OS = 0;
int16_t Acc_Bias[3], Gyr_Bias[3], Mag_Bias[3];  //Sumar Offset
float Acc_Adj[3], Gyr_Adj[3], Mag_Adj[3];       //Multiplicar Offset
uint8_t ST1;                                    //Magnetometer "Status" value

String sAUX;
float MAG_FS = 0.15;
float ACC_FS = 0.59875e-3;  //   (9.81 * 2) / 2^15
float GYR_FS = 7.6294e-3;   //    250 / 2^15

bool flag = false;

uint8_t Nmuestras = 3;
Mean Ac_mean(Nmuestras), Gy_mean(Nmuestras), Ma_mean(Nmuestras);

void setup() {
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

  
  MARGwriteByte(MPU9250_ADDR, CS, 0x37, 0x02);  //Registro INT_PIN_CFG
  MARG_Reset();
  delay(10);
  // Configurar magnetometro

  /* INTERRUPCIÓN POR DESBORDAMIENTO DEL TIMER 2 */
  SREG = (SREG & 0b01111111);               //Desabilitar interrupciones
  TIMSK2 = TIMSK2 | 0b00000001;             //Habilita la interrupción por desbordamiento
  TCCR2B = 0b00000011;                      //Preescaler para frecuencia de 250kHz
  SREG = (SREG & 0b01111111) | 0b10000000;  //Habilitar interrupciones //Desabilitar interrupciones
  delay(10);



  /**************  PROCEDIMIENTO PARA OBTENER EL OFFSET ACEL-GYRO  ***************/
  MARGwriteByte(MPU9250_ADDR, CS, 28, ACC_FSCALE_16_G);
  MARGwriteByte(MPU9250_ADDR, CS, 27, GYRO_FSCALE_1000_DPS);
  MARGwriteByte(MPU9250_ADDR, CS, ACC_GYRO_CONFIG, 0x06);   //Gyro_DLPF => 92 Hz
  MARGwriteByte(MPU9250_ADDR, CS, ACC_CONFIG_1 + 1, 0x06);  //Acc_DLPF => 92 Hz
  bool Offs_Finish = false, Off_Error = false;
  uint16_t i_Offs = 0;
  uint8_t ErrorAX, ErrorAY, ErrorGX, ErrorGY, ErrorGZ;
  //while(!Off_Error){
  Offs_Finish = false;
  while (!Offs_Finish) {
    if (flag) {
      flag = 0;
      MARGread(MPU9250_ADDR, CS, ACC_GYRO_DATA, 14, Acc_Gyro);
      // Convertir registros acelerometro
      int16_t ax = (Acc_Gyro[0] << 8 | Acc_Gyro[1]);
      int16_t ay = (Acc_Gyro[2] << 8 | Acc_Gyro[3]);
      int16_t az = (Acc_Gyro[4] << 8 | Acc_Gyro[5]);
      int16_t gx = (Acc_Gyro[8] << 8 | Acc_Gyro[9]);
      int16_t gy = (Acc_Gyro[10] << 8 | Acc_Gyro[11]);
      int16_t gz = (Acc_Gyro[12] << 8 | Acc_Gyro[13]);
      i_Offs++;
      if (i_Offs <= 50) {
        AX_OS += ax;
        AY_OS += ay;
        AZ_OS += az;
        GX_OS += gx;
        GY_OS += gy;
        GZ_OS += gz;
      } else {
        int16_t AXOFF = AX_OS / 200;
        int16_t AYOFF = AY_OS / 200;
        int16_t AZOFF = AZ_OS / 200;
        int16_t GXOFF = GX_OS / 200;
        int16_t GYOFF = GY_OS / 200;
        int16_t GZOFF = GZ_OS / 200;
        sAUX = "ACC: " + String(AXOFF) + " " + String(AYOFF) + " " + String(AZOFF);
        Serial.println(sAUX);
        sAUX = "GYR: " + String(GXOFF) + " " + String(GYOFF) + " " + String(GZOFF);
        Serial.println(sAUX);
        sAUX = "ACC_GRAV: " + String(float(AXOFF) / ACC_SENS) + " " + String(float(AYOFF) / ACC_SENS) + " " + String(float(AZOFF) / ACC_SENS);
        Serial.println(sAUX);
        sAUX = "GYR_DPS: " + String(float(GXOFF) / GYRO_SENS) + " " + String(float(GYOFF) / GYRO_SENS) + " " + String(float(GZOFF) / GYRO_SENS);
        Serial.println(sAUX);
        if (AZOFF > 0) {
          AZOFF -= ACC_SENS;
        } else {
          AZOFF += ACC_SENS;
        }
        Acc_Gyro[0] = (-GXOFF >> 8) & 0xFF;
        Acc_Gyro[1] = (-GXOFF) & 0xFF;
        Acc_Gyro[2] = (-GYOFF >> 8) & 0xFF;
        Acc_Gyro[3] = (-GYOFF) & 0xFF;
        Acc_Gyro[4] = (-GZOFF >> 8) & 0xFF;
        Acc_Gyro[5] = (-GZOFF) & 0xFF;
        //MARGwriteNBytes(MPU9250_ADDR, CS, GYRO_OFFSET, 6, Acc_Gyro);
        MARGread(MPU9250_ADDR, CS, ACC_OFFSET, 2, Acc_Gyro);
        Acc_Bias[0] = (Acc_Gyro[0] << 8 | Acc_Gyro[1]);
        MARGread(MPU9250_ADDR, CS, ACC_OFFSET + 3, 2, Acc_Gyro);
        Acc_Bias[1] = (Acc_Gyro[0] << 8 | Acc_Gyro[1]);
        MARGread(MPU9250_ADDR, CS, ACC_OFFSET + 6, 2, Acc_Gyro);
        Acc_Bias[2] = (Acc_Gyro[0] << 8 | Acc_Gyro[1]);
        sAUX = "AC_BIAS: " + String(Acc_Bias[0]) + " " + String(Acc_Bias[1]) + " " + String(Acc_Bias[2]);
        Serial.println(sAUX);
        Acc_Bias[0] -= AXOFF;
        Acc_Bias[1] -= AYOFF;
        Acc_Bias[2] -= AZOFF;
        Offs_Finish = true;
        delay(1000);
      }
    }
  }

  //  Gyr_Bias = {20, 6, 7}. Error = {-2.20,-0.35,-0.14}.
  //  Gyr_Bias = {106, 20, 8}. Error = {0,0,0}
  //  Gyr_Bias[0] = 104; Gyr_Bias[1] = 18; Gyr_Bias[2] = 23;   // OFFSET Giroscopio
  Acc_Gyro[0] = (Gyr_Bias[0] >> 8) & 0xFF;
  Acc_Gyro[1] = (Gyr_Bias[0]) & 0xFF;
  Acc_Gyro[2] = (Gyr_Bias[1] >> 8) & 0xFF;
  Acc_Gyro[3] = (Gyr_Bias[1]) & 0xFF;
  Acc_Gyro[4] = (Gyr_Bias[2] >> 8) & 0xFF;
  Acc_Gyro[5] = (Gyr_Bias[2]) & 0xFF;
  MARGwriteNBytes(MPU9250_ADDR, CS, GYRO_OFFSET, 6, Acc_Gyro);

  //Acc_Bias[0] = 5406; Acc_Bias[1] = -5410; Acc_Bias[2] = 8058;  //OFFSET Acelerometro
  Acc_Gyro[0] = (Acc_Bias[0] >> 8) & 0xFF;
  Acc_Gyro[1] = (Acc_Bias[0]) & 0xFF;
  MARGwriteNBytes(MPU9250_ADDR, CS, ACC_OFFSET, 2, Acc_Gyro);
  Acc_Gyro[0] = (Acc_Bias[1] >> 8) & 0xFF;
  Acc_Gyro[1] = (Acc_Bias[1]) & 0xFF;
  MARGwriteNBytes(MPU9250_ADDR, CS, ACC_OFFSET + 3, 2, Acc_Gyro);
  Acc_Gyro[0] = (Acc_Bias[2] >> 8) & 0xFF;
  Acc_Gyro[1] = (Acc_Bias[2]) & 0xFF;
  MARGwriteNBytes(MPU9250_ADDR, CS, ACC_OFFSET + 6, 2, Acc_Gyro);






  /**********  PROCEDIMIENTO PARA REALIZAR SELF-TEST ACEL-GYRO  *************/
  MARGwriteByte(MPU9250_ADDR, CS, ACC_GYRO_CONFIG, 0x06);   //Gyro_DLPF => 92 Hz
  MARGwriteByte(MPU9250_ADDR, CS, ACC_CONFIG_1 + 1, 0x06);  //Acc_DLPF => 92 Hz
  MARGwriteByte(MPU9250_ADDR, CS, 28, ACC_FSCALE_2_G);
  MARGwriteByte(MPU9250_ADDR, CS, 27, GYRO_FSCALE_250_DPS);
  AX_OS = 0, AY_OS = 0, AZ_OS = 0, GX_OS = 0, GY_OS = 0, GZ_OS = 0;
  int32_t AX_ST_OS = 0, AY_ST_OS = 0, AZ_ST_OS = 0, GX_ST_OS = 0, GY_ST_OS = 0, GZ_ST_OS = 0;
  uint16_t i_ST = 0;
  bool ST_Finish = false, ST_active = false;  //Norm_active = false;
  Serial.println("Acc-Gyr Self-Test starts...");
  while (!ST_Finish) {
    Serial.println(flag);
    if (flag == 1) {
      flag = 0;
      MARGread(MPU9250_ADDR, CS, ACC_GYRO_DATA, 14, Acc_Gyro);
      // Convertir registros acelerometro
      int16_t ax = (Acc_Gyro[0] << 8 | Acc_Gyro[1]);
      int16_t ay = (Acc_Gyro[2] << 8 | Acc_Gyro[3]);
      int16_t az = Acc_Gyro[4] << 8 | Acc_Gyro[5];
      int16_t gx = (Acc_Gyro[8] << 8 | Acc_Gyro[9]);
      int16_t gy = (Acc_Gyro[10] << 8 | Acc_Gyro[11]);
      int16_t gz = Acc_Gyro[12] << 8 | Acc_Gyro[13];
      i_ST++;
      Serial.println(String(i_ST) + " " + String(flag));
      if (i_ST <= 200) {
        AX_OS += ax;
        AY_OS += ay;
        AZ_OS += az;
        GX_OS += gx;
        GY_OS += gy;
        GZ_OS += gz;
      } else if (i_ST <= 400) {
        if (!ST_active) {
          MARGwriteByte(MPU9250_ADDR, CS, 0x1B, 0b11100000);  //Set Gyro SELF-TEST en X, Y y Z
          MARGwriteByte(MPU9250_ADDR, CS, 0x1C, 0b11100000);  //Set Acc SELF-TEST en X, Y y Z
          ST_active = true;
          delay(30);
        }
        AX_ST_OS += ax;
        AY_ST_OS += az;
        AZ_ST_OS += ay;
        GX_ST_OS += gx;
        GY_ST_OS += gz;
        GZ_ST_OS += gy;
      } else {
        MARGwriteByte(MPU9250_ADDR, CS, 0x1B, 0b00000000);  //Clear Gyro SELF-TEST en X, Y y Z
        MARGwriteByte(MPU9250_ADDR, CS, 0x1C, 0b00000000);  //Clear Acc SELF-TEST en X, Y y Z
        delay(30);
        int32_t AXST = (AX_ST_OS - AX_OS) / 200;
        int32_t AYST = (AY_ST_OS - AY_OS) / 200;
        int32_t AZST = (AZ_ST_OS - AZ_OS) / 200;
        int32_t GXST = (GX_ST_OS - GX_OS) / 200;
        int32_t GYST = (GY_ST_OS - GY_OS) / 200;
        int32_t GZST = (GZ_ST_OS - GZ_OS) / 200;
        sAUX = String(AX_ST_OS) + " " + String(AY_ST_OS) + " " + String(AZ_ST_OS);
        Serial.println(sAUX);
        sAUX = String(AX_OS) + " " + String(AY_OS) + " " + String(AZ_OS);
        Serial.println(sAUX);
        sAUX = String(AXST) + " " + String(AYST) + " " + String(AZST);
        Serial.println(sAUX);
        float Ac_G = 0.00006103f;
        sAUX = "AxST-2G Scale:  " + String(AXST * Ac_G) + " " + String(AYST * Ac_G) + " " + String(AZST * Ac_G);
        Serial.println(sAUX);

        sAUX = String(GX_ST_OS) + " " + String(GY_ST_OS) + " " + String(GZ_ST_OS);
        Serial.println(sAUX);
        sAUX = String(GX_OS) + " " + String(GY_OS) + " " + String(GZ_OS);
        Serial.println(sAUX);
        sAUX = String(GXST) + " " + String(GYST) + " " + String(GZST);
        Serial.println(sAUX);
        sAUX = "GxST-250°Scale:  " + String(GXST * GYR_FS) + " " + String(GYST * GYR_FS) + " " + String(GZST * GYR_FS);
        Serial.println(sAUX);
        Serial.println();
        ST_Finish = true;
        uint8_t G_ST[3], A_ST[3];
        MARGread(MPU9250_ADDR, CS, ACC_SELF_TEST, 3, A_ST);
        MARGread(MPU9250_ADDR, CS, GYRO_SELF_TEST, 3, G_ST);
        int16_t AXST_OTP = 2620 * pow(1.01, (A_ST[0] - 1));
        int16_t AYST_OTP = 2620 * pow(1.01, (A_ST[1] - 1));
        int16_t AZST_OTP = 2620 * pow(1.01, (A_ST[2] - 1));
        int16_t GXST_OTP = 2620 * pow(1.01, (G_ST[0] - 1));
        int16_t GYST_OTP = 2620 * pow(1.01, (G_ST[1] - 1));
        int16_t GZST_OTP = 2620 * pow(1.01, (G_ST[2] - 1));
        sAUX = String(AXST_OTP) + " " + String(AYST_OTP) + " " + String(AZST_OTP);
        Serial.println(sAUX);
        sAUX = String(GXST_OTP) + " " + String(GYST_OTP) + " " + String(GZST_OTP);
        Serial.println(sAUX);
        sAUX = "AxST/AxST_OTP:  " + String(float(AXST) / AXST_OTP) + " " + String(float(AYST) / AYST_OTP) + " " + String(float(AZST) / AZST_OTP);
        Serial.println(sAUX);
        sAUX = "GxST/GxST_OTP:  " + String(float(GXST) / GXST_OTP) + " " + String(float(GYST) / GYST_OTP) + " " + String(float(GZST) / GZST_OTP);
        Serial.println(sAUX);
        Serial.println("Acc-Gyr Self-Test finished");
        Serial.println();
        delay(1000);
      }
    }
  }
  delay(10);







  /******  PROCEDIMIENTO PARA OBTENER EL AJUSTE DE SENSIBILIDAD MAGNETÓMETRO  *******/
  uint8_t ASA[3];
  MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_1, 0x00);  //Set -> Power Down Mode
  delay(10);
  MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_1, 0x0F);  //Set -> Fuse ROM access Mode
  delay(10);
  MAGread(MAG_ADDRESS, CS, MAG_ASAX, 3, ASA);  //Get sensivility
  sAUX = "Sensitivity: " + String(ASA[0]) + " " + String(ASA[1]) + " " + String(ASA[2]);
  Serial.println(sAUX);
  MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_1, 0x00);  //Set -> Power Down Mode
  //const uint8_t ASA[3] = {180, 181, 170}; //Magnetometer sensivility adjudment
  Mag_Adj[0] = ((ASA[0] - 128) / 256.0f + 1);
  Mag_Adj[1] = ((ASA[1] - 128) / 256.0f + 1);
  Mag_Adj[2] = ((ASA[2] - 128) / 256.0f + 1);




  /**********  PROCEDIMIENTO PARA REALIZAR SELF-TEST MAGNETÓMETRO  ***********/
  Serial.println("Mag Self-Test Starts (16 bit mode):");
  MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_1, 0x00);  //Set -> Power Down Mode
  MAGwriteByte(MAG_ADDRESS, CS, MAG_ASTC, 0x40);    //Magnetic Fiel for SelfTest starts
  delay(10);
  MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_1, 0x18);  //Set -> SELF-TEST Mode && 16 bit output
  do {
    MAGread(MAG_ADDRESS, CS, MAG_STATUS_1, 1, &ST1);
  } while (!(ST1 & 0x01));
  MAGread(MAG_ADDRESS, CS, MAG_DATA, 7, Mag);
  MAGwriteByte(MAG_ADDRESS, CS, MAG_ASTC, 0x00);
  MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_1, 0x00);  //Set -> Power Down Mode
  delay(50);
  int16_t mx = (Mag[1] << 8 | Mag[0]);
  int16_t my = (Mag[3] << 8 | Mag[2]);
  int16_t mz = (Mag[5] << 8 | Mag[4]);
  sAUX = "NO SCALE SELF-TEST (-2^15 a 2^15): " + String(mx) + " " + String(my) + " " + String(mz);
  Serial.println(sAUX);
  MX = Mag_Adj[0] * (float)mx;
  MY = Mag_Adj[1] * (float)my;
  MZ = Mag_Adj[2] * (float)mz;
  sAUX = "SELF-TEST (): " + String(MX) + " " + String(MY) + " " + String(MZ);
  Serial.println(sAUX);
  Serial.println("Mag Self-Test Finished...");
  Serial.println();



  /***********  CALIBRACIÓN MANUAL  ************/
  //  Acc_Bias[0] = -95; Acc_Bias[1] = -415; Acc_Bias[2] = -50; // MAG_FS = 0.6; -80,-400,-167
  //  Gyr_Bias[0] = 0; Gyr_Bias[1] = 0; Gyr_Bias[2] = 0; // ACC_FS = 0.59875e-3
  //  Mag_Bias[0] = -8; Mag_Bias[1] = -87; Mag_Bias[2] = -14; // GYR_FS = 7.6294e-3; -26,-83,10
  //  Acc_Adj[0] = 1; Acc_Adj[1] = 1; Acc_Adj[2] = 1;
  //  Gyr_Adj[0] = 1; Gyr_Adj[1] = 1; Gyr_Adj[2] = 1;
  //  Mag_Adj[0] *= 0.97; Mag_Adj[1] *= 0.94; Mag_Adj[2] *= 0.97;   // Mag_Adj : ASA-->{1.20313, 1.20703, 1.1641}
}


void loop() {
}


ISR(TIMER2_OVF_vect) {
  flag = 1;
}







void MARG_Reset() {
  MARGwriteByte(MPU9250_ADDR, CS, ACC_GYRO_RESET, 0x80);  //Reset Accel-Gyro Registers
  MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_1 + 1, 0x01);    //Soft Reset Magnetometer Registers
}

void MARG_setUp() {
  // Configurar acelerometro
  MARGwriteByte(MPU9250_ADDR, CS, 28, ACC_FSCALE_2_G);
  MARGwriteByte(MPU9250_ADDR, CS, ACC_CONFIG_1 + 1, 0x06);  //Addr=0x1C  Acc_DLPF => 5 Hz
  // Configurar giroscopio
  MARGwriteByte(MPU9250_ADDR, CS, 27, GYRO_FSCALE_250_DPS);
  MARGwriteByte(MPU9250_ADDR, CS, ACC_GYRO_CONFIG, 0x02);  //Addr=0x1A Gyro_DLPF => 5 Hz
  // Configurar magnetometro
  MARGwriteByte(MPU9250_ADDR, CS, 0x37, 0x02);  //Registro INT_PIN_CFG
  MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_1, MAG_MODE_2 | MAG_FSCALE_16_bit);

  //Serial.println(".........SetUP Executed.........");
}
