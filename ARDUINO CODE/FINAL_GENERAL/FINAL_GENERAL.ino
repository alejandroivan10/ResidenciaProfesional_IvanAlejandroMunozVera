//I2C Wiring
//GND - GND
//VCC - VCC
//SDA - Pin A4
//SCL - Pin A5

//SPI Wiring
//SDA(MOSI) - Pin D11
//AD0(MISO) - Pin D12
//SCL(SCK)  - Pin D13
//NCS(CS)   - Pin D7 (arbitrario)

/*
   INDISPENSABLES:
    + Definir un protocolo de comunicación: #define "SPI_PROTOCOL" o "I2C_PROTOCOL"
*/
#define I2C_PROTOCOL
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

uint8_t Nmuestras = 3;
Mean Ac_mean(Nmuestras), Gy_mean(Nmuestras), Ma_mean(Nmuestras);


void MARG_Reset() {
  MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_2, 0x01);   //Soft Reset Magnetometer Registers
  MARGwriteByte(MPU9250_ADDR, CS, ACC_GYRO_RESET, 0x80); //Reset Accel-Gyro Registers
}
void MARG_setUp() {
#ifdef SPI_PROTOCOL
  //Indispensable para usar "SPI" sobre el Magnétometro MPU9250
  MARGwriteByte(MPU9250_ADDR, CS, MPU_INT_PIN_CFG, 0x12);
  MARGwriteByte(MPU9250_ADDR, CS, MPU_USER_CTRL, 0x30);    // I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
  MARGwriteByte(MPU9250_ADDR, CS, MPU_I2C_MST_CTRL, 0x0D);    // I2C configuration multi-master  IIC 400KHz
#endif
  // Configurar acelerometro
  MARGwriteByte(MPU9250_ADDR, CS, 28, ACC_FSCALE_2_G);
  MARGwriteByte(MPU9250_ADDR, CS, ACC_CONFIG_1 + 1, 0x06); //Addr=0x1C  Acc_DLPF => 5 Hz
  // Configurar giroscopio
  MARGwriteByte(MPU9250_ADDR, CS, 27, GYRO_FSCALE_250_DPS);
  MARGwriteByte(MPU9250_ADDR, CS, ACC_GYRO_CONFIG, 0x02);  //Addr=0x1A Gyro_DLPF => 5 Hz
  // Configurar magnetometro
  MARGwriteByte(MPU9250_ADDR, CS, 0x37, 0x02); //Registro INT_PIN_CFG
  MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_1, MAG_MODE_2 | MAG_FSCALE_16_bit);
  //Serial.println(".........SetUP Executed.........");
}




void setup(){
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

  MARG_Reset();
  MARG_setUp();

  /* INTERRUPCIÓN POR DESBORDAMIENTO DEL TIMER 2 */
  SREG = (SREG & 0b01111111); //Desabilitar interrupciones
  TIMSK2 = TIMSK2 | 0b00000001; //Habilita la interrupción por desbordamiento
  TCCR2B = 0b00000011; //Preescaler para frecuencia de 250kHz
  SREG = (SREG & 0b01111111) | 0b10000000; //Habilitar interrupciones //Desabilitar interrupciones
  delay(10);

  // Gyr_Bias = {20, 6, 7}. Error = {-2.20,-0.35,-0.14}.
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

  const uint8_t ASA[3] = {180, 181, 170}; //Magnetometer sensivility adjudment
  Mag_Adj[0] = ((ASA[0] - 128) / 256.0f + 1);
  Mag_Adj[1] = ((ASA[1] - 128) / 256.0f + 1);
  Mag_Adj[2] = ((ASA[2] - 128) / 256.0f + 1);

  Acc_Bias[0] = -95; Acc_Bias[1] = -415; Acc_Bias[2] = -50; // MAG_FS = 0.6; -80,-400,-167
  Gyr_Bias[0] = 0; Gyr_Bias[1] = 0; Gyr_Bias[2] = 0; // ACC_FS = 0.59875e-3
  Mag_Bias[0] = -8; Mag_Bias[1] = -87; Mag_Bias[2] = -14; // GYR_FS = 7.6294e-3; -26,-83,10
  Acc_Adj[0] = 1; Acc_Adj[1] = 1; Acc_Adj[2] = 1;
  Gyr_Adj[0] = 1; Gyr_Adj[1] = 1; Gyr_Adj[2] = 1;
  Mag_Adj[0] *= 0.97; Mag_Adj[1] *= 0.94; Mag_Adj[2] *= 0.97;   // Mag_Adj : ASA-->{1.20313, 1.20703, 1.1641}
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
    if (millis() - t_MgStat > 100) {
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
}


ISR(TIMER2_OVF_vect) {
  flag = 1;
}
