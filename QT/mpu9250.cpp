//#include <QString>
//#include "matlab.h"
//#include "Complementos.h"
#include <QDebug>
#include <cmath>
#include <QUdpSocket>
#include <typeinfo>
#include <initializer_list>
#include "stdio.h"
#include "wiringPi.h"
#include "wiringPiSPI.h"

#include "mpu9250.h"
#include "SPI.h"
#include "arduino.h"
#include "RegistersAndMask.h"
//#include "SetUp.h"
#include "mocapsuit.h"



/* VARIABLES GLOBALES */
uint8_t Acc_Gyro[14];  //Accel-Gyro DATA
uint8_t Mag[7];        //Magnetometer DATA

QString sAUX;
const float MAG_FS = 0.15;
const float ACC_FS = 0.59875e-3; //   (9.81 * 2) / 2^15
const float GYR_FS = 7.6294e-3;  //    250 / 2^15
uint Mpu9250::countOfSens = 0;

Mpu9250::Mpu9250(uint _CS, MocapSuit& _Suit, floatList _Ac_Adj,
                 floatList _Gy_Adj, floatList _Mg_Adj, floatList _UE_init) :
    CS(_CS), Ac_Adj(_Ac_Adj), Gy_Adj(_Gy_Adj), Mg_Adj(_Mg_Adj), UE_init(_UE_init)
{
    Ac_Adj_Sens = Ac_Adj(0)/Ac_Adj(0,2); //Acel_Calib
    Ac_Adj_Offs = Ac_Adj(3,5);
    Mg_Adj_Sens = Mg_Adj(0)/Mg_Adj(0,2); //Mag_Calib
    Mg_Adj_Offs = Mg_Adj(3,5);
    ++countOfSens;
    id = countOfSens;

    _Suit.addSensor(this);
}
Mpu9250::~Mpu9250(){
    pinMode(CS, INPUT);
}




void Mpu9250::setup(void){
    pinMode(CS, OUTPUT);
    digitalWrite(CS, HIGH);
    delayMicroseconds(10);
    delay(10);


#ifdef SPI_PROTOCOLt
    uint8_t Dato[2] = {29,44};
    uint8_t Data[2];

    while(true){
        SPIwriteNBytes(CS, 99, 2, Dato);  //Gyro_DLPF => 5 Hz
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

#ifdef SPI_PROTOCOL_ARDUINO
 #ifdef MPU9250
    //Indispensable para usar "SPI" sobre el Magnétometro MPU9250
    MARGwriteByte(MPU9250_ADDR, CS, MPU_INT_PIN_CFG, 0x12);
    MARGwriteByte(MPU9250_ADDR, CS, MPU_USER_CTRL, 0x30);    // I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
    MARGwriteByte(MPU9250_ADDR, CS, MPU_I2C_MST_CTRL, 0x0D);    // I2C configuration multi-master  IIC 400KHz

    MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_2, 0x01); /*
     MARGwriteByte(MPU9250_ADDR, CS, MPU_I2C_SLV0_ADDR, MAG_ADDRESS);  // Set the I2C slave addres of AK8963 and set for write.
     MARGwriteByte(MPU9250_ADDR, CS, MPU_I2C_SLV0_REG, MAG_CNTL_2); // I2C slave 0 register address from where to begin data transfer
     MARGwriteByte(MPU9250_ADDR, CS, MPU_I2C_SLV0_DO, 0x01);   // Reset AK8963
     MARGwriteByte(MPU9250_ADDR, CS, MPU_I2C_SLV0_CTRL, 0x81); // Enable I2C and set 1 byte     */

    MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_1, 0x12); /*
     MARGwriteByte(MPU9250_ADDR, CS, MPU_I2C_SLV0_REG, MAG_CNTL_1); // I2C slave 0 register address from where to begin data transfer
     MARGwriteByte(MPU9250_ADDR, CS, MPU_I2C_SLV0_DO, 0x12);   // Register value to 8Hz continuous measurement in 16bit
     MARGwriteByte(MPU9250_ADDR, CS, MPU_I2C_SLV0_CTRL, 0x81);  //Enable I2C and set 1 byte   */


//    while(true){
//        MAGread(MAG_ADDRESS, CS, MAG_DATA, 7, Mag);
//        /*for(int Reg=0; Reg<=6; Reg++){
//          //MARGread(MPU9250_ADDR, CS, Reg, 1, &Data);
//          sAUX = "Registro #"+String(Reg)+" =  "+String(Mag[Reg]);
//          Serial.println(sAUX);
//        }*/
//        uint8_t Data[2];
//        for(uint8_t Reg=1; Reg<=125; Reg++){
//            Data[0]=66;
//            SPIread(CS, Reg, 1, Data);
//            sAUX = "Registro #"+String(Reg)+" =  "+String(Data[0]);
//            Serial.println(sAUX);
//        }
//        Serial.println();
//        delay(10000);
//    }
 #endif
#endif


    AcGy_Reset();
    AcGy_setUp();
    Mag_Reset();
    Mag_setUp();
    delay(1);
    // Configurar magnetometro
    //MARGwriteByte(MPU9250_ADDR, CS, 0x37, 0x02); //Registro INT_PIN_CFG




    {

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
}
    Gyr_Bias[0] = 103; Gyr_Bias[1] = 18; Gyr_Bias[2] = 23;   // OFFSET Giroscopio
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


    //   uint8_t Data;
    //   for(uint8_t Reg=0; Reg<=19; Reg++){
    //     MAGread(MAG_ADDRESS, CS, Reg, 1, &Data);
    //     sAUX = "Registro #"+String(Reg)+" =  "+String(Data);
    //     Serial.println(sAUX);
    //   }
}






void Mpu9250::loop(loopFlag loopType){
    t_ant = t_act;
    t_act = micros();
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
    uint32_t t_MgStat = millis();
//        do {
//            MAGread(MAG_ADDRESS, CS, MAG_STATUS_1, 1, &ST1);
//            if(millis() - t_MgStat > 1000){
//              Mag_Reset();
//              Mag_setUp();
//              t_MgStat = millis();
//              ST1 = 1;
//            }
//        } while (!(ST1 & 0x01));
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


    if ((ax_ant == ax) || (ay_ant == ay) || (az_ant == az)) {
        ++aCSSV;
        qDebug() << "   aCSSV: " << aCSSV;
        if (aCSSV >= 4) {
            AcGy_Reset();
            delay(1);
            int set = AcGy_setUp();
            if(set < 0){
                fprintf(stderr, "   .........Error %d: AcGy_Setup()........ \n", set);
            }
            aCSSV = 0;
        }
    } else {
        aCSSV = 0;
    }

    if ((mx_ant == mx) || (my_ant == my) || (mz_ant == mz)) {
        ++mCSSV;
        qDebug() << "   mCSSV: " << mCSSV;
        if (mCSSV == 4) {
            Mag_Reset();
            delay(1);
            Mag_setUp();
        }else if(mCSSV >= 7){
            Mag_Reset();
            AcGy_Reset();
            delay(1);
            int set = AcGy_setUp();
            if(set < 0){
                fprintf(stderr, "   .........Error %d: AcGy_Setup()........ \n", set);
            }
            set = Mag_setUp();
            if(set < 0){
                fprintf(stderr, "   .........Error %d: Mag_Setup()........ \n", set);
            }
            delay(1);
            mCSSV = 0;
        }
    } else {
        mCSSV = 0;
    }
    ax_ant = ax;  ay_ant = ay;  az_ant = az;
    mx_ant = mx;  my_ant = my;  mz_ant = mz;

    /* IMPRESIÓN DE VALORES POR PANTALLA  */
    /*
    // Acelerometro
    QString sAUX;
    sAUX = "Ac=[" + String(AX) + " " + String(AY) + " " + String(AZ) + "] ";
    QByteArray dataSnd = sAUX.toUtf8();
    Serial.print(sAUX);
    //   sAUX = "  Magnitud:  "+String(AR)+"  Diff: "+String(100*delta_AR/t_delta);
    //   Serial.println(sAUX);
    // Giroscopio

    sAUX = "Gy=[" + String(GX) + " " + String(GY) + " " + String(GZ) + "] ";
    dataSnd.append(sAUX.toUtf8());
    Serial.print(sAUX);
    //   sAUX = "  Magnitud:  "+String(GR)+"  Diff: "+String(100*delta_GR/t_delta);
    //   Serial.println(sAUX);
    // Magnetometro
    sAUX = "Mg=[" + String(MX) + " " + String(MY) + " " + String(MZ) + "] ";
    dataSnd.append(sAUX.toUtf8());
    Serial.print(sAUX);
    //   sAUX = "  Magnitud:  "+String(MR)+"  Diff: "+String(100*delta_MR/t_delta);
    //   Serial.println(sAUX);

    //sAUX = "Tiempo 4:  "+String(micros() - t_p);   Serial.println(sAUX);
    Serial.print(t_act);
    Serial.println();
    */

    //   uint8_t Data;
    //  for(uint8_t Reg=0; Reg<=19; Reg++){
    //    MAGread(MAG_ADDRESS, CS, Reg, 1, &Data);
    //    sAUX = "Registro #"+String(Reg)+" =  "+String(Data);
    //    Serial.println(sAUX);
    //  }













    /*            COMIENZA MATLAB              */

    // Ejecutar bucle infinito
    /// COMIENZA WHILE(TRUE)
    time_sensor = static_cast<float>(micros()) / 1000000.0f;
    t1 = t2;   t2 = time_sensor;   deltat = t2 - t1;
    Vector Ac = {AX, AY, AZ};  Vector Gy = {GX, GY, GZ};
    Vector Mg = {MX, MY, MZ};
    Vector Ac_Raw = Ac,    Mg_Raw = Mg,     Gy_Raw = Gy;
    Ac = (Ac - Ac_Adj_Offs) * Ac_Adj_Sens;
    Mg = (Mg - Mg_Adj_Offs) * Mg_Adj_Sens;
    Gy =  Gy - Gy_Adj;

//    bool isMagAnormal = (20 > norm(Mg) || norm(Mg) > 150) && (delta_MR > 15);
//    bool isAcAnormal = (8.6 > norm(Mg) || norm(Mg) > 11) && (delta_MR > 2.5);
//    if(isMagAnormal || isAcAnormal){
//        fprintf(stderr, "   Reset-SetUp por comportamiento anormal:   \n");
//        Mag_Reset();
//        AcGy_Reset();
//        delay(1);
//        if(AcGy_setUp() < 0){
//            fprintf(stderr, "   .........Error: AcGy_Setup()........ \n");
//        }
//        if(Mag_setUp() < 0){
//            fprintf(stderr, "   .........Error: Mag_Setup()........ \n");
//        }
//        delay(5);

//    }


    // OBTENCIÓN DE LA ORIENTACIÓN POR MADGWICK
    Vector halfSEq(4), twoSEq(4);
    halfSEq(0) = 0.5f * SEq(0);
    halfSEq(1) = 0.5f * SEq(1);
    halfSEq(2) = 0.5f * SEq(2);
    halfSEq(3) = 0.5f * SEq(3);
    twoSEq(0) = 2.0f * SEq(0);
    twoSEq(1) = 2.0f * SEq(1);
    twoSEq(2) = 2.0f * SEq(2);
    twoSEq(3) = 2.0f * SEq(3);
//    float twob_x = 2.0f * b_x;
//    float twob_z = 2.0f * b_z;
    Vector twob_xSEq(4), twob_zSEq(4);
    twob_xSEq(0) = 2.0f * b_x * SEq(0);
    twob_xSEq(1) = 2.0f * b_x * SEq(1);
    twob_xSEq(2) = 2.0f * b_x * SEq(2);
    twob_xSEq(3) = 2.0f * b_x * SEq(3);
    twob_zSEq(0) = 2.0f * b_z * SEq(0);
    twob_zSEq(1) = 2.0f * b_z * SEq(1);
    twob_zSEq(2) = 2.0f * b_z * SEq(2);
    twob_zSEq(3) = 2.0f * b_z * SEq(3);
    //SEq_1SEq_2;
    float SEq_1SEq_3 = SEq(0) * SEq(2);
    //SEq_1SEq_4;
    //SEq_2SEq_3;
    float SEq_2SEq_4 = SEq(1) * SEq(3);
    //SEq_3SEq_4;
    Vector two_m(3);
    two_m(0) = 2.0f * Mg(0);
    two_m(1) = 2.0f * Mg(1);
    two_m(2) = 2.0f * Mg(2);

    // magnalise the accelerometer measurement
    float magn;
            magn = norm(Ac);
    Vector Ac_one = Ac/magn;
    // magnalise the magnetometer measurement
            magn = norm(Mg);
    Vector Mg_one = Mg/magn;
    Gy = Gy*(PI/180.0f); //Conversion entre (Deg/seg) a (Rad/seg)
//    fprintf(stderr, "i=%d Ac=[%.3f %.3f %.3f] Gy=[%.3f %.3f %.3f]  Mg=[%.3f %.3f %.3f]\n",
//        i+1,AX,AY,AZ,GX,GY,GZ,MX,MY,MZ);
    fprintf(stderr, "i=%d Ac=[%.3f %.3f %.3f] Gy=[%.3f %.3f %.3f]  Mg=[%.3f %.3f %.3f]\n",
        i+1,Ac(0),Ac(1),Ac(2),Gy(0),Gy(1),Gy(2),Mg(0),Mg(1),Mg(2));
//    fprintf(stderr, "i=%d Ac=[%.3f %.3f %.3f] Gy=[%.3f %.3f %.3f]  Mg=[%.3f %.3f %.3f]\n",
//        i+1,Ac_one(0),Ac_one(1),Ac_one(2),Gy(0),Gy(1),Gy(2),Mg_one(0),Mg_one(1),Mg_one(2));
    //fprintf(stderr, "Tiempo = %f \n", t2);

    // compute the objective function and Jacobian
    Vector f1(3), f2(3);
    f1 = SErotM * g_E - Ac_one;
    f2 = SErotM * b_E - Mg_one;
    f = append(f1, f2);
    //f(1:3,1) = [0 0 0];
    // compute the gradient (matrix multiplication)
    Matriz J(6,4);
    J(2,0) = 0;  J(2,3) = 0;
    J(1,3) = twoSEq(2);  J(0,0) = -J(1,3);// J_11 negated in matrix multiplication
    J(0,1) = twoSEq(3);  J(1,2) =  J(0,1);
    J(1,1) = twoSEq(0);  J(0,2) = -J(1,1); // J_12 negated in matrix multiplication
    J(0,3) = twoSEq(1);  J(1,0) =  J(0,3);
    J(2,1) = -2.0f * twoSEq(1); // negated in matrix multiplication
    J(2,2) = -2.0f * twoSEq(2); // negated in matrix multiplication
    J(3,0) = -twob_zSEq(2); // negated in matrix multiplication
    J(3,1) =  twob_zSEq(3);
    J(3,2) = -2.0f * twob_xSEq(2) - twob_zSEq(0); // negated in matrix multiplication
    J(3,3) = -2.0f * twob_xSEq(3) + twob_zSEq(1); // negated in matrix multiplication
    J(4,0) = -twob_xSEq(3) + twob_zSEq(1); // negated in matrix multiplication
    J(4,1) =  twob_xSEq(2) + twob_zSEq(0);
    J(4,2) =  twob_xSEq(1) + twob_zSEq(3);
    J(4,3) = -twob_xSEq(0) + twob_zSEq(2); // negated in matrix multiplication
    J(5,0) = twob_xSEq(2);
    J(5,1) = twob_xSEq(3) - 2.0f * twob_zSEq(1);
    J(5,2) = twob_xSEq(0) - 2.0f * twob_zSEq(2);
    J(5,3) = twob_xSEq(1);
    SEqHatDot = f*J;
    // magnalise the gradient to estimate direction of the gyroscope error
    SEqHatDot = SEqHatDot / norm(SEqHatDot);

    // compute angular estimated direction of the gyroscope error
    Vector err = 2.0f * quatmultiply(quatconj(SEq), SEqHatDot);
    Vector w_err(3);   w_err(0) = err(1);   w_err(1) = err(2);   w_err(2) = err(3);
    // compute and remove the gyroscope baises
    w_bx = w_bx + w_err(0) * deltat * zeta;
    w_by = w_by + w_err(1) * deltat * zeta;
    w_bz = w_bz + w_err(2) * deltat * zeta;
    w_bx = 0.0f;  w_by = 0.0f;  w_bz = 0.0f;
    Gy(0) = Gy(0) - w_bx;
    Gy(1) = Gy(1) - w_by;
    Gy(2) = Gy(2) - w_bz;
    // compute the quaternion rate measured by gyroscopes
    Vector Gy0(4);  Gy0(0) = 0.0f;  Gy0(1) = Gy(0);  Gy0(2) = Gy(1);  Gy0(3) = Gy(2);
    Vector SEqDot_omega = 0.5f * quatmultiply(SEq, Gy0);
    //SEqDot_omega = [0 0 0 0];

    // compute then integrate the estimated quaternion rate
    SEq(0) = SEq(0) + (SEqDot_omega(0) - (beta * SEqHatDot(0))) * deltat;
    SEq(1) = SEq(1) + (SEqDot_omega(1) - (beta * SEqHatDot(1))) * deltat;
    SEq(2) = SEq(2) + (SEqDot_omega(2) - (beta * SEqHatDot(2))) * deltat;
    SEq(3) = SEq(3) + (SEqDot_omega(3) - (beta * SEqHatDot(3))) * deltat;
    // magnalise quaternion
    magn = norm(SEq);
    SEq(0) = SEq(0) / magn;
    SEq(1) = SEq(1) / magn;
    SEq(2) = SEq(2) / magn;
    SEq(3) = SEq(3) / magn;
    // compute flux in the earth frame
    float SEq_1SEq_2 = SEq(0) * SEq(1); // recompute axulirary variables
          SEq_1SEq_3 = SEq(0) * SEq(2);
    float SEq_1SEq_4 = SEq(0) * SEq(3);
    float SEq_3SEq_4 = SEq(2) * SEq(3);
    float SEq_2SEq_3 = SEq(1) * SEq(2);
          SEq_2SEq_4 = SEq(1) * SEq(3);

    SErotM = Quat2RotMat(SEq);
    Vector h_S = (SErotM * two_m);
    SErotM = SErotM.t();

    // magnalise the flux vector to have only components in the x and z
//     b_x = sqrt((h_S(1) * h_S(1)) + (h_S(2) * h_S(2)));
//     b_z = h_S(3);
    //fprintf('----bx=%.3f bz=%.3f   w_bx=%.3f w_by=%.3f w_bz=%.3f \n', b_x,b_z,w_bx,w_by,w_bz);



    //% CALIBRACIÓN ELÍPTICA AUTOMÁTICA
    float btw_angle = dot(Ac,Ac_capt(i_capt))/(norm(Ac)*norm(Ac_capt(i_capt))); // Angle Between measurements
        bool if_btw_angAc = btw_angle < 0.9659;  // btw_angle < que 15°
    btw_angle = dot(Mg,Mg_capt(i_capt))/(norm(Mg)*norm(Mg_capt(i_capt))); // Angle Between measurements
        bool if_btw_angMg = btw_angle < 0.9659;  // btw_angle < que 15°
    d_Ac(i % nCalib) = norm(Ac-Ac_ant) /norm(Ac_ant);
    d_Mg(i % nCalib) = norm(Mg-Mg_ant) /norm(Mg_ant);
        bool No_Move_Ac_Mg = (sum(d_Ac)/nCalib < 0.05)  &&  (sum(d_Mg)/nCalib < 0.05);
    //fprintf(stderr, 'Mg_ant= [%.3f  %.3f  %.3f]   Mg= [%.3f  %.3f  %.3f]\n', Mg, Mg_ant);
    //fprintf('Ac_ant= [%.3f  %.3f  %.3f]   Ac= [%.3f  %.3f  %.3f]\n', Ac, Ac_ant);
    //fprintf('btw_angle= %.3f  d_Ac= %.3f   d_Mg= %.3f \n', btw_angle, sum(d_Ac)/nCalib, sum(d_Mg)/nCalib);
    //NoMove_Gy = sum(Gy.^2) < 4; // Gy < 4 [°/seg]

    // Proceso de validación y posible captura de datos
    Ac_ant = Ac;
    Gy_ant = Gy;
    Mg_ant = Mg;
    //bol = if_btw_ang & No_Move_Ac_Mg
    if  (No_Move_Ac_Mg){     //if true, entonces captura datos
        if (if_btw_angMg && if_btw_angAc && (aCSSV == 0 && mCSSV == 0)){
            i_capt = j % nCapt;
            Ac_capt(i_capt) = Ac;
            Gy_capt(i_capt) = Gy;
            Mg_capt(i_capt) = Mg;
            if (i_capt == nCapt){
                Vector Ac_Adj_new = RegresiLinealMultip(Ac_capt);
                Vector Mg_Adj_new = RegresiLinealMultip(Mg_capt);
                Vector Gy_Adj_new = sum(Gy_capt,1); Gy_Adj_new = Gy_Adj_new / nCapt;

                if (Ac_Adj_new(0) != -1.0f){ // Si es diferente de -1
                    Vector Ac_TamH_new = Ac_Adj_new;
                    // Proceso de aprobación/Rechazo de los nuevos parámetros de ajuste
                    Vector Ac_AdjError = 1.0f - sum( pow( (Ac_capt-Ac_Adj_new(3,5)) / Ac_Adj_new(0,2) ), 2 ); // Ac_AdjError = 1 - ((x-h)/m)^2 - ((y-h)/n)^2 - ((z-h)/o)^2
                    Vector Ac_AdjRel = sqrt(1.0f/(1.0f-Ac_AdjError));  // Rel = (1/(1-Error))^0.5
                        Vector if_Ac_inv = (0 < Ac_AdjRel) && (Ac_AdjRel < 1);
                    Ac_AdjRel = (!if_Ac_inv)*Ac_AdjRel + if_Ac_inv/Ac_AdjRel;
                    float Ac_Adj_TotalErr = sum(Ac_AdjRel) / Ac_capt.getDimV();
                    fprintf(stderr, "AcTol_Err = %.3f  \n", Ac_Adj_TotalErr);
                    if (Ac_Adj_TotalErr < Ac_Tol){
                        Ac_Tol = 0.7*(Ac_Tol) + 0.3*(1.08);  //  = Ac_Tol - 0.4*(Ac_Tol-1.08)
                        // If TRUE, enonces son parámetros validos, y los actualizamos
                        Vector Ac_Adj_new_Sens =  Ac_Adj_new(0)/Ac_Adj_new(0,2);
                        Vector Ac_Adj_new_Offs = Ac_Adj(3,5) + Ac_Adj_new(3,5)/Ac_Adj(0,2);
                        Ac_Adj_new_Sens = Ac_Adj(0,2)*Ac_Adj_new(0,2);
//                         i_adj = mod(k,10)+1;
//                         Ac_Adj_new(4:6) = Ac_Adj(4:6) + Ac_Adj_new(4:6).*Ac_Adj(1:3);
//                         Ac_Adj_new(1:3) = Ac_Adj(1:3).*Ac_Adj_new(1:3);
//                         Mg_Adj_new(4:6) = Mg_Adj(4:6) + Mg_Adj_new(4:6).*Mg_Adj(1:3);
//                         Mg_Adj_new(1:3) = Mg_Adj(1:3).*Mg_Adj_new(1:3);
//                         Ac_Adj_Store(i_adj,:) = Ac_Adj; //Ac_Adjust_Storage
//                         Mg_Adj_Store(i_adj,:) = Mg_Adj;

                        // Se mezclan los viejos y los nuevos parametros con 60% y 40% c/u
                        Ac_Adj_new = append(Ac_Adj_new_Sens, Ac_Adj_new_Offs);
                        Ac_Adj = 0.6*Ac_Adj + 0.4*Ac_Adj_new;
                        Vector VPrint = Ac_TamH_new(0)/Ac_Adj(0,2);
                        fprintf(stderr, "Ac_Adj =");  fprintf(stderr, "   %.3f   %.3f   %.3f", VPrint[0], VPrint[1], VPrint[2]);
                            fprintf(stderr, "   %.3f   %.3f   %.3f \n", Ac_Adj[3], Ac_Adj[4], Ac_Adj[5]);
                        pause(5);
                    }else{
                        fprintf(stderr, "ACC: Test de TOLERANCIA de ajuste REPROBADO\n");
                        Ac_Tol = Ac_Tol * 1.08;
                        pause(2);
                    }
                }else{
                    fprintf(stderr, "ACCEL: AMPL Y OFFS NEGATIVOS\n");
                    pause(2);
                    return;
                }

                if (Mg_Adj_new(0) != -1.0f){ // Si es diferente de -1
                    Vector Mg_TamH_new = Mg_Adj_new;
                    // Proceso de aprobación/Rechazo de los nuevos parámetros de ajuste
                    Vector Mg_AdjError = 1.0f - sum( pow( (Mg_capt-Mg_Adj_new(3,5)) / Mg_Adj_new(0,2) ), 2);
                    Vector Mg_AdjRel = sqrt(1.0f/(1.0f-Mg_AdjError));
                        Vector if_Mg_inv = (0 < Mg_AdjRel) && (Mg_AdjRel < 1);
                    Mg_AdjRel = (!if_Mg_inv)*Mg_AdjRel + if_Mg_inv/Mg_AdjRel;
                    float Mg_Adj_TotalErr = sum(Mg_AdjRel) / Mg_capt.getDimV();
                    fprintf(stderr, "MgTol_Err = %.3f\n\n",Mg_Adj_TotalErr);
                    if (Mg_Adj_TotalErr < Mg_Tol){
                        Mg_Tol = 0.7*(Mg_Tol) + 0.3*(1.08);  //  = Mg_Tol - 0.4*(Mg_Tol-1.08)
                        // If TRUE, enonces son parámetros validos, y los actualizamos
                        Vector Mg_Adj_new_Sens = Mg_Adj_new(0)/Mg_Adj_new(0,2);
                        Vector Mg_Adj_new_Offs = Mg_Adj(3,5) + Mg_Adj_new(3,5)/Mg_Adj(0,2);
                        Mg_Adj_new_Sens = Mg_Adj(0,2)*Mg_Adj_new(0,2);

                        // Se mezclan los viejos y los nuevos parametros con 60% y 40% c/u
                        Mg_Adj_new = append(Mg_Adj_new_Sens, Mg_Adj_new_Offs);
                        Mg_Adj = 0.6*Mg_Adj + 0.4*Mg_Adj_new;
                        Vector VPrint = Mg_TamH_new(0)/Mg_Adj(0,2);
                        fprintf(stderr, "Mg_Adj =");  fprintf(stderr, "   %.3f   %.3f   %.3f", VPrint[0], VPrint[1], VPrint[2]);
                            fprintf(stderr, "   %.3f   %.3f   %.3f \n", Mg_Adj[3], Mg_Adj[4], Mg_Adj[5]);
                        pause(5);
                    }else{
                        fprintf(stderr, "MAG: Test de TOLERANCIA de ajuste REPROBADO\n");
                        Mg_Tol = Mg_Tol * 1.08;
                        pause(2);
                    }
                }else{
                    fprintf(stderr, "MAGNET: AMPL Y OFFS NEGATIVOS\n");
                    pause(2);
                    return;
                }

            }
            j = j + 1;
            fprintf(stderr, "%d  VALOR(ES) CAPTURADO(S) \n",j);
        }
    }




    // Algoritmo de alineación en pose-T}
    float Align_Err = norm(f);
    fprintf(stderr, "   Align_Err = %.3f \n", Align_Err);
    if(!isAligned){
        if(loopType == Align){
            float Align_Tol = 0.200;
            if(aCSSV == 0 && mCSSV == 0 && Align_Err < Align_Tol){
                fprintf(stderr, "Sensor alineado\n");
                delay(5000);
                MatrizCuadrada Mpu_init_inv = SErotM;  // Mpu_init^-1 = SErotM (Previamente invertida con "t()")
                Align_Ajd = Mpu_init_inv;  //Euler2RotMat(UE_init) *
                isAligned = true;
            }
        }
    }

    //% COMUNICACIÓN UDP MATLAB - UNREAL
    //    Vector SEeuler = RotMat2Euler(SErotM.t());
    //    SEeuler = (SEeuler < 0.0f)*360.0f + SEeuler;
    //    SEeuler(1) = 360 - SEeuler(1);
    //    SEeuler(2) = 360 - SEeuler(2);

//    QString datSEND = String(SEeuler(0))+","+String(SEeuler(1))+","+String(SEeuler(2));
//    QByteArray prueba = QByteArray(datSEND.toAscii());
//    fprintf(stderr, "%s\n", prueba.data());

    i = i+1;
}





void Mpu9250::AcGy_Reset(void){
    MARGwriteByte(MPU9250_ADDR, CS, ACC_GYRO_RESET, 0x80); //Reset Accel-Gyro Registers
}

int Mpu9250::AcGy_setUp(void){    
    //Indispensable para usar "SPI" sobre el Magnétometro MPU9250
    MARGwriteByte(MPU9250_ADDR, CS, MPU_USER_CTRL, 0x30);    // I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
    MARGwriteByte(MPU9250_ADDR, CS, MPU_INT_PIN_CFG, 0x12);
    MARGwriteByte(MPU9250_ADDR, CS, MPU_I2C_MST_CTRL, 0x0D);    // I2C configuration multi-master  IIC 400KHz

    // Configurar acelerometro
    MARGwriteByte(MPU9250_ADDR, CS, 28, ACC_FSCALE_2_G);
    MARGwriteByte(MPU9250_ADDR, CS, ACC_CONFIG_1+1, 0x06);  //Acc_DLPF => 5 Hz
    // Configurar giroscopio
    MARGwriteByte(MPU9250_ADDR, CS, 27, GYRO_FSCALE_250_DPS);
    MARGwriteByte(MPU9250_ADDR, CS, ACC_GYRO_CONFIG, 0x02);  //Gyro_DLPF Bandwidth ==> 92 Hz

    uint8_t Data;
    MARGread(MPU9250_ADDR, CS, MPU_USER_CTRL, 1, &Data);
    if(Data != 0x30) return -1;
    MARGread(MPU9250_ADDR, CS, MPU_INT_PIN_CFG, 1, &Data);
    if(Data != 0x12) return -2;
    MARGread(MPU9250_ADDR, CS, MPU_I2C_MST_CTRL, 1, &Data);
    if(Data != 0x0D) return -3;

    Serial.println("   .........AcGy_SetUP() Executed.........");
    return 0;
}

void Mpu9250::Mag_Reset(void){
    MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_1+1, 0x01);   //Soft Reset Magnetometer Registers
    // Configurar magnetometro
    //MARGwriteByte(MPU9250_ADDR, CS, 0x37, 0x02); //Registro INT_PIN_CFG
}

int Mpu9250::Mag_setUp(void){
    // Una vez sea posible establecer comunicación con el Magnetómetro, entonces:
    MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_1, MAG_MODE_1 | MAG_FSCALE_16_bit);

    uint8_t Data;
    MAGread(MAG_ADDRESS, CS, MAG_CNTL_1, 1, &Data);
    if(Data != MAG_MODE_1 | MAG_FSCALE_16_bit) return -1;

    Serial.println("   .........Mag_SetUP() Executed.........");
    return 0;
}
