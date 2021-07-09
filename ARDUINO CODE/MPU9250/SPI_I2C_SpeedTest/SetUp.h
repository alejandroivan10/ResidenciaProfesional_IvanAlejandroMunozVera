#ifndef SETUP_H
#define SETUP_H

#include "RegistersAndMask.h";

void MARG_Reset(){
  MARGwriteByte(MPU9250_ADDR, CS, ACC_GYRO_RESET, 0x80); //Reset Accel-Gyro Registers
  MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_1+1, 0x01);   //Soft Reset Magnetometer Registers
     // Configurar magnetometro
  //MARGwriteByte(MPU9250_ADDR, CS, 0x37, 0x02); //Registro INT_PIN_CFG
}

void MARG_setUp(){
  MARGwriteByte(MPU9250_ADDR, CS, ACC_GYRO_CONFIG, 0x06);  //Gyro_DLPF => 5 Hz
  MARGwriteByte(MPU9250_ADDR, CS, ACC_CONFIG_1+1, 0x06);  //Acc_DLPF => 5 Hz

  // Configurar acelerometro
   MARGwriteByte(MPU9250_ADDR, CS, 28, ACC_FSCALE_2_G);
   MARGwriteByte(MPU9250_ADDR, CS, ACC_CONFIG_1+1, 0x06);  //Acc_DLPF => 5 Hz
   // Configurar giroscopio
   MARGwriteByte(MPU9250_ADDR, CS, 27, GYRO_FSCALE_250_DPS);
   MARGwriteByte(MPU9250_ADDR, CS, ACC_GYRO_CONFIG, 0x02);  //Gyro_DLPF => 5 Hz
   // Configurar magnetometro
   MARGwriteByte(MPU9250_ADDR, CS, 0x37, 0x02); //Registro INT_PIN_CFG

   MAGwriteByte(MAG_ADDRESS, CS, MAG_CNTL_1, MAG_MODE_2 | MAG_FSCALE_16_bit);
   //Serial.println(".........SetUP Executed.........");
}



#endif
