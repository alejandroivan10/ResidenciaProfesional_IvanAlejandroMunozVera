#ifndef SETUP_H
#define SETUP_H

#include "RegistersAndMask.h";

void MARG_Reset(){
  I2CwriteByte(MPU9250_ADDR, ACC_GYRO_RESET, 0x80); //Reset Accel-Gyro Registers
  I2CwriteByte(MAG_ADDRESS, MAG_CNTL_1+1, 0x01);   //Soft Reset Magnetometer Registers
     // Configurar magnetometro
  //I2CwriteByte(MPU9250_ADDR, 0x37, 0x02); //Registro INT_PIN_CFG
}

void MARG_setUp(){
  I2CwriteByte(MPU9250_ADDR, ACC_GYRO_CONFIG, 0x06);  //Gyro_DLPF => 5 Hz
  I2CwriteByte(MPU9250_ADDR, ACC_CONFIG_1+1, 0x06);  //Acc_DLPF => 5 Hz

  // Configurar acelerometro
   I2CwriteByte(MPU9250_ADDR, 28, ACC_FSCALE_2_G);
   I2CwriteByte(MPU9250_ADDR, ACC_CONFIG_1+1, 0x06);  //Acc_DLPF => 5 Hz
   // Configurar giroscopio
   I2CwriteByte(MPU9250_ADDR, 27, GYRO_FSCALE_250_DPS);
   I2CwriteByte(MPU9250_ADDR, ACC_GYRO_CONFIG, 0x02);  //Gyro_DLPF => 5 Hz
   // Configurar magnetometro
   I2CwriteByte(MPU9250_ADDR, 0x37, 0x02); //Registro INT_PIN_CFG

   I2CwriteByte(MAG_ADDRESS, MAG_CNTL_1, MAG_MODE_2 | MAG_FSCALE_16_bit);
   //Serial.println(".........SetUP Executed.........");
}



#endif
