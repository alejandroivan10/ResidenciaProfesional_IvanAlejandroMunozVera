#ifndef REGISTERSANDMASK_H
#define REGISTERSANDMASK_H

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
#define MPU_INT_PIN_CFG  0x37
#define MPU_USER_CTRL    0x6A
#define MPU_I2C_MST_CTRL 0x24
#define   MAG_DATA         0x03  //Leer 7 registros(Two complement big endian):
           //Mag(1-6) Status(7)        Primer byte LOW, 2nd byte HIGH
#define   MAG_CNTL_1       0x0A  //Registro #1 de configuración del sensor
#define   MAG_CNTL_2       0x0B
#define   MAG_ASTC         0x0C  // SELF-TEST Register
#define   MAG_STATUS_1     0x02  //Status register #1 (Data ready - Data Overrun)
#define   MAG_ASAX         0x10  //Sensitivity adjudment values. Leer 3 registros

/*  REGISTROS PARA "SPI" EN MPU9250*/
#define MPU_I2C_SLV0_ADDR     0x25
#define MPU_I2C_SLV0_REG      0x26
#define MPU_I2C_SLV0_CTRL     0x27
#define MPU_I2C_SLV0_DO       0x63
#define MPU_EXT_SENS_DATA_00  0x49

/*     CONTANTES GLOBALES     */
#define  ACC_SENS   2048   //  32768 LSB / 16 G
#define  GYRO_SENS  33    //  32768 LSB / 1000 deg/seg


/* ---------------------------- PROGRAM STARTS ------------------------------- */
#define   MAG_MODE_MASK       ob11110000  //Register 0xA [3:0 bit]
#define   MAG_MODE_2          0x06

#define MPU_FIFO_EN      0x23   //   
#define MPU_USR_CNTL     0x6A   // 


#endif
