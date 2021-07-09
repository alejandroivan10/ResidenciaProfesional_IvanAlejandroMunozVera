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
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) {
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}

// Funcion auxiliar de escritura
int I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data) {
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  if (Wire.endTransmission() != 0) return -1;
  else return 0;
}



uint8_t Mag[7];        //Magnetometer DATA
float AX, AY, AZ, GX, GY, GZ, MX, MY, MZ; //9 GDL´s DATA
uint8_t ST1;
const uint8_t ASA[3] = {180, 181, 170};        //Magnetometer sensivility adjudment
float Mag_ADJ[3];

uint8_t Data;
void setup() {
  Wire.begin();
  Serial.begin(115200);
  //I2CwriteByte(MAG_ADDRESS, MAG_CNTL_1+1, 0x01); //Soft Reset
  delay(5000);

#if defined(MAG_PROOF)
  MARG_Reset();
  delay(100);
  MARG_setUp();
  uint8_t Dato[2] = {77, 39};
  uint8_t Data[2];
  delay(100);
  while (true) {
    //MAGwriteByte(MAG_ADDRESS, CS, 0x0C, Dato); //Gyro_DLPF => 5 Hz
    for (uint8_t Reg = 0; Reg <= 127; Reg++) {
      Data[0] = 67;
      MARGread(Reg, CS, 0x01, 1, Data);
      //MAGread(MAG_ADDRESS, CS, Reg, 1, Data);
      //sAUX = "Registro #"+String(Reg)+" =  "+String(Data[0]);
      if (Data[0] != 67) {
        sAUX = "Respuesta de la dirección: " + String(Reg);
        Serial.println(sAUX);
      }
    }
    Serial.println();
    delay(10000);
  }
#endif
}




void loop() {
  delay(500);
  uint8_t Data;
  for (uint8_t Reg = 0; Reg <= 126; Reg++) {
    I2Cread(MPU9250_ADDR, Reg, 1, &Data);
    sAUX = "Registro #" + String(Reg) + " =  " + String(Data);
    Serial.println(sAUX);
  }

  while (true) {
  }
}
