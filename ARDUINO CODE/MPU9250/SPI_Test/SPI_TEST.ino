//MISO - Pin D12
//MOSI - Pin D11
//SCK  - Pin D13
//CS   - Pin D7 (arbitrario)
#define SPI_PROTOCOL
#include <SPI.h>

#define ACC_GYRO_CONFIG  0x1A   //Configuraciones generales

/* VARIABLES GLOBALES */
uint8_t Acc_Gyro[14];  //Accel-Gyro DATA
uint8_t Mag[7];        //Magnetometer DATA
uint8_t ST1;           //Magnetometer "Status" value

String sAUX;
float MAG_FS = 0.15;
float ACC_FS = 0.59875e-3; //   (9.81 * 2) / 2^15
float GYR_FS = 7.6294e-3;  //    250 / 2^15




void SPIread(uint8_t CS, uint8_t Reg, uint8_t nBytes, uint8_t* Data){
  digitalWrite(CS, LOW);
  delay(1);
  SPI.transfer(Reg | 0b10000000); //Primer bit en "1" para Lectura
  uint8_t index = 0;
  while (index < nBytes){
    Data[index++] = SPI.transfer(0x00);
  }
  digitalWrite(CS, HIGH);
}

void SPIwriteByte(uint8_t CS, uint8_t Reg, uint8_t Data){
   digitalWrite(CS, LOW);
   delay(1);
   SPI.transfer(Reg);
   SPI.transfer(Data);
   digitalWrite(CS, HIGH);
}

void SPIwriteNBytes(uint8_t CS, uint8_t Reg, uint8_t nBytes, uint8_t* Data){
  digitalWrite(CS, LOW);
  delay(1);
  SPI.transfer(Reg | 0b00000000); //Primer bit en "0" para Escritura
  uint8_t index = 0;
  while (index < nBytes){
    SPI.transfer(Data[index++]);
  }
  digitalWrite(CS, HIGH);
}


void setup()
{
  Serial.begin(115200);
  
  #ifdef SPI_PROTOCOL
  uint8_t CS = 7;
  SPI.begin();
  delay(100);
  pinMode(CS, OUTPUT);
  uint8_t Dato[] = {4, 0, 6, 0, 0};
  SPIwriteNBytes(CS, ACC_GYRO_CONFIG, 3, Dato);  //Gyro_DLPF => 5 Hz
  uint8_t Data[5];
  while(true){
    for(int Reg=0; Reg<=126; Reg++){
      SPIread(CS, Reg, 1, Data);
      sAUX = "Registro #"+String(Reg)+" =  "+String(Data[0]);
      Serial.println(sAUX);
    }
    while(true){
      
    }
    delay(10000);
  }
  #endif
}


void loop(){
}
