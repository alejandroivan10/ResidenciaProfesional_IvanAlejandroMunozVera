#include <QDebug>
#include <QString>

#include "wiringPi.h"
#include "wiringPiSPI.h"
#include "arduino.h"
#include "Complementos.h"
#include "RegistersAndMask.h"
#include "SPI.h"

_SPI SPI;
_Serial Serial;

void _Serial::print(QString cadena){
    //qDebug() << cadena;
    fprintf(stderr, cadena.toLocal8Bit().data());
}

void _Serial::print(float val){
    fprintf(stderr, "%.3f", val);
}

void _Serial::println(QString cadena){
    qDebug() << cadena;
}

void _Serial::println(float val){
    fprintf(stderr, "%.3f \n", val);
}

void _Serial::println(void){
    fprintf(stderr, "\n");
}

QString String (float value){
    return QString::number(value, 'f', 3);
}

QString String (int value){
    return QString::number(value);
}














//////////// SPI.h  //////////////////
int _SPI::setClockDivider(uint8_t Divider){    ///Cuidado: Unable to open SPI: demasiados archivos abiertos
    int success = wiringPiSPISetupMode(0, 16000000/Divider, 3);
    return success;
}

uint8_t _SPI::transfer(uint8_t Data){
    wiringPiSPIDataRW(0, &Data, 1);
    return Data;
}












//////////// Complementos.h  //////////////////

#define READ_FLAG 0b10000000
#define WRITE_FLAG 0b01111111

#if defined(I2C_PROTOCOL)
//Funcion auxiliar lectura
void I2Cread(uint8_t Address, uint8_t Register, uint8_t nBytes, uint8_t* Data)
{
   Wire.beginTransmission(Address);
   Wire.write(Register);
   Wire.endTransmission();

   Wire.requestFrom(Address, nBytes);
   uint8_t index = 0;
   while (Wire.available())
      Data[index++] = Wire.read();
}

// Funcion auxiliar de escritura
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
   Wire.beginTransmission(Address);
   Wire.write(Register);
   Wire.write(Data);
   Wire.endTransmission();
}

void I2CwriteNBytes(uint8_t Address, uint8_t Register, uint8_t nBytes, uint8_t* Data)
{
   Wire.beginTransmission(Address);
   Wire.write(Register);
   for(int i=0; i<nBytes; i++){
    Wire.write(Data[i]);
   }
   Wire.endTransmission(true);
}

#elif defined(SPI_PROTOCOL)
uint us_delay = 1000;

void SPIread(uint8_t CS, uint8_t Reg, uint8_t nBytes, uint8_t* Data){
  digitalWrite(CS, LOW);
  delayMicroseconds(us_delay);
  SPI.transfer(Reg | 0b10000000); //Primer bit en "1" para Lectura
  uint8_t index = 0;
  while (index < nBytes){
    Data[index] =  0;
    Data[index] = SPI.transfer(0x00);
    ++index;
  }
  digitalWrite(CS, HIGH);
  delayMicroseconds(us_delay);
}

void SPIwriteByte(uint8_t CS, uint8_t Reg, uint8_t Data){
  digitalWrite(CS, LOW);
  delayMicroseconds(us_delay);
  SPI.transfer(Reg & 0b01111111);
  SPI.transfer(Data);
  digitalWrite(CS, HIGH);
  delayMicroseconds(us_delay);
}

void SPIwriteNBytes(uint8_t CS, uint8_t Reg, uint8_t nBytes, uint8_t* Data){
  digitalWrite(CS, LOW);
  delayMicroseconds(us_delay);
  SPI.transfer(Reg & 0b01111111); //Primer bit en "0" para Escritura
  uint8_t index = 0;
  while (index < nBytes){
    SPI.transfer(Data[index++]);
  }
  digitalWrite(CS, HIGH);
  delayMicroseconds(us_delay);
}

  #ifdef MPU9250
void SPIreadMg(uint8_t Addr, uint8_t CS, uint8_t Reg, uint8_t nBytes, uint8_t* Data){
  SPIwriteByte(CS, MPU_I2C_SLV0_ADDR, Addr | READ_FLAG);  // Prepare the I2C slave addres and set for read.
  SPIwriteByte(CS, MPU_I2C_SLV0_REG, Reg);  // I2C slave 0 register address from where "data" will be collected
  if (nBytes > 7)  {return;}
  SPIwriteByte(CS, MPU_I2C_SLV0_CTRL, 0x80 | nBytes);
  delay(2);
  SPIread(CS, MPU_EXT_SENS_DATA_00, nBytes, Data);
}

void SPIwriteByteMg(uint8_t Addr, uint8_t CS, uint8_t Reg, uint8_t Data){
  SPIwriteByte(CS, MPU_I2C_SLV0_ADDR, Addr & WRITE_FLAG);  // Prepare the I2C slave addres and set for read.
  SPIwriteByte(CS, MPU_I2C_SLV0_REG, Reg);  // I2C slave 0 register address from where "data" will be collected
  SPIwriteByte(CS, MPU_I2C_SLV0_DO, Data);
  SPIwriteByte(CS, MPU_I2C_SLV0_CTRL, 0x81);
}
  #endif
#endif



////****************************************////
Mean::Mean(uint8_t _NumOfValues){
  NumOfValues = _NumOfValues;
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
  float NOV = float(NumOfValues);
  mean[0] = (mean[0]*NOV - val[0][ix] + NewX)/NOV;
  mean[1] = (mean[1]*NOV - val[1][ix] + NewY)/NOV;
  mean[2] = (mean[2]*NOV - val[2][ix] + NewZ)/NOV;
  val[0][ix] = NewX;
  val[1][ix] = NewY;
  val[2][ix] = NewZ;
  (*idx)++;
}
