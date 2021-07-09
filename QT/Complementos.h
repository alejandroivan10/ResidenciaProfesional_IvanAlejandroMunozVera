#ifndef COMPLEMENTOS_H
#define COMPLEMENTOS_H

#include <cstdint>

#define SPI_PROTOCOL
#define MPU9250

/*
 * INDISPENSABLES:
 *  + Definir un protocolo de comunicación: #define "SPI_PROTOCOL" o "I2C_PROTOCOL"
 *  + Definir un sensor: #define "MPU9250" o "BMX055"
*/



#define READ_FLAG 0b10000000
#define WRITE_FLAG 0b01111111

#if defined(I2C_PROTOCOL)
    //Funcion auxiliar lectura
void I2Cread(uint8_t Address, uint8_t Register, uint8_t nBytes, uint8_t* Data);
    // Funcion auxiliar de escritura
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data);
void I2CwriteNBytes(uint8_t Address, uint8_t Register, uint8_t nBytes, uint8_t* Data);

#elif defined(SPI_PROTOCOL)
void SPIread(uint8_t CS, uint8_t Reg, uint8_t nBytes, uint8_t* Data);
void SPIwriteByte(uint8_t CS, uint8_t Reg, uint8_t Data);
void SPIwriteNBytes(uint8_t CS, uint8_t Reg, uint8_t nBytes, uint8_t* Data);

  #ifdef MPU9250 
void SPIreadMg(uint8_t Addr, uint8_t CS, uint8_t Reg, uint8_t nBytes, uint8_t* Data);
void SPIwriteByteMg(uint8_t Addr, uint8_t CS, uint8_t Reg, uint8_t Data);
  #endif
#endif



/////////////////////////////////////////////////////////////////

class Index{
public:
  Index(uint8_t _NumOfIndices){
    NumOfIndices = _NumOfIndices;
    idx = 0;
  }
  uint8_t getIndex(){return idx;}
  Index operator++(int a){
    if(idx >= NumOfIndices - 1){
        Index i_temp(NumOfIndices);
        idx = 0;
        i_temp.idx = 0;
        return i_temp;
    }
    else{
        Index i_temp(NumOfIndices);
        idx++;
        i_temp.idx = idx;
        return i_temp;
    }
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

#endif
