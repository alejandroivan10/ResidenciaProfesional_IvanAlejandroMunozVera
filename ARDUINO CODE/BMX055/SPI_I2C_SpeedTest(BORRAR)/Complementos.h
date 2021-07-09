
//Funcion auxiliar lectura
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
   Wire.beginTransmission(Address);
   Wire.write(Register);
   Wire.endTransmission();
 
   Wire.requestFrom(Address, Nbytes);
   uint8_t index = 0;
   while (Wire.available()){
      Data[index++] = Wire.read();
//      Serial.print("NewValue = "+String(Data[index]));
//      Serial.print("  Addr = "+String(Address)+"  Indx = ");
//      Serial.println(index);
//      index++;
   }
}
 
// Funcion auxiliar de escritura
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
   Wire.beginTransmission(Address);
   Wire.write(Register);
   Wire.write(Data);
   Wire.endTransmission();
}

void I2CwriteNBytes(uint8_t Address, uint8_t Register, uint8_t* Data, uint8_t Nbytes)
{
   Wire.beginTransmission(Address);
   Wire.write(Register);
   for(int i=0; i<Nbytes; i++){
    Wire.write(Data[i]);
   }
   Wire.endTransmission(true);
}




// Clase Index y Mean se utilizan para obtener una media móvil
class Index{
public:
  Index(uint8_t _NumOfIndices){
    NumOfIndices = _NumOfIndices;
    idx = 0;
  }
  uint8_t getIndex(){return idx;
  }
  operator++(){
    if(idx >= NumOfIndices - 1) idx = 0;
    else idx++;
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
Mean::Mean(uint8_t _NumOfValues){
  NumOfValues = _NumOfValues;
  float NOV = float(NumOfValues);
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
  float NOV = NumOfValues;
  mean[0] = (mean[0]*NOV - val[0][ix] + NewX)/NOV;
  mean[1] = (mean[1]*NOV - val[1][ix] + NewY)/NOV;
  mean[2] = (mean[2]*NOV - val[2][ix] + NewZ)/NOV;
  val[0][ix] = NewX;
  val[1][ix] = NewY;
  val[2][ix] = NewZ;
  (*idx)++;
}
