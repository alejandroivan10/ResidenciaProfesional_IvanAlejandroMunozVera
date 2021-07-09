#ifndef MPU9250_H
#define MPU9250_H


#include <QString>
#include "matlab.h"
#include "Complementos.h"


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


/* VARIABLES GLOBALES */
extern uint8_t Acc_Gyro[14];  //Accel-Gyro DATA
extern uint8_t Mag[7];        //Magnetometer DATA


/**********************   FINAL CóDIGO ARDUINO   *******************/


extern QString sAUX;
extern const float MAG_FS;
extern const float ACC_FS; //   (9.81 * 2) / 2^15
extern const float GYR_FS;  //    250 / 2^15


typedef std::initializer_list<float> floatList;
class MocapSuit;


class Mpu9250{
public:
    Mpu9250(uint _CS, MocapSuit& _Suit, floatList _Ac_Adj,
            floatList _Gy_Adj, floatList _Mg_Adj, floatList _UE_init);
    ~Mpu9250();

    enum loopFlag{
        Normal,
        Align,
        NoImprimir
    };

    void setup(void);
    void loop(loopFlag loopType = Normal);

    void AcGy_Reset(void);
    int AcGy_setUp(void);
    void Mag_Reset(void);
    int Mag_setUp(void);
    friend class MocapSuit;

private:
    const uint CS;   // Chip Select
    const Vector UE_init; // Ángulos de Euler en la pose-T de Unreal
    MatrizCuadrada Align_Ajd = {1,0,0,  0,1,0,  0,0,1};
    bool isAligned = false;
    uint id = 0;
    static uint countOfSens;

    float AX, AY, AZ, GX, GY, GZ, MX, MY, MZ; //9 GDL´s DATA
    float AR, GR, MR, AR_ant, GR_ant, MR_ant, delta_AR, delta_GR, delta_MR;
    int16_t Acc_Bias[3], Gyr_Bias[3], Mag_Bias[3]; //Sumar Offset
    float Acc_Adj[3], Gyr_Adj[3], Mag_Adj[3]; //Multiplicar Offset
    uint8_t ST1;           //Magnetometer "Status" value

    /////////////////   Complementos.h   //////////////////////

    uint8_t Nmuestras = 3;
    Mean Ac_mean = Mean(Nmuestras);
    Mean Gy_mean = Mean(Nmuestras);
    Mean Ma_mean = Mean(Nmuestras);



    ///////// DESPUéS DEL VOID SETUP, PERO ANTES DEL VOID LOOP:  //////////
    int16_t ax_ant, ay_ant, az_ant; uint8_t aCSSV = 0; //Count Stagnant Sensor Values
    int16_t mx_ant, my_ant, mz_ant; uint8_t mCSSV = 0; //Count Stagnant Sensor Values
    uint32_t t_act, t_ant;
    float t_delta;




    /////// COMIENZAN VARIABLES MATLAB: ///////
    // // System constants
    float deltat = 0.001f; // sampling period in seconds (shown as 1 ms)
    float gyroMeasError = 3.14159265358979f * (5.0f / 180.0f); // gyroscope measurement error in rad/s (shown as 5 deg/s)
    float gyroMeasDrift = 3.14159265358979f * (0.2f / 180.0f); // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
    float beta = sqrt(5.0f / 4.0f) * gyroMeasError; // compute beta beta_init = 2)
    float zeta = sqrt(3.0f / 4.0f) * gyroMeasDrift; // compute zeta

    //  Global system variables
    float b_x = 0.454f, b_y = -0.189f, b_z = -0.871f; // reference direction of flux in earth frame
    Vector b = {b_x,b_y,b_z};
    const Vector b_E = b/norm(b);
    const Vector g_E = {0.0f, 0.0f, 1.0f};
    float w_bx = 0.0f,  w_by = 0.0f, w_bz = 0.0f; // estimate gyroscope biases error

    // // Local system variables
    //SEqDot_omega = zeros(1,4); // quaternion rate from gyroscopes elements
    Vector f = zeros(6); // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64;
    Vector SEqHatDot = zeros(4); // estimated direction of the gyroscope error
    //w_err(1); w_err(2); w_err(3); // estimated direction of the gyroscope error (angular)
    //h_S(1), h_S(2), h_S(3); // computed flux in the earth frame
    // axulirary variables to avoid reapeated calcualtions
    Vector SEq = {1.0f, 0.0f, 0.0f, 0.0f};
    MatrizCuadrada SErotM = Quat2RotMat(SEq).t();

    //// inicializar CALIBRACIÓN
    uint8_t i_capt=0; // Index de captura
    uint8_t nCapt = 20; // #Num de Valores por capturar
    float Ac_Tol = 1.08f; // Tolerancia de Error Total Accel
    float Mg_Tol = 1.08f; // Tolerancia de Error Total Magnet
    Matriz Ac_capt = ones(nCapt,3); // = ones(nCapt,3); //It can't be zeros(), because make "inf" values
    Matriz Mg_capt = ones(nCapt,3); //Same comment of "Ac_capt"
    Matriz Gy_capt = ones(nCapt,3);
    Vector Ac_ant = {0.1f, 0.1f, 0.1f}; //Same comment of "Ac_capt"
    Vector Gy_ant = {0.1f, 0.1f, 0.1f};
    Vector Mg_ant = {0.1f, 0.1f, 0.1f}; //Same comment of "Ac_capt"

    uint8_t nCalib = 2;
    Vector d_Ac = ones(nCalib); //It can't be zeros()
    Vector d_Mg = ones(nCalib);
    Vector Ac_Adj; // = {9.5669,    8.8451,   10.6146,   0.0165,    0.7612,    1.6092};
    Vector Ac_Adj_Sens = ones(3);// = Ac_Adj(0)/Ac_Adj(0,2); //Acel_Calib
    Vector Ac_Adj_Offs = zeros(3);// = Ac_Adj(3,5);
    Vector Gy_Adj;// = {-1.3005,    0.0240,   -0.6723}; // Gyro_Calib
    Vector Mg_Adj;// = {39.6496,   43.5980,   40.8667,   -5.3843,   10.1385,  -12.7517};
    Vector Mg_Adj_Sens = ones(3);// = Mg_Adj(0)/Mg_Adj(0,2); //Mag_Calib
    Vector Mg_Adj_Offs = zeros(3);// = Mg_Adj(3,5);

    uint32_t i=0,  j=0;
    float t1 = 0.0f, t2 = 0.0f, time_sensor = 0.0f;
};

#endif // MPU9250_H
