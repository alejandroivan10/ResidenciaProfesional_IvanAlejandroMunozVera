#include <initializer_list>
#include <cstdint>
#include <cstdlib>
#include <QDebug>
#include <stdio.h>
#include <math.h>

#include "matlab.h"
#include "wiringPi.h"
#include <QDebug>


Vector Vector::operator()(uint8_t p1, uint8_t p2) const{
    uint8_t _dim = p2+1-p1;
    if(p2 < dim && dim >= _dim && p2 > p1){
        Vector v(_dim);
        for(int i=0;i<_dim;i++){
            v[i] = vtr[p1 + i];
        }
        return v;
    }else{
        fprintf(stderr, "Error Vector::Operator()(p1,p2): Las dimensiones son incorrectas \n");
        exit(0);
    }
}
void Vector::operator=(Vector& v2){
    if(dim == v2.getDim()){
        for(int i=0;i<dim;i++){
            vtr[i] = v2[i];
        }
    }
    else{
        fprintf(stderr, "Error Vector::Operator=(&): Las dimensiones son diferentes \n");
        exit(0);
    }
}
void Vector::operator=(Vector&& v2){
    if(dim == v2.getDim()){
        for(int i=0;i<dim;i++){
            vtr[i] = v2[i];
        }
    }
    else{
        fprintf(stderr, "Error Vector::Operator=(&&): Las dimensiones son diferentes \n");
        exit(0);
    }
}

Vector operator*(const Vector& _v, float cte){
    uint8_t _dim = _v.getDim();
    Vector v(_v);
    for(int i=0;i<_dim;i++) v(i)=v(i)*cte;
    return v;
}
Vector operator*(float cte, const Vector &_v)
{
    uint8_t _dim = _v.getDim();
    Vector v(_v);
    for(int i=0;i<_dim;i++) v(i)=v(i)*cte;
    return v;
}
Vector operator*(const Vector& v1, const Vector& v2){
    if(v1.getDim() == v2.getDim()){
        uint8_t _dim = v1.getDim();
        Vector Res(_dim);
        for(int i=0;i<_dim;i++){
            Res[i] = v1[i] * v2[i];
        }
        return Res;
    }else{
        fprintf(stderr, "Error friend Vector operator*(Vector, Vector) \n");
        exit(0);
    }
}
Vector operator/(const Vector& _v, float cte){
    Vector v(_v);
    uint8_t _dim = v.getDim();
    for(int i=0;i<_dim;i++) v(i)=v(i)/cte;
    return v;
}
Vector operator/(float cte, const Vector& _v){
    Vector v(_v);
    uint8_t _dim = v.getDim();
    for(int i=0;i<_dim;i++) v(i)=cte/v(i);
    return v;
}
Vector operator/(const Vector& v1, const Vector& v2){
    if(v1.getDim() == v2.getDim()){
        uint8_t _dim = v1.getDim();
        Vector Res(_dim);
        for(int i=0;i<_dim;i++){
            Res[i] = v1[i] / v2[i];
        }
        return Res;
    }else{
        fprintf(stderr, "Error Operator/(Vector, Vector) \n");
        exit(0);
    }
}
Vector operator+(const Vector& v1, const Vector& v2){
    if(v1.getDim() == v2.getDim()){
        uint8_t _dim = v1.getDim();
        Vector Res(_dim);
        for(int i=0;i<_dim;i++){
            Res[i] = v1[i] + v2[i];
        }
        return Res;
    }else{
        fprintf(stderr, "Error friend Vector operator+() \n");
        exit(0);
    }
}
Vector operator-(const Vector& v1, const Vector& v2){
    if(v1.getDim() == v2.getDim()){
        uint8_t _dim = v1.getDim();
        Vector Res(_dim);
        for(int i=0;i<_dim;i++){
            Res[i] = v1[i] - v2[i];
        }
        return Res;
    }else{
        fprintf(stderr, "Error friend Vector operator+() \n");
        exit(0);
    }
}
Vector operator-(float cte, const Vector& _v){
    uint8_t _dim = _v.getDim();
    Vector v(_v);
    for(int i=0;i<_dim;i++) v(i) = cte - v(i);
    return v;
}
Vector operator<(const Vector& _v, float cte){
    uint8_t _dim = _v.getDim();
    Vector v(_v);
    for(int i=0;i<_dim;i++){
        if(_v[i] < cte)  v(i) =  1.0f;
        else             v(i) =  0.0f;
    }
    return v;
}
Vector operator<(float cte, const Vector& _v){
    uint8_t _dim = _v.getDim();
    Vector v(_v);
    for(int i=0;i<_dim;i++){
        if(cte < _v[i])  v(i) =  1.0f;
        else             v(i) =  0.0f;
    }
    return v;
}
Vector operator&&(const Vector& v1, const Vector& v2){
    if(v1.getDim() == v2.getDim()){
        uint8_t _dim = v1.getDim();
        Vector v(v1);
        for(int i=0;i<_dim;i++){
            if(v1[i] == 1.0f && v2[i] == 1.0f)  v(i) =  1.0f;
            else                                v(i) =  0.0f;
        }
        return v;
    }else{
        fprintf(stderr, "Error operator&(Vector, Vector) \n");
        exit(0);
    }
}
Vector operator!(const Vector& _v){
    uint8_t _dim = _v.getDim();
    Vector v(_v);
    for(int i=0;i<_dim;i++){
        if(v[i] == 0.0)  v(i) =  1.0f;
        else             v(i) =  0.0f;
    }
    return v;
}

float norm(const Vector &v){
    float sum = 0;
    for(int i=0;i<v.getDim();i++) sum += pow(v(i),2);
    return sqrt(sum);
}
float dot(const Vector &v1, const Vector &v2){
    float sum = 0;
    uint8_t _dim = v1.getDim();
    if(v1.getDim() == v2.getDim()){
        for(int i=0;i<_dim;i++) sum += v1(i)*v2(i);
        return sum;
    }
    else{
        fprintf(stderr, "Dot() Error: Vecotres de diferente dimensión \n");
        return 0;
    }
}
Vector quatconj(const Vector &_v){
    if(_v.getDim() == 4){
        Vector Res(_v);
        Res[1] = - Res[1];
        Res[2] = - Res[2];
        Res[3] = - Res[3];
        return Res;
    }else{
        fprintf(stderr, "quatconj() Error: Vector de dimensión diferente a 4 \n");
        exit(0);
    }
}
Vector append(const Vector &v1, const Vector &v2){
    uint8_t _dim1 = v1.getDim();
    uint8_t _dim2 = v2.getDim();
    Vector Res(_dim1+_dim2);
    for(int i=0;i<_dim1;i++){
        Res[i] = v1[i];
    }
    for(int i=0;i<_dim2;i++){
        Res[_dim1+i] = v2[i];
    }
    return Res;
}
Vector zeros(uint8_t n){
    Vector v(n);
    for(int i=0;i<n;i++) v(i)  = 0;
    return v;
}
Vector ones(uint8_t dim){
    Vector temp(dim);
    for(int i=0;i<dim;i++){
        temp[i] = 1;
    }
    return temp;
}









Matriz::Matriz(uint8_t _dimV, uint8_t _dimH){
    dimV = _dimV;
    dimH = _dimH;
    mtx = new Vector[_dimV];
    for(int i=0;i<dimV;i++){
        mtx[i].resize(_dimH);
    }
}
Matriz::Matriz(const Matriz &_m){
    dimV = _m.getDimV();
    dimH = _m.getDimH();
    mtx = new Vector[dimV];
    for(int i=0;i<dimV;i++){
        mtx[i].resize(dimH);
    }
    for(int i=0;i<dimV;i++){
        for(int j=0;j<dimH;j++){
            mtx[i][j] = _m[i][j];
        }
    }
}
Matriz::Matriz(const Matriz &&_m){
    dimV = _m.getDimV();
    dimH = _m.getDimH();
    mtx = new Vector[dimV];
    for(int i=0;i<dimV;i++){
        mtx[i].resize(dimH);
    }
    for(int i=0;i<dimV;i++){
        for(int j=0;j<dimH;j++){
            mtx[i][j] = _m[i][j];
        }
    }
}


void Matriz::operator=(Matriz& m){
    uint8_t _dimV = m.getDimV();
    uint8_t _dimH = m.getDimH();
    if(dimV == _dimV && dimH == _dimH){
        for(int i=0;i<dimV;i++){
            for(int j=0;j<dimH;j++){
                mtx[i][j] = m[i][j];
            }
        }
    }else{
        fprintf(stderr, "Error operator=(MatrizCuadrada &)");
        exit(0);
    }
}


Vector& Matriz::operator[](uint16_t _idx) const{   // /////////////  REVISAR QUE ESTO FUNCIONE:    CUIDADO
    if(_idx < dimV){
        return mtx[_idx];
    }
    else{
        fprintf(stderr, "Error Matriz Operator[]: Indice mayor que la dimension \n");
        exit(0);
    }
}
Vector& Matriz::operator()(uint8_t _idx) const{
    if(_idx < dimV){
        return mtx[_idx];
    }
    else{
        fprintf(stderr, "Error Matriz Operator(): Indice mayor que la dimension \n");
        exit(0);
    }
}
void Matriz::operator=(Matriz&& m){
    uint8_t _dimV = m.getDimV();
    uint8_t _dimH = m.getDimH();
    if(dimV == _dimV && dimH == _dimH){
        for(int i=0;i<dimV;i++){
            for(int j=0;j<dimH;j++){
                mtx[i][j] = m[i][j];
            }
        }
    }else{
        fprintf(stderr, "Error operator=(MatrizCuadrada &&)");
        exit(0);
    }
}










void MatrizCuadrada::operator=(const MatrizCuadrada& m){
    if(dimV == m.getDim()){
        for(int i=0;i<dimV;i++){
            for(int j=0;j<dimV;j++){
                mtx[i][j] = m[i][j];
            }
        }
    }else{
        fprintf(stderr, "Error operator=(MatrizCuadrada &)");
        exit(0);
    }
}
void MatrizCuadrada::operator=(const MatrizCuadrada&& m){
    if(dimV == m.getDim()){
        for(int i=0;i<dimV;i++){
            for(int j=0;j<dimV;j++){
                mtx[i][j] = m[i][j];
            }
        }
    }else{
        fprintf(stderr, "Error operator=(MatrizCuadrada &&)");
        exit(0);
    }
}


MatrizCuadrada Inversa(const MatrizCuadrada& _mtx){
    uint8_t dim = _mtx.getDim();
    float Mat[dim][2*dim];
    Vector* const temp = _mtx.getMatriz();
    //float Inv[dim][dim];

    for(int i=0;i<dim;i++){
        for(int j=0;j<dim;j++){         // Agregar matriz a la cuál queremos obtener su INVERSA
            Mat[i][j] = temp[i][j];
        }
        for(int j=dim;j<2*dim;j++){    // Agregar matriz identidad
            if(i==(j-dim)) Mat[i][j] = 1;
            else           Mat[i][j] = 0;
        }
    }
    float val_Piv;
    for(int piv=0;piv<(dim-1);piv++){   // piv = (#de Pivote - 1)
    /*   Si el pivote es Igual a "0", Entonces: Intercambiamos filas  */
        if( std::abs(Mat[piv][piv]) > 0.0001 ){
            val_Piv = 1 / Mat[piv][piv];
        }else{
            for(int i=piv+1;i<dim;i++){
                if(Mat[i][piv] > 0.0001){
                    for(int j=0;j<2*dim;j++){
                        float aux = Mat[piv][j];
                        Mat[piv][j] = Mat[i][j];
                        Mat[i][j] = aux;
                    }
                    break;
                }else{
                    if(i == dim-1){
                        fprintf(stderr, "Error Inversa(): La matriz no es invertible \n");
                        MatrizCuadrada error = zerosMC(dim); // La matriz NO tiene inversa
                        return error;
                    }
                }
            }
        }
    /* Problema del pivote igual a "0" resuelto hasta esta linea  */

        for(int j=piv;j<2*dim;j++){     //Multiplicar "fila pivote" para que el pivote sea = 1
            Mat[piv][j] *= val_Piv;
        }
        float val;
        for(int i=piv+1;i<dim;i++){     // Reucción Gauss
            val = Mat[i][piv];
            for(int j=piv;j<2*dim;j++){
                Mat[i][j] = Mat[i][j] - val*Mat[piv][j];
            }
        }
    }

    MatrizCuadrada Inv(dim);
//    float** Inv = new float* [dim];
//    for(int i=0;i<dim;i++) Inv[i] = new float[dim];

/*   Replantear
    for(int i=dim-1;i>=0;i--){
        for(int k=0;k<dim;k++){
            Inv[i][k] = Mat[i][k+dim];
            for(int j=dim-1;j>i;j--){
                Inv[i][k] = Inv[i][k] - Mat[][]*Mat[][];
            }
            Inv[i][k] = Inv[i][k] / Mat[i][i];
        }
    }
   Fin Replantear   */

    for(int j=0;j<dim;j++){
        for(int i=dim-1;i>=0;i--){
            Inv[i][j] = Mat[i][j+dim];
            for(int k=dim-1;k>i;k--){
                Inv[i][j] = Inv[i][j] - Mat[i][k]*Inv[k][j];
            }
            Inv[i][j] = Inv[i][j] / Mat[i][i];
        }
    }

    return Inv;
}

MatrizCuadrada zerosMC(uint8_t _dim){
    MatrizCuadrada temp(_dim);
    for(int i=0;i<_dim;i++){
        for(int j=0;j<_dim;j++){
            temp[i][j] = 0.0f;
        }
    }
    return temp;
}

Matriz ones(uint8_t dim1, uint8_t dim2){
    Matriz temp(dim1, dim2);
    for(int i=0;i<dim1;i++){
        for(int j=0;j<dim2;j++){
            temp[i][j] = 1;
        }
    }
    return temp;
}

//static MatrizCuadrada Q2RM(3);
MatrizCuadrada Quat2RotMat(const Vector &q){
    if(q.getDim() == 4){
        MatrizCuadrada R(3);
        R[0][0] = float(2*pow(q[0],2) + 2*pow(q[1],2) - 1);
        R[0][1] = float(2*(q[1]*q[2] - q[0]*q[3]));
        R[0][2] = float(2*(q[1]*q[3] + q[0]*q[2]));
        R[1][0] = float(2*(q[1]*q[2] + q[0]*q[3]));
        R[1][1] = float(2*pow(q[0],2) + 2*pow(q[2],2) - 1);
        R[1][2] = float(2*(q[2]*q[3] - q[0]*q[1]));
        R[2][0] = float(2*(q[1]*q[3] - q[0]*q[2]));
        R[2][1] = float(2*(q[2]*q[3] + q[0]*q[1]));
        R[2][2] = float(2*pow(q[0],2) + 2*pow(q[3],2) - 1);
        //Q2RM = R;
        return R;
    }else{
        fprintf(stderr, "Quat2Rotmat Error Fatal: Vector no es de dimensión 4 \n");
        exit(0);
    }
}
Matriz operator-(const Matriz& _m, const Vector& _v){
    if(_m.getDimH() == _v.getDim()){
        uint8_t _dimV = _m.getDimV();
        uint8_t _dimH = _m.getDimH();
        Matriz Res(_dimV, _dimH);
        for(int i=0;i<_dimV;i++){
            for(int j=0;j<_dimH;j++){
                Res[i][j] = _m[i][j] - _v[j];
            }
        }
        return Res;
    }else{
        fprintf(stderr, "Error operator-(Matriz, Vector) \n");
        exit(0);
    }
}
Matriz operator/(const Matriz& _m, const Vector& _v){
    if(_m.getDimH() == _v.getDim()){
        uint8_t _dimV = _m.getDimV();
        uint8_t _dimH = _m.getDimH();
        Matriz Res(_dimV, _dimH);
        for(int i=0;i<_dimV;i++){
            for(int j=0;j<_dimH;j++){
                Res[i][j] = _m[i][j] / _v[j];
            }
        }
        return Res;
    }else{
        fprintf(stderr, "Error operator/(Matriz, Vector) \n");
        exit(0);
    }
}
Matriz pow(const Matriz& _m){
    uint8_t _dimV = _m.getDimV();
    uint8_t _dimH = _m.getDimH();
    Matriz Res(_dimV, _dimH);
    for(int i=0;i<_dimV;i++){
        for(int j=0;j<_dimH;j++){
            Res[i][j] = pow(Res[i][j], 2);
        }
    }
    return Res;
}

Vector operator*(const Matriz& _m, const Vector& _v){
    if(_m.getDimH() == _v.getDim()){
        uint8_t _dimH = _m.getDimH();
        uint8_t _dimV = _m.getDimV();
        Vector Res(_dimV);
        for(int i=0;i<_dimV;i++){
            Res[i] = 0.0f;
            for(int j=0;j<_dimH;j++){
                Res[i] += _m[i][j] * _v[j];
            }
        }
        return Res;
    }else{
        fprintf(stderr, "Error operator*(Matriz, Vector): Dimensiones diferentes \n");
        exit(0);
    }
}
Vector operator*(const Vector& _v, const Matriz& _m){
    if(_m.getDimV() == _v.getDim()){
        uint8_t _dimH = _m.getDimH();
        uint8_t _dimV = _m.getDimV();
        Vector Res(_dimH);
        for(uint8_t i=0;i<_dimH;i++){
            Res[i] = 0.0f;
            for(uint8_t j=0;j<_dimV;j++){
                Res[i] += _v[j] * _m[j][i];
            }
        }
        return Res;
    }else{
        fprintf(stderr, "Error operator*(Matriz, Vector): Dimensiones diferentes \n");
        exit(0);
    }
}
MatrizCuadrada operator*(const MatrizCuadrada &_m1, const MatrizCuadrada &_m2){
    if(_m1.getDimH() == _m2.getDimV()){
        uint8_t _dimV  = _m1.getDimV();
        uint8_t _dimH  = _m2.getDimH();
        uint8_t _dimHV = _m1.getDimH();
        MatrizCuadrada Res(_dimV);
        for(uint8_t i=0;i<_dimV;i++){
            for(uint8_t j=0;j<_dimH;j++){
                Res[i][j] = 0.0f;
                for(uint8_t k=0;k<_dimHV;k++){
                    Res[i][j] += _m1[i][k] * _m2[k][j];
                }
            }
        }
        return Res;
    }else{
        fprintf(stderr, "Error operator*(Matriz, Matriz): Dimensiones diferentes \n");
        exit(0);
    }
}

Vector quatmultiply(const Vector& q1, const Vector& q2){
    if(q1.getDim() == 4 && q2.getDim() == 4){
        Vector Res(4);
        Res[0] = q1[0]*q2[0] -(q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]);
        Res[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
        Res[2] = q1[0]*q2[2] + q1[2]*q2[0] + q1[3]*q2[1] - q1[1]*q2[3];
        Res[3] = q1[0]*q2[3] + q1[3]*q2[0] + q1[1]*q2[2] - q1[2]*q2[1];
        return Res;
    }else{
        fprintf(stderr, "Error quatmultiply: Dimensiones diferentes a 4 \n");
        exit(0);
    }
}

static Matriz SysEqf(const Matriz& _m){
    uint8_t _dimV = _m.getDimV();
    uint8_t _dimH = 6;
    Matriz Res(_dimV, _dimH);

    for(int i=0;i<_dimV;i++){
        Res[i][0] = pow(_m[i][0], 2);
        Res[i][1] = -2*_m[i][0];
        Res[i][2] = pow(_m[i][1], 2);
        Res[i][3] = -2*_m[i][1];
        Res[i][4] = pow(_m[i][2], 2);
        Res[i][5] = -2*_m[i][2];
    }
    return Res; //[x.^2 -2*x y.^2 -2*y z.^2 -2*z];
}
MatrizCuadrada mult_Mt2M(const Matriz& m1, const Matriz& m2){
    if(m1.getDimV() == m2.getDimV()  &&  m1.getDimH() == m2.getDimH()){
        uint8_t _dimH = m1.getDimH();
        uint8_t _dimV = m1.getDimV();
        MatrizCuadrada Res(_dimH);
        for(int i=0;i<_dimH;i++){
            for(int j=0;j<_dimH;j++){
                Res[i][j] = 0.0f;
                for(int k=0;k<_dimV;k++){
                    Res[i][j] += m1[k][i]*m2[k][j];
                }
            }
        }
        return Res;
    }else{
        fprintf(stderr, "Errror mult_vvToM \n");
        exit(0);
    }
}

Vector RegresiLinealMultip(const Matriz& _m){   // [Amplitud Bias]
    uint8_t nVal = _m.getDimV();
    // Regresión lineal multivariable: "Ax = Y"  o bien, "SysEq*XRes = SysRes"
    Vector SysRes = ones(nVal); // Resultado del sistema de ecuaciones
    Matriz SysEq = SysEqf(_m); // Sistema de ecuaciones

    //Res = mldivide(SysEq'*SysEq, SysEq'*SysRes);
    MatrizCuadrada Cuad = mult_Mt2M(SysEq, SysEq);
    Cuad = Inversa(Cuad);
    Vector XRes(6);
    if(Cuad[0][0] != 0.0f){
        XRes = SysRes*SysEq;
        XRes = Cuad*XRes;
    }else{
        Vector ampli_bias = {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f};
        return ampli_bias;
    }
//     [re,co] = rref([SysEq SysRes]);
//     XRes = re(1:6,7);
    //SysEq = [x.^2 -2*x y.^2 -2*y z.^2 -2*z];

    if (XRes(0)>0 && XRes(2)>0  && XRes(4)>0){
        // Obtención de parametros elípticos
        float lambda = pow(XRes(1),2)/XRes(0) + pow(XRes(3),2)/XRes(2) + pow(XRes(5),2)/XRes(4);
        float p = lambda/(1+lambda);   float q = 1-p;    //q = 1 - p_org;
        XRes = XRes*q;
        Vector tam = { sqrt(1/XRes(0)),  sqrt(1/XRes(2)),  sqrt(1/XRes(4)) };
        float hx = XRes(1)/XRes(0);  float hy = XRes(3)/XRes(2);  float  hz = XRes(5)/XRes(4);

        Vector ampli_bias = {tam(0), tam(1), tam(2), hx, hy, hz};
        return ampli_bias;
    }else{
        Vector ampli_bias = {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f};
        return ampli_bias;
    }
}
Vector RotMat2Euler(const MatrizCuadrada& R){
    if(R.getDim() == 3){
        // R(Phi).*R(Theta).*R(Psi)
    //     phi   =-atan2d( R(2,3), R(3,3) );
    //     theta =  asind(-R(3,1) );
    //     psi   = atan2d( R(2,1), R(1,1) );

        // Rz(Psi).*Ry(Theta).*Rx(Phi)
        float phi   = atan2( R(2,1), R(2,2) ) * (180.0f/PI);
        float theta =  asin(-R(2,0) ) * (180.0f/PI);
        float psi   = atan2( R(1,0), R(0,0) ) * (180.0f/PI);

        Vector euler = {phi, theta, psi};
        return euler;
    }else{
        fprintf(stderr, "Error: RotMat2Euler() \n");
        exit(0);
    }
}
MatrizCuadrada Euler2RotMat(const Vector& V){
    if(V.getDim() == 3){
        MatrizCuadrada R(3);
        float conv = 3.1425926/180;

        float SPhi = sin(V[0]*conv);
        float CPhi = cos(V[0]*conv);
        float STheta = sin(V[1]*conv);
        float CTheta = cos(V[1]*conv);
        float SPsi = sin(V[2]*conv);
        float CPsi = cos(V[2]*conv);

        /// Rz(Psi).*Ry(Theta).*Rx(Phi)
        R[0][0] = CPsi*CTheta;
        R[0][1] = CPsi*STheta*SPhi - SPsi*CPhi;
        R[0][2] = CPsi*STheta*CPhi + SPsi*SPhi;
        R[1][0] = SPsi*CTheta;
        R[1][1] = SPsi*STheta*SPhi + CPsi*CPhi;
        R[1][2] = SPsi*STheta*CPhi - CPsi*SPhi;
        R[2][0] = -STheta;
        R[2][1] = CTheta*SPhi;
        R[2][2] = CTheta*CPhi;
        return R;
    }else{
        fprintf(stderr, "Error: Euler2RotMat() \n");
        exit(0);
    }
}
Vector sqrt(const Vector& _v){
    uint8_t _dim = _v.getDim();
    Vector v(_v);
    for(int i=0;i<_dim;i++) v(i) = sqrt(v(i));
    return v;
}
Vector sum(const Matriz& _m, uint8_t _idx){
    if(_idx == 1){
        uint8_t _dimV = _m.getDimV();
        uint8_t _dimH = _m.getDimH();
        Vector Res = zeros(_dimH);
        for(int i=0;i<_dimH;i++){
            for(int j=0;j<_dimV;j++)
                Res[i] += _m[j][i];
        }
        return Res;
    }else if(_idx == 2){
        uint8_t _dimV = _m.getDimV();
        uint8_t _dimH = _m.getDimH();
        Vector Res = zeros(_dimV);
        for(int i=0;i<_dimV;i++){
            for(int j=0;j<_dimH;j++)
                Res[i] += _m[i][j];
        }
        return Res;
    }else{
        fprintf(stderr, "Error sum(Matriz, uint8_t) \n");
        exit(0);
    }
}
float sum(const Vector& _v){
    uint8_t _dim = _v.getDim();
    float Res = 0.0f;
    for(int i=0;i<_dim;i++){
        Res += _v[i];
    }
    return Res;
}
static float t1 = 0.0f;
void tic(void){
    t1 = float_t(micros()) / 1000000;
}

float toc(void){
    float t2 = float_t(micros()) / 1000000;
    return t2 - t1;
}
void pause(uint8_t seconds){
    delay(seconds*1000);
}

void imprime(const Vector &v){
    uint8_t dim = v.getDim();
    for(int i=0; i<dim;i++){
        fprintf(stderr, "%.4f ", v[i]);
    }
    fprintf(stderr, "\n\n");
}
void imprime(const Matriz &m){
    uint8_t dimV = m.getDimV();
    uint8_t dimH = m.getDimH();
    for(int i=0;i<dimV;i++){
        for(int j=0;j<dimH;j++){
            fprintf(stderr, "%.4f ", m(i,j));
        }
        fprintf(stderr, "\n");
    }
    fprintf(stderr, "\n");
}


