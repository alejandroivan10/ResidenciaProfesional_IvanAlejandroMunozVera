#ifndef MATLAB_H
#define MATLAB_H

#include <cmath>

#define PI 3.1415926535f

class Vector{
public:
  Vector(void){
      dim = 0;
      vtr = nullptr;
  }
  Vector(const uint8_t _dim) : vtr(new float[_dim]){
      dim = _dim;
  }
  Vector(uint8_t _dim, float* _pointer) : vtr(_pointer){
      dim = _dim;
  }
  Vector(const Vector& v) : vtr(new float[v.getDim()]){
      dim = v.getDim();
      for(int i=0;i<dim;i++){
          vtr[i] = v[i];
      }
  }
//  Vector(const Vector&& v) : vtr(new float[v.getDim()]){
//      dim = v.getDim();
//      for(int i=0;i<dim;i++){
//          vtr[i] = v[i];
//      }
//  }
  Vector(std::initializer_list<float> l){
      dim = l.size();
      vtr = new float[dim];
      for(int i=0;i<dim;i++) vtr[i] = l.begin()[i];
  }
//  Vector(int _dim, ...) : vtr(new float[_dim]){
//      dim = _dim;
//      va_list param;
//      va_start(n, param);
//      for(int i=0;i<dim;i++){
//          vtr(i) = float(va_arg(param, double));
//      }
//  }
  ~Vector(){
      delete[] vtr;
      vtr = nullptr;
  }
  uint8_t getDim(void) const{
      return dim;
  }
  void resize(uint8_t _dim);

  float& operator()(uint8_t pointer) const{
      return vtr[pointer];
  }
  float& operator[](uint8_t pointer) const{
      return vtr[pointer];
  }
  Vector operator()(uint8_t p1, uint8_t p2) const;
  void operator=(Vector& v2);
  void operator=(Vector&& v2);
  float* vtr = nullptr;
protected:
  uint8_t dim = 0;
};

inline void Vector::resize(uint8_t _dim){
    if(dim > 0 && vtr != nullptr) delete[] vtr;
    dim = _dim;
    vtr = new float[_dim];
}

Vector operator*(const Vector& _v, float cte);
Vector operator*(const Vector& v1, const Vector& v2);
Vector operator*(float cte, const Vector& _v);
Vector operator/(const Vector& _v, float cte);
Vector operator/(float cte, const Vector& _v);
Vector operator/(const Vector& v1, const Vector& v2);
Vector operator+(const Vector& v1, const Vector& v2);
Vector operator-(const Vector& v1, const Vector& v2);
Vector operator-(float cte, const Vector& _v);
Vector operator<(const Vector& _v, float cte);
Vector operator<(float cte, const Vector& _v);
Vector operator&&(const Vector& v1, const Vector& v2);
Vector operator!(const Vector& _v);
float norm(const Vector& v);
float dot(const Vector& v1, const Vector& v2);
Vector quatconj(const Vector& _v);
Vector append(const Vector& v1, const Vector& v2);
Vector zeros(uint8_t n);
Vector ones(uint8_t dim);






class Matriz{
public:
    Matriz(uint8_t _dimV, uint8_t _dimH);
    Matriz(const Matriz& _m);
    Matriz(const Matriz&& _m);
    ~Matriz(){
        for(int i=0;i<dimV;i++){
            mtx[i].~Vector();
        }
        delete[] mtx;
        mtx = nullptr;
    }
    Vector* const getMatriz() const{              //CUIDADO
        return mtx;
    }
    float& operator()(uint16_t p1, uint8_t p2) const{   // /////////////  REVISAR QUE ESTO FUNCIONE:    CUIDADO
        return mtx[p1][p2];
    }
    Vector& operator[](uint16_t _idx) const;
    Vector& operator()(uint8_t _idx) const;
    uint8_t getDimV(void) const { return dimV;}
    uint8_t getDimH(void) const { return dimH;}

    void operator=(Matriz & m);
    void operator=(Matriz && m);

    Vector* mtx = nullptr;
protected:
    uint8_t dimV = 0;
    uint8_t dimH = 0;

};

class MatrizCuadrada : public Matriz{
public:
    MatrizCuadrada(uint8_t _dim) : Matriz(_dim, _dim){
    }
    MatrizCuadrada(const MatrizCuadrada& _m) : Matriz(_m){
    }
    MatrizCuadrada(const MatrizCuadrada&& _m) : Matriz(_m){
    }
    MatrizCuadrada(std::initializer_list<float> l) : Matriz(int(sqrt(l.size())), int(sqrt(l.size()))){
        uint8_t dim = dimV;
        if(dim*dim != l.size()){
            fprintf(stderr, "Constructor MatrizCuadrada Error Fatal: Initializer list no genera una matriz cuadrada \n");
            delete this;
            exit(5);
        }else{
            for(int i=0;i<dim;i++){
                for(int j=0;j<dim;j++){
                    mtx[i][j] = l.begin()[dim*i+j];
                }
            }
        }
    }
    ~MatrizCuadrada(){
    }
    uint8_t getDim() const{
        return dimV;
    }
    void operator=(const MatrizCuadrada & m);
    void operator=(const MatrizCuadrada && m);

    MatrizCuadrada t(void) const{
        MatrizCuadrada trans = MatrizCuadrada(dimV);
        for(int i=0;i<dimV;i++){
            for(int j=0;j<dimV;j++){
                trans[i][j] = mtx[j][i];
            }
        }
        return trans;
    }
};

MatrizCuadrada Inversa(const MatrizCuadrada& _mtx);
MatrizCuadrada zerosMC(uint8_t _dim);
Matriz ones(uint8_t dim1, uint8_t dim2);
MatrizCuadrada Quat2RotMat(const Vector& q);
Matriz operator-(const Matriz& _m, const Vector& _v);
Matriz operator/(const Matriz& _m, const Vector& _v);
Matriz pow(const Matriz& _m);

Vector operator*(const Matriz& _m, const Vector& _v);
Vector operator*(const Vector& _v, const Matriz& _m);
MatrizCuadrada operator*(const MatrizCuadrada& _m1, const MatrizCuadrada& _m2);
Vector quatmultiply(const Vector& q1, const Vector& q2);
Vector RegresiLinealMultip(const Matriz& _m);
Vector RotMat2Euler(const MatrizCuadrada& R);
MatrizCuadrada Euler2RotMat(const Vector& V);
Vector sqrt(const Vector& _v);
Vector sum(const Matriz& _m, uint8_t _idx);
float  sum(const Vector& _v);

void tic(void);
float toc(void);
void pause(uint8_t seconds);

void imprime(const Vector &v);
void imprime(const Matriz &m);


#endif // MATLAB_H
