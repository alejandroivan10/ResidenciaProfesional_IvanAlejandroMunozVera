#ifndef ARDUINOFUNCTIONS_H
#define ARDUINOFUNCTIONS_H

class QString;

class _Serial{
public:
    void print(QString cadena);
    void print(float val);
    void println(QString cadena);
    void println(float val);
    void println(void);

private:

};

extern _Serial Serial;

QString String (float value);
QString String (int value);


#endif // ARDUINOFUNCTIONS_H
