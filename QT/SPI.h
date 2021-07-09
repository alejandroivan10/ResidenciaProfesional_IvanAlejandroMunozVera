#ifndef SPI_H
#define SPI_H

#define SPI_CLOCK_DIV2 2
#define SPI_CLOCK_DIV4 4
#define SPI_CLOCK_DIV8 8
#define SPI_CLOCK_DIV16 16
#define SPI_CLOCK_DIV32 32
#define SPI_CLOCK_DIV64 64

class _SPI{
public:
    static int setClockDivider(uint8_t Divider); ///Agregar la seleccion de canal por parametro
    static uint8_t transfer(uint8_t Data);

private:

};

extern _SPI SPI;

#endif // SPI_H
