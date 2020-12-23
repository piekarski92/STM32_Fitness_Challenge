#ifndef ACCEL_H_INCLUDED
#define ACCEL_H_INCLUDED

#define INFO1                       0x0D
#define INFO2                       0x0E
#define WHO_AM_I                    0x0F
#define OFF_X                       0x10
#define OFF_Y                       0x11
#define OFF_Z                       0x12
#define CTRL_REG4                   0x20
#define CTRL_REG1                   0x21
#define CTRL_REG2                   0x22
#define CTRL_REG3                   0x23
#define CTRL_REG5                   0x24
#define CTRL_REG6                   0x25
#define OUT_X_L                     0x28
#define OUT_X_H                     0x29
#define OUT_Y_L                     0x2A
#define OUT_Y_H                     0x2B
#define OUT_Z_L                     0x2C
#define OUT_Z_H                     0x2D
#define FIFO_CTRL_REG               0x2E
#define READ                        0x80
#define WRITE                       0x00

const double PI = 3.1459;

void mems_startup();
void readData(int16_t *X, int16_t *Y, int16_t *Z);
void accel_set2G();
void accel_set4G();

double getMag(int x, int y, int z);
double getAxisMag(int a, int b, int gain);
double getXangle(int y, int z);
double getYangle(int x,int y, int z);

#endif 