#include <mbed.h>
#include "accel.h"

SPI spi(SPI_MOSI,SPI_MISO,SPI_SCK);
DigitalOut cs(PE_3);
int G_COEFF = 2;

int16_t getVelocity(int *buf[], int length){
    int sum = 0, vel;
    for(int i=0;i<length;i++){
        sum += *buf[i];
    }
    vel = sum / length;
    return vel;
}

double getMag(int x, int y, int z){
    double sq = x*x + y*y + z*z;
    return G_COEFF*sqrt(sq) / (32767);
}

double getAxisMag(int a, int b, int gain){
    double sq = a*a + b*b;
    return gain*G_COEFF*sqrt(sq) / (32767);
}

double getXangle(int y, int z){
    return atan2(y, z) * 180/PI;
}

double getYangle(int x,int y, int z){
    return atan2(-x, sqrt(y*y + z*z)) * 180/PI;
}

void writeReg(uint8_t addr, uint8_t data){
      cs=0;
      spi.write(WRITE | addr);
      spi.write(data);
      cs=1;
      
      }

uint8_t readReg(uint8_t addr){
      uint8_t data;
      cs=0;
      spi.write(READ | addr);
      data = spi.write(0x00);
      cs=1;
      return data;
      }

void readData(int16_t *X, int16_t *Y, int16_t *Z){
    uint8_t xLSB=0, xMSB, yLSB, yMSB, zLSB, zMSB;       // 8-bit values from accelerometer
    
    xMSB = readReg(OUT_X_H);                    // read X high byte register
    xLSB = readReg(OUT_X_L);                     // read X low byte register
    yMSB = readReg(OUT_Y_H);
    yLSB = readReg(OUT_Y_L);
    zMSB = readReg(OUT_Z_H);
    zLSB = readReg(OUT_Z_L);
        
    //pack MSB and LSB bytes for X, Y, and Z
    *X = (xMSB << 8) | (xLSB);   
    *Y = (yMSB << 8) | (yLSB);
    *Z = (zMSB << 8) | (zLSB);
}

void accel_set4G(){
    writeReg(CTRL_REG5,0x08);
    G_COEFF = 4;
}

void accel_set2G(){
    writeReg(CTRL_REG5,0x00);
    G_COEFF = 2;
}

void mems_startup(){

    cs=1;
    spi.format(8,3);
    spi.frequency(1000000);
    writeReg(CTRL_REG4, 0x77);             // Normal power mode, all axes enabled, 50 Hz ODR
    writeReg(CTRL_REG5, 0x80);             // 200 Hz antialias filter, +/- 2g FS range   
    writeReg(FIFO_CTRL_REG, 0);            // configure FIFO for bypass mode   
    writeReg(CTRL_REG6, 0x10);             // disable FIFO, enable register address auto-increment
    writeReg(CTRL_REG4, 0x00);
    writeReg(CTRL_REG4, 0x37);
}