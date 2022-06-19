#ifndef SABERTOOTH_H
#define SABERTOOTH_H
#include <Arduino.h>
class Motor
{
  protected:
    byte address, channel;
    HardwareSerial *serial = NULL;
    void motor(byte address, byte motor, int power);  
    void throttleCommand(byte address,byte command, int power);
    void command(byte address,byte command, byte value);
  public:
    Motor(HardwareSerial *serial, byte address, byte channel);
    void config(long baudrate);
    void config(long baudrate, byte rx, byte tx);
    void anti_clockwise(int speed);
    void clockwise(int speed);
    void brake();
};
#endif
