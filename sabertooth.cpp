#include "sabertooth.h"

Motor::Motor(HardwareSerial *serial, byte address, byte channel)
{
  this->serial = serial; this->address = address; this->channel = channel;
}

void Motor::config(long baudrate)
{
  this->serial->begin(baudrate);
  this->serial->write(0xAA);
}

void Motor::config(long baudrate, byte rx, byte tx)
{
//  this->serial->begin(baudrate, SERIAL_8N1, rx, tx);
  this->serial->write(0xAA);
}

void Motor::command(byte address, byte command, byte value) {
  this->serial->write(address);
  this->serial->write(command);
  this->serial->write(value);
  this->serial->write((address + command + value) & B01111111);
}

void Motor::throttleCommand(byte address, byte command, int power) {
  power = constrain(power, -126, 126);
  this->command(address, command, (byte)abs(power));
}

void Motor::motor(byte address, byte motor, int power) {
  if (motor < 1 || motor > 2) {
    return;
  }
  this->throttleCommand(address, (motor == 2 ? 4 : 0) + (power < 0 ? 1 : 0), power);
}
void Motor::anti_clockwise(int speed) {
  speed = map(speed, 0, 255, 0, -127 );
  this->motor(this->address, this->channel, speed);
}

void Motor::clockwise(int speed) {
  speed = map(speed, 0, 255, 0, 127);
  this->motor(this->address, this->channel, speed);
}

void Motor::brake() {
  this->motor(this->address, this->channel, 0);
}
