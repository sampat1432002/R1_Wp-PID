#include "pid-controller.h"

PID::PID(double kp, double ki, double kd) {
  this->kp = kp; this->ki = ki; this->kd = kd;
}

PID::PID(double kp, double ki, double kd, double period) {
  this->kp = kp; this->ki = ki; this->kd = kd; this->period = period;
}

double PID::clamp(double value, double low, double high) {
  return value < low ? low : (value > high ? high : value);
}

double PID::calculate(double input, double setpoint, bool *run) {
  this->setpoint = setpoint;
  return this->calculate(input, run);
}

double PID::calculate(double input, bool *run) {
  if (micros() - prev_time < (this->period * uSec)) {
    *run = false;
    return this->output;
  }
  *run = true;
  this->input = input;
  this->prev_error = this->position_error;
  this->position_error = this->setpoint - this->input;
  this->velocity_error = (this->position_error - this->prev_error) / this->period;
  if (this->ki) {
    this->total_error = this->clamp(
                          this->total_error + this->position_error * this->period,
                          this->min_intergral / this->ki,
                          this->max_intergral / this->ki
                        );
  }
  this->output = this->kp * this->position_error + this->ki * this->total_error + this->kd * this->velocity_error;
  this->output = this->clamp(this->output, this->min_limit, this->max_limit);
  this->prev_time = micros();
  return this->output;
}

void PID::set_tunings(double kp, double ki, double kd) {
  this->kp = kp; this->ki = ki; this->kd = kd;
}

void PID::set_kp(double kp) {
  this->kp = kp;
}

void PID::set_ki(double ki) {
  this->ki = ki;
}

void PID::set_kd(double kd) {
  this->kd = kd;
}

void PID::set_period(double period) {
  this->period = period;
}

void PID::set_setpoint(double setpoint) {
  this->setpoint = setpoint;
  this->position_error = this->setpoint - this->input;
  this->velocity_error = (this->position_error - this->input) / this->period;
}

void PID::set_intergrator_range(double min_intergral, double max_intergral) {
  this->min_intergral = min_intergral; this->max_intergral = max_intergral;
}

void PID::set_tolerance(double position_tolerance) {
  this->set_tolerance(position_tolerance, DBL_MAX);
}

void PID::set_tolerance(double position_tolerance, double velocity_tolerance) {
  this->position_tolerance = position_tolerance; this->velocity_tolerance = velocity_tolerance;
}

void PID::set_output_limits(double min_limit, double max_limit) {
  this->min_limit = min_limit; this->max_limit = max_limit;
}

bool PID::at_setpoint() {
  return abs(this->position_error) < this->position_tolerance && abs(this->velocity_error) < this->velocity_tolerance;
}

double PID::get_kp() {
  return this->kp;
}

double PID::get_ki() {
  return this->ki;
}

double PID::get_kd() {
  return this->kd;
}

double PID::get_period() {
  return this->period;
}

double PID::get_setpoint() {
  return this->setpoint;
}

double PID::get_output() {
  return this->output;
}

double PID::get_position_error() {
  return this->position_error;
}

double PID::get_velocity_error() {
  return this->velocity_error;
}

void PID::reset() {
  this->position_error = this->velocity_error = this->total_error = this->prev_error = 0;
}
