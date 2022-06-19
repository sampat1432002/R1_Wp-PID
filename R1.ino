#include "pid-controller.h"
#include "VescUart.h"
#include <Encoder.h>
#include <analogWrite.h>
#include <Ps3Controller.h>
#define direction 26
#define pwm 27

Encoder encoder(21, 33);
VescUart UART;
PID control(1, 10, 0.025);
double input = 0, setpoint = 2000, output = 0;
bool run = false;
long position[10] = {0, -6687, -3134, -7623, -9064, -6546, -7757, -3226, -5033, -2173};
uint16_t bldc_erpm[] = {0, 21000, 21000, 21000, 21000, 21000, 21000, 21000, 21000, 21000, 21000, 21000};

void anti_clockwise(uint8_t speed)
{
  digitalWrite(direction, LOW);
  analogWrite(pwm, speed);
}

void clockwise(uint8_t speed)
{
  digitalWrite(direction, HIGH);
  analogWrite(pwm, speed);
}

void brake() {
  analogWrite(pwm, 0);
}

bool running = true;
int8_t counter = 0;
uint8_t turret_limit = 39;

struct ps3_button {
  bool triangle, circle, cross, square;
} ps3;

void update() {
  if (Ps3.event.button_down.triangle) {
    ps3.triangle = true;
    running = true;
    counter = counter < 9 ? counter + 1 : 9;
  }
  if (Ps3.event.button_up.triangle) ps3.triangle = false;
  if (Ps3.event.button_down.circle) ps3.circle = true;
  if (Ps3.event.button_up.circle) ps3.circle = false;
  if (Ps3.event.button_down.cross) {
    ps3.cross = true;
    running = true;
    counter = counter > 0 ? counter - 1 : 0;
  }
  if (Ps3.event.button_up.cross) ps3.cross = false;
  if (Ps3.event.button_down.square) {
    ps3.square = true;
    running = false;
  }
  if (Ps3.event.button_up.square) ps3.square = false;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 13, 14);
  pinMode(direction, OUTPUT);
  pinMode(pwm, OUTPUT);
  pinMode(turret_limit, INPUT);
  Ps3.begin("11:11:11:11:11:11");
  Ps3.attach(update);
  UART.setSerialPort(&Serial1);
  control.set_period(0.02);
  control.set_output_limits(-120, 120);
  control.set_intergrator_range(-5, 5);
  control.set_tolerance(2);
  home();
  disableCore0WDT();
  xTaskCreatePinnedToCore(
    Task_BLDC
    ,  "Task_BLDC"   // A name just for humans
    ,  20000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL
    , 0);
  xTaskCreatePinnedToCore(
    Task_PID
    ,  "Task_PID"   // A name just for humans
    ,  20000  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL
    , 1);
}

void Task_BLDC(void *pvParameters) {
  vTaskDelay(100);
  for (;;)
    UART.setRPM(10000);
}

void Task_PID(void *pvParameters)
{
  for (;;)
    get_position(position[counter]);
}

void home() {
  running = true;
  while (!digitalRead(turret_limit) && running) {
    anti_clockwise(50);
  }
  brake();
  delay(200);
  encoder.write(0);
  Serial.print(" [*] Homing completed, position : ");
  Serial.println(encoder.read());
}

void get_position(double target) {
  control.set_setpoint(target);
  if (running) {
    Serial.println(counter);
    while (running) {
      if (target != position[counter])
        break;
      input = encoder.read();
      output = control.calculate(input, &run);
      if (control.at_setpoint())
        break;
      Serial.print(" [input] ");
      Serial.print(input);
      Serial.print(", [output] ");
      Serial.print(output);
      Serial.print(", [run] ");
      Serial.println(run);
      if (run) {
        if (output > 0)
          anti_clockwise(output);
        else if (output < 0)
          clockwise(abs(output));
      }
    }
    running = false;
    brake();
    Serial.println(" [*] setpoint attained.");
    if (!counter) {
      while (!digitalRead(turret_limit))
        anti_clockwise(50);
      brake();
      delay(200);
      encoder.write(0);
    }
  }
}

void loop() {
  if (running)
    get_position(position[counter]);
}
