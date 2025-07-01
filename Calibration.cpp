#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//#define ST_FLAG_NOBUILTIN
#include "SerialTerminal.hpp"
#include <avr/pgmspace.h>
//#include <avr/stdlib.h>
#include <stdarg.h>


maschinendeck::SerialTerminal *term;

// called this way, it uses the default address 0x40
TwoWire twi = TwoWire();
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_I2C_ADDRESS, twi);


#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define SERVOMIN 102 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 500 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOMIN_S "102" // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX_S "500" // this is the 'maximum' pulse length count (out of 4096)

#define SERVO_COUNT 16

uint8_t servonum = 0; // address of servo to be calibrated
int pos = ((SERVOMAX - SERVOMIN) / 2) + SERVOMIN;
bool power = false;
int powerDelay = 100;
int pos_open[SERVO_COUNT] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int pos_close[SERVO_COUNT] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};



// Printf-style function for F() strings
void printf_F(const __FlashStringHelper *fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    // Copy format string from PROGMEM to RAM
    strncpy_P(buf, (const char*)fmt, sizeof(buf));
    buf[sizeof(buf) - 1] = '\0';
    // Format into a second buffer
    char out[128];
    vsnprintf(out, sizeof(out), buf, args);
    va_end(args);
    Serial.print(out);
}

// Macro to allow printf_F(F(...)) to work as printf_F(F(...))
//#define printf_P(fmt, ...) printf_F(FPSTR(fmt), ##__VA_ARGS__)

void setPosition(int servo, int newpos)
{
  pos = min(max(newpos, SERVOMIN), SERVOMAX);
  pwm.setPin(servo, pos);
  printf_F(F("Setting servo %d to position %d\n"), servo, pos);

  if(!power){
    delay(powerDelay);
    pwm.setPWM(servo, 0, 4096);
    printf_F(F("Powered off after %dms\n"), powerDelay);
  }
}

void reset(const String &opts)
{
  setPosition(servonum, (SERVOMAX - SERVOMIN) / 2 + SERVOMIN);
}

void addValue(const String &opts)
{
  maschinendeck::Pair<String, String> operands = maschinendeck::SerialTerminal::ParseCommand(opts);
  setPosition(servonum, pos + static_cast<int>(operands.first().toInt()));
}

void subValue(const String &opts)
{
  maschinendeck::Pair<String, String> operands = maschinendeck::SerialTerminal::ParseCommand(opts);
  setPosition(servonum, pos - static_cast<int>(operands.first().toInt()));

}

void setValue(const String &opts)
{
  maschinendeck::Pair<String, String> operands = maschinendeck::SerialTerminal::ParseCommand(opts);
  setPosition(servonum, static_cast<int>(operands.first().toInt()));
}

void setPower(const String &opts)
{
  maschinendeck::Pair<String, String> operands = maschinendeck::SerialTerminal::ParseCommand(opts);
  power = static_cast<int>(operands.first().toInt());
  if(power){
    Serial.println(F("Power on"));
    
  } else {
    Serial.println(F("Power off"));
  }
}

void setPowerDelay(const String &opts)
{
  maschinendeck::Pair<String, String> operands = maschinendeck::SerialTerminal::ParseCommand(opts);
  powerDelay = static_cast<int>(operands.first().toInt());
  printf_F(F("Power delay set to %d\n"), powerDelay);

}

void setServo(const String &opts)
{
  printf_F(F("Servo calibration: %s\n"), opts.c_str());

  maschinendeck::Pair<String, String> operands = maschinendeck::SerialTerminal::ParseCommand(opts);
  int servo = static_cast<int>(operands.first().toInt());
  String openOrClose = operands.second();
  if(servo < 0 || servo > 15){
    servo = 15;
  }
  if(openOrClose == "o"){
    setPosition(servo, pos_open[servo]);
    printf_F(F("Servo set open position to %d\n"), pos_open[servo]);

  } else if(openOrClose == "c"){
    setPosition(servo, pos_close[servo]);
    printf_F(F("Servo set closed position to %d\n"), pos_close[servo]);
  } else {
    servonum = servo;
    printf_F(F("Servo set to %d\n"), servo);
  }
}

void setOpenPosition(const String &opts)
{
  if(opts.length() > 0){
    maschinendeck::Pair<String, String> operands = maschinendeck::SerialTerminal::ParseCommand(opts);
    pos_open[servonum] = static_cast<int>(operands.first().toInt());
    printf_F(F("Open position set to %d\n"), pos_open[servonum]);
  } else if (pos_open[servonum] > 0){
    setPosition(servonum, pos_open[servonum]);
    Serial.println(F("Open position set"));
  }
 
}


void setClosePosition(const String &opts)
{
  if(opts.length() > 0){
    maschinendeck::Pair<String, String> operands = maschinendeck::SerialTerminal::ParseCommand(opts);
    pos_close[servonum] = static_cast<int>(operands.first().toInt());
    printf_F(F("Close position set to %d\n"), pos_close[servonum]);
  } else if(pos_close[servonum] > 0){
    setPosition(servonum, pos_close[servonum]);
    Serial.println(F("Close position set"));
  }
 
}

void printPositions(const String &opts){
 
  printf_F(F("Servo position calibration\n"));
  printf_F(F("Servo: %d\n"),servonum);
  printf_F(F("Current position: %d\n"),pos);
  printf_F(F("Open position: %d\n"),pos_open[servonum]);
  printf_F(F("Close position: %d\n"),pos_close[servonum]);
  printf_F(F("Power: %d\n"),power);
  printf_F(F("Power delay: %d\n" ),powerDelay);

}

void setup()
{
  delay(1000);
  Serial.begin(115200);

  Serial.println(F("Servo calibration started"));
  term = new maschinendeck::SerialTerminal(115200);
  term->add("r", &reset, F("reset the servo driver"));
  term->add("s", &setServo, F("set servo port [0-15], or set open/closed [0-15] [o/c]"));
  term->add("+", &addValue, F("add to servo driver"));
  term->add("-", &subValue, F("sub from servo driver"));
  term->add("x", &setValue, F("set value to servo driver"));
  term->add("p", &setPower, F("set power [0/1] after movement"));
  term->add("d", &setPowerDelay, F("set power delay in ms"));
  term->add("o", &setOpenPosition, F("set open position"));
  term->add("c", &setClosePosition, F("set close position"));
  term->add("t", &printPositions, F("print positions"));
  
  pwm.begin();
  pwm.setOscillatorFrequency(FREQUENCY_OSCILLATOR);
  pwm.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates

  Serial.println(F("Servo calibration"));
  Serial.println(F("Remember to set initial servo position with \"x 200\" or similar"));
  
  Serial.print(F("Centre point: "));
  Serial.println(pos);
  delay(10);
}

void loop()
{
  term->loop();
  delay(2);
}

