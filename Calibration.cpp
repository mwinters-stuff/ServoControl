#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define ST_FLAG_NOBUILTIN
#include <SerialTerminal.hpp>

maschinendeck::SerialTerminal *term;

// called this way, it uses the default address 0x40
TwoWire twi = TwoWire();
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_I2C_ADDRESS, twi);

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define SERVOMIN 102 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 500 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOMIN_S "102" // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX_S "500" // this is the 'maximum' pulse length count (out of 4096)

uint8_t servonum = 0; // address of servo to be calibrated
int pos = ((SERVOMAX - SERVOMIN) / 2) + SERVOMIN;
bool power = false;
int powerDelay = 100;
int pos_open = 0;
int pos_close = 0;

void setPosition(int newpos)
{
  pos = min(max(newpos, SERVOMIN), SERVOMAX);
  pwm.setPin(servonum, pos);
  Serial.println("now at " + String(pos));
  if(!power){
    delay(powerDelay);
    pwm.setPWM(servonum, 0, 4096);

    Serial.println("Powered off after " + String(powerDelay) + "ms");
  }
}

void reset(String opts)
{
  setPosition((SERVOMAX - SERVOMIN) / 2 + SERVOMIN);
}

void addValue(String opts)
{
  maschinendeck::Pair<String, String> operands = maschinendeck::SerialTerminal::ParseCommand(opts);
  setPosition(pos + operands.first().toInt());
}

void subValue(String opts)
{
  maschinendeck::Pair<String, String> operands = maschinendeck::SerialTerminal::ParseCommand(opts);
  setPosition(pos - operands.first().toInt());

}

void setValue(String opts)
{
  maschinendeck::Pair<String, String> operands = maschinendeck::SerialTerminal::ParseCommand(opts);
  setPosition(operands.first().toInt());
}

void setPower(String opts)
{
  maschinendeck::Pair<String, String> operands = maschinendeck::SerialTerminal::ParseCommand(opts);
  power = operands.first().toInt();
  if(power){
    Serial.println("Power on");
    
  } else {
    Serial.println("Power off");
  }
}

void setPowerDelay(String opts)
{
  maschinendeck::Pair<String, String> operands = maschinendeck::SerialTerminal::ParseCommand(opts);
  powerDelay = operands.first().toInt();
  Serial.println("Power delay set to " + String(powerDelay));
 
}

void setServo(String opts)
{
  maschinendeck::Pair<String, String> operands = maschinendeck::SerialTerminal::ParseCommand(opts);
  servonum = operands.first().toInt();
  if(servonum > 15){
    servonum = 15;
  }
  Serial.println("Servo set to " + String(servonum));
}

void setOpenPosition(String opts)
{
  if(opts.length() > 0){
    maschinendeck::Pair<String, String> operands = maschinendeck::SerialTerminal::ParseCommand(opts);
    pos_open = operands.first().toInt();
    Serial.println("Open position set to " + String(pos_open));
  } else if (pos_open > 0){
    setPosition(pos_open);
    Serial.println("Open position set");
  }
 
}


void setClosePosition(String opts)
{
  if(opts.length() > 0){
    maschinendeck::Pair<String, String> operands = maschinendeck::SerialTerminal::ParseCommand(opts);
    pos_close = operands.first().toInt();
    Serial.println("Close position set to " + String(pos_close));
  } else if(pos_close > 0){
    setPosition(pos_close);
    Serial.println("Close position set");
  }
 
}

void printPositions(String opts){
  Serial.println("Servo position calibration");
  Serial.println("Servo: " + String(servonum));
  Serial.println("Current position: " + String(pos));
  Serial.println("Open position: " + String(pos_open));
  Serial.println("Close position: " + String(pos_close));
  Serial.println("Power: " + String(power));
  Serial.println("Power delay: " + String(powerDelay));

}

void setup()
{
  // Serial.begin(115200);
  term = new maschinendeck::SerialTerminal(115200);
  term->add("r", &reset, "reset the servo driver");
  term->add("s", &setServo, "set servo port [0-15]");
  term->add("+", &addValue, "add to servo driver");
  term->add("-", &subValue, "sub from servo driver");
  term->add("x", &setValue, "set value to servo driver");
  term->add("p", &setPower, "set power [0/1] after movement");
  term->add("d", &setPowerDelay, "set power delay in ms");
  term->add("o", &setOpenPosition, "set open position");
  term->add("c", &setClosePosition, "set close position");
  term->add("p", &printPositions, "print positions");
  
  pwm.begin();
  pwm.setOscillatorFrequency(FREQUENCY_OSCILLATOR);
  pwm.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates

  Serial.println("Servo calibration");
  Serial.println("Remember to set initial servo position with \"x 200\" or similar");
  
  Serial.println("Centre point:");
  Serial.println(pos);
  delay(10);
}

void loop()
{
  term->loop();
  delay(2);
}
