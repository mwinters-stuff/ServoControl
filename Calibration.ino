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
#define SERVOMAX 350 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOMIN_S "102" // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX_S "350" // this is the 'maximum' pulse length count (out of 4096)

uint8_t servonum = 0; // address of servo to be calibrated
String readString;
int pos = ((SERVOMAX - SERVOMIN) / 2) + SERVOMIN;
bool power = false;
int powerDelay = 100;
uint8_t outputBuffer[5];

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
  setPosition(pos);
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
  
  pwm.begin();
  pwm.setOscillatorFrequency(FREQUENCY_OSCILLATOR);
  pwm.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates
  setPosition(pos);

  // Serial.println("Servo calibration");
  // Serial.println("Use this to calibrate your servo to find the range of movement required");
  // Serial.println("The servo should start close to the centre of the range");
  // Serial.println("Type \"+\" or \"-\" then a value to move the servo in that direction");
  // Serial.println("For example \"+ 10\" or \"- 20\"");
  // Serial.println("To move to a specific location use strings like \"x 200\" or \"x 210\" for new servo position");
  // Serial.println("Type \"reset\" to reset the servo to the centre point");
  // Serial.println("Move the servo to find the required range for whatever you're operating.");
  // Serial.println("Servos min and max can vary, try the " SERVOMIN_S " to " SERVOMAX_S " range to start with.");
  // Serial.println("WARNING: Exceeding the max range could damage the servo.");
  // Serial.println();
  
  Serial.println("Centre point:");
  Serial.println(pos);
  delay(10);
}

void loop()
{
  term->loop();
  delay(2);
}

