#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 400 // maximum distance to be measured (unit: mm)
#define _DIST_ALPHA 0.5

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw, dist_prev, dist_ema, alpha; // unit: mm
unsigned long last_sampling_time; // unit: ms
int last_raw;
int i;
float scale; // used for pulse duration to distance conversion
Servo myservo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW); 
  pinMode(PIN_ECHO,INPUT);

  myservo.attach(PIN_SERVO); 
  myservo.write(90);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  alpha = _DIST_ALPHA;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = dist_prev = 0.0; // raw distance output from USS (unit: mm)
  last_raw = 0.0;
  i = 10;
  scale = 0.001 * 0.5 * SND_VEL;

// initialize serial port
  Serial.begin(57600);

// initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);
  dist_ema = alpha * dist_raw +((1 - alpha) * dist_ema);

// output the read value to the serial port
  Serial.print("Min:100,raw:");
  Serial.print(dist_raw);
  Serial.print(" ,ema:");
  Serial.print(dist_ema);
  Serial.print(",servo:");
  Serial.print(myservo.read());  
  Serial.println("Max:400");

// adjust servo position according to the USS read value

  if(dist_raw < 180){
    analogWrite(PIN_LED, 255);
    myservo.write(0);
    i = 10;
  }
  else if(dist_raw > 180 && dist_raw < 360){
    analogWrite(PIN_LED, 0);
    if(int(dist_raw/10) > last_raw){
      myservo.write(i);
      i += 10;
    }
    else if(int(dist_raw/10) == last_raw){
      myservo.write(i);
    }
    else if(last_raw > int(dist_raw/10)){
      i -= 10;
      myservo.write(i);
    }
  }
  else if(dist_raw > 360){
    analogWrite(PIN_LED, 255);
    myservo.write(180);
    i = 180;
  }

  last_raw = int(dist_raw/10);
// update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.

  if(reading == 0.0) reading = dist_prev;
  else dist_prev = reading;
  
  return reading;
}
