#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

// configurable parameters
#define _DIST_ALPHA 0.5
// global variables
int a, b;
float alpha;
float dist_ema;


Servo myservo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  
// initialize serial port
  Serial.begin(57600);
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(1600);

  a = 69;
  b = 410;
  alpha = _DIST_ALPHA;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  dist_ema = alpha * dist_cali +((1 - alpha) * dist_ema);
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.print(dist_cali);
  Serial.print(",dist_ema:");
  Serial.println(dist_ema);
  
  if(dist_ema > 255) {
    digitalWrite(PIN_LED, 0);
    myservo.writeMicroseconds(2000);
    
    
  }
  else{
    digitalWrite(PIN_LED, 1);
    myservo.writeMicroseconds(1200);
    
  }
  delay(20);
}
