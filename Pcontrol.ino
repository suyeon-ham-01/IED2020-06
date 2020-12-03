#include <Servo.h>


/////////////////////////////
// Configurable parameters //
/////////////////////////////


// Arduino pin assignment
#define PIN_LED 9                            // [1234] LED를 아두이노의 GPIO 9번 핀에 연결
#define PIN_SERVO 10                //[3152] 서보모터를 아두이노의 10번 핀에 연결
#define PIN_IR A0                        //[3158] IR센서를 아두이노의 A0 핀에 연결


// Framework setting
#define _DIST_TARGET 255  //[3166]목표로 하는 탁구공 중심 위치까지 거리255mm로 고정
#define _DIST_MIN 10                       //[3164] 최소 측정 거리 10mm로 고정 
#define _DIST_MAX 410   // [3401] 측정 거리의 최댓값를 410mm로 설정


// Distance sensor
#define _DIST_ALPHA 0.35  // [3162] ema 필터의 alpha 값을 0.0으로 설정
#define LENGTH 30
#define k_LENGTH 8

// Servo range
#define _DUTY_MIN 1000                 //[3148]  서보의 가동 최소 각도(0)
#define _DUTY_NEU 1600                //[3150] servo neutral position (90 degree)
#define _DUTY_MAX 2000                // [3169] 서보의 최대 가동 각도(180º)


// Servo speed control
#define _SERVO_ANGLE 30        //[3159] 서보의 각도(30º) //[3150] 레일플레이트가 사용자가 원하는 가동범위를 움직일때, 이를 움직이게 하는 서보모터의 가동범위
#define _SERVO_SPEED 60            //[3147]  서보 속도를 30으로 설정


// Event periods
#define _INTERVAL_DIST 20         // [3153] Distance Sensing을 20(ms) 마다 실행한다.
#define _INTERVAL_SERVO 20 // [3401] 서보를 20ms마다 조작하기
#define _INTERVAL_SERIAL 100  // [3151] 시리얼 0.1초 마다 업데이트


// PID parameters
#define _KP 2.5                         // [3158] 비례상수 설정
#define _KD 85.0





//////////////////////
// global variables //
//////////////////////


// Servo instance
Servo myservo;        // [3153] Servo를 제어할 Object를 생성해 준다
// Distance sensor
float dist_target; // location to send the ball
float dist_raw;
int iter;
float dist_ema = 0.0;    //[3160] 실제 거리측정값과 ema필터를 적용한 값을 저장할 변수
float dist_list[LENGTH], sum, alpha;
int a, b;


// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; // [3162] 거리, 서보, 시리얼에 대한 마지막 샘플링 시간을 나타내는 변수
bool event_dist, event_servo, event_serial; // [5283] 거리센서, 서보, 시리얼 모니터의 발생을 지시하는 변수


// Servo speed control
int duty_chg_per_interval;    //[3167] 주기동안 duty 변화량 변수
int duty_target, duty_curr;    // [3157] 서보의 목표위치와 서보에 실제로 입력할 위치


// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm; // [3401] 비례 제어를 위한 전에 측정한 오차, 새로 측정한 오차 값, 비례항, 적분항, 미분항 변수



void setup() {
// initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED, OUTPUT);  // [3155] LED 핀 설정
  myservo.attach(PIN_SERVO);  // [3155] Servo 핀 설정

  myservo.writeMicroseconds(_DUTY_NEU);
// initialize global variables
  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0; // [3155] 샘플링 시각 기록 변수 초기화
  event_dist = event_servo = event_serial = false;  // [3155] 이벤트 bool값 초기화
  dist_target = _DIST_TARGET; // [3147] 목표지점 변수 초기화

  a = 69;
  b = 360;
  alpha = _DIST_ALPHA;
  
  

// move servo to neutral position
   // [3147] 서보를 중간으로 이동
// initialize serial port
    Serial.begin(57600);                          // [3169] 57600 보드레이트로 아두이노와 통신
// convert angle speed into duty change per interval.
  duty_chg_per_interval = int((_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * (float(_INTERVAL_SERVO) / 1000.0));   //[3159] 서보 각속도에 맞추어 센서 인식 간격 변경? [3150] 서보의 각속도를 원하는 Angle로 나누어 interval을 설정한다.
}
  


void loop() {
/////////////////////
// Event generator // [3155] 설정된 주기마다 이벤트 생성
/////////////////////
unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }




////////////////////
// Event handlers //
////////////////////


  if(event_dist) {
     event_dist = false;
  // get a distance reading from the distance sensor
      dist_raw = ir_distance_filter();

  // PID control logic
    error_curr = _DIST_TARGET - dist_raw; // [3158] 목표값 에서 현재값을 뺀 값이 오차값
    dterm = _KD * (error_curr - error_prev);
    pterm = _KP * error_curr; 
    control = pterm + dterm;


  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;


  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN) {
      duty_target = _DUTY_MIN;  // duty_target < _DUTY_MIN 일 때 duty_target 를 _DUTY_MIN 로 고정
   } else if (duty_target > _DUTY_MAX) {
       duty_target = _DUTY_MAX; // duty_target > _DUTY_MAX 일 때 duty_target 를 _DUTY_MAX 로 고정
    }  // [3153] [_DUTY_MIN, _DUTY_MAX] 로 서보의 가동범위를 고정하기 위한 최소한의 안전장치

    error_prev = error_curr;
  }
  
  if(event_servo) {
    event_servo = false; // [3153] servo EventHandler Ticket -> false
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }  
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }       //[3166] 서보가 현재위치에서 목표위치에 도달할 때까지 duty_chg_per_interval값 마다 움직임(duty_curr에 duty_chg_per_interval값 더하고 빼줌)
    
    // update servo position
    myservo.writeMicroseconds(duty_curr);   //[3166]위에서 바뀐 현재위치 값을 갱신
  }
  
  if(event_serial) {
    event_serial = false; // [3153] serial EventHandler Ticket -> false
    Serial.print("filtered_dist:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",dterm:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,900,2000,400,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.print(",duty_chg_per_interval:");
    Serial.print(duty_chg_per_interval);
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
}


float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;                                        //[3150] 적외선 센서를 통해 거리를 return시킨다.
}

float ir_distance_filter(void){
  sum = 0;
  iter = 0;
  while(iter < LENGTH){
    dist_list[iter] = 100 + 300.0 / (b - a) * (ir_distance() - a);
    sum += dist_list[iter];
    iter++;
  }
  
  for (int i = 0; i < LENGTH - 1; i++) {
    for (int j = i+1; j < LENGTH; j++){
      if(dist_list[i] > dist_list[j]){
        float tmp = dist_list[i];
        dist_list[i] = dist_list[j];
        dist_list[j] = tmp;
      }
    } 
  }

  for(int i=0; i<k_LENGTH; i++){
    sum -= dist_list[i];
  }
  for(int i = 1; i <= k_LENGTH; i++){
    sum = dist_list[LENGTH-i];
  }

  float dist_cali = sum / (LENGTH - 2 * k_LENGTH);
  dist_ema = alpha * dist_cali + (1 - alpha)*dist_ema;
  return dist_ema * 10;
}
