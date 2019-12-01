#define dW digitalWrite
#define dR digitalRead

/* PIN DEFINITION */

#define LEDG  13

#define MOT1  2
#define MOT2  3
#define MOT3  4
#define MOT4  5

/* GLOBAL CONSTANTS */
float sens_scaled[5];
#define WHITE  1
#define BLACK   0

#define COLOR   BLACK         //Line color

#define N_SENS        5       //Number of sensors
#define R_SENS        1000    //Sensor readings are mapped to this range
#define WEIGHT_UNIT   1000    //Unit for weighted average
#define RIGHT_DIR     1
#define LEFT_DIR      0
#define PRESS         0
#define RELEASE       1
#define CAL_SPEED     100   //Sensor calibration speed
#define CAL_TIME      1000     //Calibration time
#define P_LINE_MIN    0.5     //Minimum brightness percentage to consider part of the line

const float SPEED = 600;
const float KP = .3;
const float KD =.6 ;
const float KI =.0001;

/* GLOBAL VARIABLES */

unsigned long ms = 0;
const int SENSOR[N_SENS] = { A0, A1, A2, A3, A4 };     //Arduino pins
int sens_max[N_SENS];          //Maximum value each sensor measures (calibrated)
int sens_min[N_SENS];          //Minimum value each sensor measures (calibrated)
int start = 0;
float line_pos = 0;
float last_line_pos = 0;

void setup() {
  Serial.begin(9600);
  InitializeGPIO();
  
  for(int x = 0; x < N_SENS; x++){
      sens_max[x] = 0;
      sens_min[x] = 1023;
  }
  
}

void loop() {

  delay(1000);
      
      calibrate(CAL_TIME, CAL_SPEED, RIGHT_DIR);
      calibrate(CAL_TIME, CAL_SPEED, LEFT_DIR);
      calibrate(CAL_TIME, CAL_SPEED, RIGHT_DIR);
      calibrate(CAL_TIME, CAL_SPEED, LEFT_DIR);
      delay(2000);
  
  
             while(1){ race();
           Serial.print(sens_scaled[0] );
Serial.print(' ');
Serial.print(sens_scaled[1]);
Serial.print(' ');
Serial.print(sens_scaled[2]);
Serial.print(' ');
Serial.print(sens_scaled[3]);
Serial.print(' ');
Serial.print(sens_scaled[4]);
Serial.println(' ');
delay(10);}
              
}

void race(void){
    last_line_pos = line_pos;
    line_pos = get_line_pos(COLOR, (last_line_pos>0));
        
    float PID_correction = get_PID_correction(line_pos, last_line_pos, KP, KD, KI);
    float max_correction = SPEED;                   //Can be changed to a lower value in order to limit the correction, needs to be at most SPEED
    
    if(PID_correction > 0){
        PID_correction = (PID_correction > max_correction)? max_correction : PID_correction;
        motorSpeed(SPEED, SPEED - PID_correction);
    }else{
        PID_correction = (PID_correction < -max_correction)? -max_correction : PID_correction;
        motorSpeed(SPEED + PID_correction, SPEED);
    }    
}


float get_line_pos(int color, int last_dir){
    float line = 0;
    int line_detected = 0;
    
    float avg_num = 0;          //Average numerator
    float avg_den = 0;          //Average denominator
    
    for(int x = 0; x < N_SENS; x++){
        //Scale from 0 to R_SENS
        sens_scaled[x] = analogRead(SENSOR[x]) - sens_min[x];
        sens_scaled[x] *= R_SENS;
        sens_scaled[x] /= (sens_max[x] - sens_min[x]);
        
        if(color ==BLACK){
            sens_scaled[x] = R_SENS - sens_scaled[x];     //Reverse scale to go from R_SENS to 0
        }
        if(sens_scaled[x] >= (float) R_SENS * ((float)P_LINE_MIN / 100.0)){   //At least one sensor has to detect a line
            line_detected = 1;
        }
        
        avg_num += sens_scaled[x] * x * WEIGHT_UNIT;
        avg_den += sens_scaled[x];        
    }
    if(line_detected == 1){
        line = avg_num / avg_den;                           //Weighted average
        line = line - (WEIGHT_UNIT * (N_SENS - 1) / 2);     //Change scale from 0 _ 4000 to -2000 _ 2000
        dW(LEDG, LOW);
    }else{
        line = WEIGHT_UNIT * (N_SENS - 1) * last_dir;       //Use last direction to calculate error as the maximum value
        line = line - (WEIGHT_UNIT * (N_SENS - 1) / 2);     //Change scale
        dW(LEDG, HIGH);
    }
    
    return line;
}

float get_PID_correction(float line, float last_line, float kp, float kd, float ki){
  float proportional = line;
  float derivative = line - last_line;
  float integral = line + last_line;
  float correction = ( proportional*KP + derivative*KD + KI* integral);

  return correction;
}

void motorSpeed(int m1, int m2) {           //From -1000 to 1000
  int pwm1 = map(abs(m1), 0, 1000, 0, 255);
  int pwm2 = map(abs(m2), 0, 1000, 0, 255);
  pwm1 = (m1>0) ? 255-pwm1 : pwm1;
  pwm2 = (m2>=0) ? pwm2 : 255-pwm2; 
  analogWrite(MOT2, pwm1);
  analogWrite(MOT4, pwm2);
  digitalWrite(MOT1, (m1 > 0) ? HIGH : LOW);
  digitalWrite(MOT3, (m2 >= 0) ? LOW : HIGH);
}

void InitializeGPIO(){
  
  pinMode(LEDG, OUTPUT);

  pinMode(MOT1, OUTPUT);
  pinMode(MOT2, OUTPUT);
  pinMode(MOT3, OUTPUT);
  pinMode(MOT4, OUTPUT);

  for(int x = 0; x <= N_SENS; x++){
    pinMode(SENSOR[x], INPUT);
  }
}
void calibrate(int cal_time, int cal_speed, int cal_dir){
  ms = millis();
  dW(LEDG, LOW);
  while((ms + cal_time) > millis()){
    dW(LEDG, millis()%100 < 50);        //Blink led
    if(cal_dir == RIGHT_DIR)  motorSpeed(cal_speed, -cal_speed);
    if(cal_dir == LEFT_DIR)  motorSpeed(-cal_speed, cal_speed);
    
    int sens_value[N_SENS];
    for(int x = 0; x < N_SENS; x++){
      sens_value[x] = analogRead(SENSOR[x]);
      sens_min[x] = (sens_value[x] < sens_min[x]) ? sens_value[x] : sens_min[x];
      sens_max[x] = (sens_value[x] > sens_max[x]) ? sens_value[x] : sens_max[x];
    }
  }
  motorSpeed(0, 0);
  dW(LEDG, HIGH);
}
