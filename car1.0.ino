


#define PWML 9
#define PWMR 10
#define IN1L 4
#define IN2L 5
#define IN1R 6
#define IN2R 7

const byte spdctrlPin = 2;
const byte dirctrlPin = 3;
volatile int spd_val;
volatile int dir_val;

typedef struct motor {
  uint8_t in1;
  uint8_t in2;
  uint8_t pwm;
}motor_t;
  
void motor_init(motor_t *motor,uint8_t in1,uint8_t in2,uint8_t pwm)
{
  motor->in1=in1;
  motor->in2=in2;
  motor->pwm=pwm;
  pinMode(motor->in1,OUTPUT);
  pinMode(motor->in2,OUTPUT);
  pinMode(motor->pwm,OUTPUT);
  digitalWrite(motor->in1,LOW);
  digitalWrite(motor->in2,LOW);
  digitalWrite(motor->pwm,LOW);
}

void motor_dir(motor_t * motor,uint8_t rot_dir)
{
  digitalWrite(motor->in1,rot_dir);
  digitalWrite(motor->in2,!rot_dir); 
}
void motor_stop(motor_t *motor)
{
    digitalWrite(motor->in1,0);
    digitalWrite(motor->in2,0); 
}
void motor_speed(motor_t * motor,int pwm)
{
    analogWrite(motor->pwm,pwm);
}

  
motor_t motor_l,motor_r;


void motor_run(int spd,int dir)
{
  int pwml,pwmr;
  uint8_t dirl=LOW,dirr=LOW;
  spd=map(spd,1100,1900,-255,255);
  //x^2函数曲线控制方向（0-1）
  int delta = dir-1500;
  dir = (double)delta*abs(delta)/(400.0)+1500;   //二次函数
  //dir = (double)delta*delta*delta/(400.0*400.0)+1500;   //三次函数
//  Serial.println("修正后 的输出值");
//  Serial.println(dir);
  dir=map(dir,1100,1900,255,-255);
  
  pwml=constrain(spd+dir,-255,255);
  pwmr=constrain(spd-dir,-255,255);
//
//  Serial.print("pwml:");
//  Serial.println(spd);
//  Serial.print("pwmr:");
//  Serial.println(dir);

  if(pwml>0)
  {
    dirl=HIGH;
  }
  if(pwmr>0)
  {
    dirr=HIGH;
  }
  motor_speed(&motor_l,abs(pwml));
  motor_dir(&motor_l,dirl);

  motor_speed(&motor_r,abs(pwmr));
  motor_dir(&motor_r,dirr);
  }
void setup() {
  // put your setup code here, to run once:

  motor_init(&motor_l,IN1L,IN2L,PWML);         //init dir pin and speed pin
  motor_init(&motor_r,IN1R,IN2R,PWMR);
  
  motor_dir(&motor_l,0);                   //init dir
  motor_dir(&motor_r,0);

  motor_speed(&motor_l,0);
  motor_speed(&motor_r,0);
   //接收速度与方向信号
  pinMode(spdctrlPin, INPUT_PULLUP);
  pinMode(dirctrlPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(spdctrlPin), rc1, CHANGE);   
  attachInterrupt(digitalPinToInterrupt(dirctrlPin), rc2, CHANGE);   
  
  Serial.begin(115200);
}

void loop() {
  
  // put your main code here, to run repeatedly:
  //Serial.flush();
  
//  Serial.print("speed:");
//  Serial.println(spd_val);
//  Serial.print("direction:");
//  Serial.println(dir_val);
//  delay(100);

  motor_run(spd_val,dir_val);

}

unsigned long rc1_PulseStartTicks;
void rc1()
{
        // did the pin change to high or low?
        if (digitalRead( spdctrlPin ) == HIGH)
                rc1_PulseStartTicks = micros();    // store the current micros() value
        else
                spd_val = micros() - rc1_PulseStartTicks;
}
unsigned long rc2_PulseStartTicks;
void rc2()
{
        // did the pin change to high or low?
        if (digitalRead( dirctrlPin ) == HIGH)
                rc2_PulseStartTicks = micros();   
        else
                dir_val = micros() - rc2_PulseStartTicks;
}
