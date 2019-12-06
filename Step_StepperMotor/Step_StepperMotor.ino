
#include "freertos/task.h"

/*****加速センサ**********/
#include "MPU9250_asukiaaa.h"
#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 25
#define SCL_PIN 26
#endif

#define CALIB_SEC 5
MPU9250 mySensor;

uint8_t sensorId;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;

/********ステッピングモータ***********/
int led = 2;
int motorPulsePin = 15, motorDirectionPin = 0;//ドライバにより異なる．バルスと方向ピン
int16_t currentPulse = 0, targetPulse = 0;
int microsStepping = 1;//ドライバのマイクロ・ステッピング，
long nowTime = 0, lastTime = 0;
int loopTimer = 20;//20ms,モータコントロール更新ループとデータ送受信ループ
int Velocity = 0, Velocity0 = 0, VelocityMax = 10;//VelocityMax = 20/loopTimer = 20pulse/20ms
int Acceleration = 1;//8pulse/loopTimer/loopTimer = 8pulse/20ms/20ms

/**********加速度センサとデータ送受信ループ************/
void ReadSensorValues(void* arg) {
  while(1){
    nowTime = millis();
    if(nowTime-lastTime >= loopTimer)
    {
      Serial.write(255);
      if(Serial.available() >=2){//2bytes来たら  
        targetPulse = (Serial.read() + (Serial.read() << 8));//short型
        digitalWrite(led, !digitalRead(led));
      }
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  pinMode(motorPulsePin, OUTPUT);
  digitalWrite(motorPulsePin, 0);
  pinMode(motorDirectionPin, OUTPUT);
  digitalWrite(motorDirectionPin, 0);

  xTaskCreatePinnedToCore(ReadSensorValues, "readSensor", 4096, NULL, 10, NULL, 0);//Core 0タスク優先順位は 1～25 の範囲です。大きい方が優先順位が高まります
  //mainはCore0?
}

/************モータコントロールループ*****************/
int lastStepPulse = 0;
void loop() {
  int stepPulse = targetPulse - currentPulse;
   
  int direct = 1;
  if(stepPulse < 0){
    direct = 0;
  }
  if(lastStepPulse * stepPulse < 0){//逆方向
    Velocity0 = 0;
  }
  lastStepPulse = stepPulse;
  
  Velocity = Acceleration + Velocity0;//V = a*t + V0
  if(Velocity > VelocityMax)
    Velocity = VelocityMax;

  if(stepPulse > Velocity)
    stepPulse = Velocity;
  else if(stepPulse < -Velocity)
    stepPulse = -Velocity;

  Velocity0 = abs(stepPulse);
  
  digitalWrite(motorDirectionPin, direct);
  for(int i = 0; i < abs(stepPulse) * microsStepping; i++){
    long T = (loopTimer * 1000 / microsStepping / abs(stepPulse));//micro second
    digitalWrite(motorPulsePin, 1);
    delayMicroseconds(T/2);
    digitalWrite(motorPulsePin, 0);
    delayMicroseconds(T/2); 
  }
  currentPulse += stepPulse; 
}
