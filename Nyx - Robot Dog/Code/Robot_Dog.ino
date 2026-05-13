#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_MIN 150
#define SERVO_MAX 600
#define SERVO_FREQ 50

#define SDA_PIN 18
#define SCL_PIN 19

const float L1 = 100;
const float L2 = 133;


void setup() {
  

}

void loop() {
  solveIK(30, -60);
  delay(1000);

  solveIK(0, -80);
  delay(1000);

}

void solveIK(float x, float y){
  float D = sqrt(x*x + y * y);

  if (D > (L1 + L2) || D < abs(L1 - L2)) {
    return;
  }
float cosTheta2 = (x*x + y*y - L1*L1 - L2*L2) / (2*L1*L2);
float theta2 = acos(cosTheta2);

float alpha = atan2(y, x);
float beta = acos((x*x + y*y + L1*L1 - L2*L2)/ (2*L1*D));
float theta1 = alpha + beta;

float hip = theta1 * 180 / PI;
float knee = theta2 * 180 / PI;
}