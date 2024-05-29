#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include "FastIMU.h"

#define NUM_LEDS 14
#define DATA_PIN 10
#define SENSOR_PIN A1

#define IMU_ADDRESS 0x68
MPU6500 IMU;
calData calib = { 0 };  //Calibration data

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);

const int 
  SLIDER_PIN = 5,
  LED_PIN = 2,

  SERVO1_PIN = 6,
  BUTTON_LEFT = 3,
  BUTTON_RIGHT = 4,
  
  
  SERVO2_PIN = 7,
  SERVO4_PIN = 11,
  SERVO3_PIN = 9;

const int SERVO_SPEED = 2;
unsigned long last_time = 0;

typedef Servo BobServo;
BobServo servo1;
BobServo servo2;
BobServo servo3;
BobServo servo4;

struct ServoState {
  int left_ear_down = 90;
  int right_ear_down = 90;
  int left_ear_up = 90;
  int right_ear_up = 90;
};

void setup() {
  Serial.begin(115200);

  strip.begin();
  strip.show();
  strip.setBrightness(50);


  // servo 1
  // pinMode(BUTTON_LEFT, INPUT_PULLUP);
  // pinMode(BUTTON_RIGHT, INPUT_PULLUP);
  servo1.attach(SERVO1_PIN);
  servo1.write(90);

  // servo 2
  servo2.attach(SERVO2_PIN);


  // servo 3
  servo3.attach(SERVO3_PIN);
  servo3.write(90);

  // servo 4
  servo4.attach(SERVO4_PIN);
  servo4.write(90);



  // IMU stuff
  Wire.begin();
  Wire.setClock(400000); //400khz clock

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    while (true) {
      ;
    }
  }

  delay(1000);
  IMU.calibrateAccelGyro(&calib);
  delay(1000);
  IMU.init(calib, IMU_ADDRESS);

  last_time = millis();
}


ServoState prev;

unsigned long lastBeatTime = 0;
unsigned long lastInterval = 0;

float bpm;

void loop() {
  IMU.update();
  unsigned long now = millis();

  // button servo (servo 1)
  // bool left_button = digitalRead(BUTTON_LEFT) == LOW;
  // bool right_button = digitalRead(BUTTON_RIGHT) == LOW;
  // if (now - last_time > 15) {
  //   if (left_button) servo1_pos = max(0, servo1_pos - SERVO_SPEED);
  //   if (right_button) servo1_pos = min(180, servo1_pos + SERVO_SPEED);

  //   servo1.write(servo1_pos);
  //   last_time = now;
  // }

  // pot servo (servo 2)
  // int pot = analogRead(POT_PIN);
  // int servo_orientation = map(pot, 0, 1023, 0, 180);
  // servo2.write(servo_orientation);

  

  // Serial.println(heartSensor);
  int heartSensor = analogRead(SENSOR_PIN);
  unsigned long currentTime = millis();
  const int threshold = 800;
  if (heartSensor > threshold && currentTime - lastBeatTime > 250) {  // Debounce the signal (minimum time between beats)
    lastInterval = currentTime - lastBeatTime;
    lastBeatTime = currentTime;
    bpm = 60000.0 / lastInterval;  // Calculate BPM
    Serial.print("BPM: ");
    Serial.println(bpm);
  }

  /* heart rate + leds */
  const int heartMax = 150, heartMin = 90;
  int heartClipped = min(max(0, bpm - heartMin), heartMax - heartMin);
  int angerValue = map(heartClipped, 0, heartMax - heartMin, 0, 255);
  int colorValue = 255 - angerValue; //map(heartSensor, 0, 1023, 0, 255);
  uint32_t newColor = strip.Color(angerValue, colorValue, colorValue / 2);
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, newColor);
  }
  //strip.show();


  /* SERVO EARS PART DO NOT TOUCH */

  // gyroscope (servo 1)
  IMU.update();

  AccelData accel;
  IMU.getAccel(&accel);
  
  // 1 (anticlockwise (left))
  // -1 (clockwise (right))

  // trim value to range -1 - 1
  int yAccelVal = min(1., max(-1., accel.accelY)) * 100;
  int hrotation = map(yAccelVal, -100, 100, -90, 90);

  int xAccelVal = min(1., max(-1., accel.accelX)) * 100;
  int vrotation = map(xAccelVal, -100, 100, -90, 90);
  
  ServoState servos;
  const int ear_rot = 0;

  if (vrotation < -40) {
    servos.left_ear_down = 0;
    servos.right_ear_down = 180;

    servos.right_ear_up = ear_rot;
    servos.left_ear_up = 180 - ear_rot;
  } else {
    if (hrotation < -30) servos.right_ear_up = ear_rot; //map(rotation, -30, -60, 90, 0);
    else if (hrotation > 30) servos.left_ear_up = 180 - ear_rot;
  }

  unsigned int current = millis();
  if (bpm > 140 || current - last_time > 250) {
    strip.show();
    last_time = current;
  }

  if (memcmp(&servos, &prev, sizeof(ServoState)) != 0) {
    servo1.write(servos.left_ear_down);  //  0 - 180 (90 def)
    servo2.write(servos.left_ear_up);  // (180 - 120)-0?? (90 def)
    servo3.write(servos.right_ear_down);  // 0 - 180 (90 def)
    servo4.write(servos.right_ear_up);  // 0 - <130 (90 def)
    Serial.println("update state");
  }

  prev = servos;
}
