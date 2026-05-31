#include <Wire.h>

const int MPU = 0x68; 

const byte ENA = 5; 
const byte IN1 = 7;
const byte IN2 = 13;

const byte ENB = 6;
const byte IN3 = 4;
const byte IN4 = 12;

const int delayTime = 20; 
float gyroAngle = 0; 
float angle = 0; 

float gyBias = -0.62; 

void setup(){
    pinMode(ENA, OUTPUT); 
    pinMode(IN1, OUTPUT); 
    pinMode(IN2, OUTPUT); 

    pinMode(ENB, OUTPUT); 
    pinMode(IN3, OUTPUT); 
    pinMode(IN4, OUTPUT);

    analogWrite(ENA, 255); 
    analogWrite(ENB, 255); 

    Serial.begin(9600); 
    Wire.begin(); 

    Wire.beginTransmission(MPU); 
    Wire.write(0x6B); 
    Wire.write(0); 
    Wire.endTransmission(true); 
    Serial.println("MPU awake");
}

void loop() {
    int16_t AcX, AcY, AcZ; 

    Wire.beginTransmission(MPU); 
    Wire.write(0x3B); 
    Wire.endTransmission(false); 

    Wire.requestFrom(MPU, 6, true); 

    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();

    float accelAngle = atan2((float)AcZ, (float)AcX) * 180.0 / PI; 

    int16_t GyX, GyY, GyZ; 
    Wire.beginTransmission(MPU); 
    Wire.write(0x43); 
    Wire.endTransmission(false); 

    Wire.requestFrom(MPU, 6, true); 

    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();


    float gyroRate = (GyY / 131.0) - gyBias ; 
    float dt = delayTime/1000.0; 
    gyroAngle += gyroRate * dt;

    angle = 0.98 * (gyroAngle) + 0.02 * accelAngle;
    Serial.println(angle); 

    delay(delayTime);
}