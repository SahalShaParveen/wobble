#include <Wire.h>

const int MPU = 0x68; 

const byte ENA = 5; 
const byte IN1 = 7;
const byte IN2 = 13;

const byte ENB = 6;
const byte IN3 = 4;
const byte IN4 = 12;

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

    float angle = atan2((float)AcZ, (float)AcX) * 180.0 / PI; 
    Serial.println(angle); 

    delay(200);
}      