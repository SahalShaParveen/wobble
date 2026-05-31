#include <Wire.h>

const int MPU = 0x68; 

const byte ENA = 5; 
const byte IN1 = 7;
const byte IN2 = 13;

const byte ENB = 6;
const byte IN3 = 4;
const byte IN4 = 12;

const float GYRO_SCALE = 131.0; 
const float ALPHA = 0.98; 
const int LOOP_DELAY_MS = 20;  
const float DT  = LOOP_DELAY_MS /1000.0; 

float angle = 0; 
float gyBias = -0.62; 

void readAccel(int16_t &ax, int16_t &ay, int16_t &az) {
        Wire.beginTransmission(MPU); 
    Wire.write(0x3B); 
    Wire.endTransmission(false); 

    Wire.requestFrom(MPU, 6, true); 

    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();
}


void readGyro(int16_t &gx, int16_t &gy, int16_t &gz) {
    Wire.beginTransmission(MPU); 
    Wire.write(0x43); 
    Wire.endTransmission(false); 

    Wire.requestFrom(MPU, 6, true); 

    gx = Wire.read() << 8 | Wire.read();
    gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();
}

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
    int16_t GyX, GyY, GyZ; 

    readAccel(AcX, AcY, AcZ); 
    readGyro(GyX, GyY, GyZ); 

    float accelAngle = atan2((float)AcZ, (float)AcX) * 180.0 / PI; 
    float gyroRate = (GyY / GYRO_SCALE) - gyBias ; 
    angle = ALPHA * (angle + gyroRate * DT) + (1.0 - ALPHA) * accelAngle; 

    Serial.println(angle); 
    delay(LOOP_DELAY_MS); 
}