#include <Wire.h>

const int MPU = 0x68; 

const byte ENA = 13; 
const byte IN1 = 12;
const byte IN2 = 14;

const byte ENB = 25;
const byte IN3 = 26;
const byte IN4 = 27;

const float GYRO_SCALE = 131.0; 
const float ALPHA = 0.98; 
const int LOOP_DELAY_MS = 20;  
const float DT  = LOOP_DELAY_MS /1000.0; 

float angle = 0; 
float gyBias = -0.62; 

float BALANCE_ANGLE = 109; 

int Kp = 7; 
float Ki = 0.2;

float integral = 0;
const float INTEGRAL_LIMIT = 100;

float Kd = 0.5;
float lastError = 0;

String inputString = "";
bool stringComplete = false;

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


void setMotorSpeed(int speed) {
    const int MIN_PWM = 100; 
    speed = constrain(speed, -255, 255);
    int pwm = 0; 
    
    if (speed != 0)
    {
        pwm = map(abs(speed), 0, 255, MIN_PWM, 255);
    }

    analogWrite(ENA, pwm);
    analogWrite(ENB, pwm);

    if (speed < 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    }
    else if (speed > 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    }
    else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
    }
}


void handleSerialCommand(String cmd) {
    cmd.trim();
    if (cmd.length() < 2) return;

    char type = cmd.charAt(0);
    float value = cmd.substring(1).toFloat();

    switch (type) {
        case 'P':
            Kp = value;
            Serial.print("Kp set to "); Serial.println(Kp);
            break;

        case 'I':
            Ki = value;
            Serial.print("Ki set to "); Serial.println(Ki);
            break;

        case 'D':
            Kd = value;
            Serial.print("Kd set to "); Serial.println(Kd);
            break;

        case 'A':
            BALANCE_ANGLE = value;
            Serial.print("Balance angle set to "); Serial.println(BALANCE_ANGLE);
            break;

        default:
            Serial.println("Unknown command");
            break;
    }
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

    Serial.begin(115200); 
    Wire.begin(); 

    Wire.beginTransmission(MPU); 
    Wire.write(0x6B); 
    Wire.write(0); 
    Wire.endTransmission(true); 
    Serial.println("MPU awake");
}


void loop() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();

        if (inChar == '\n' || inChar == '\r') {
            if (inputString.length() > 0) {
                stringComplete = true;
            }
        } else {
            inputString += inChar;
        }
    }

    if (stringComplete) {
        handleSerialCommand(inputString);
        inputString = "";
        stringComplete = false;
    }

    int16_t AcX, AcY, AcZ; 
    int16_t GyX, GyY, GyZ; 

    readAccel(AcX, AcY, AcZ); 
    readGyro(GyX, GyY, GyZ); 

    float accelAngle = atan2((float)AcZ, (float)AcX) * 180.0 / PI; 
    float gyroRate = (GyY / GYRO_SCALE) - gyBias ; 
    angle = ALPHA * (angle + gyroRate * DT) + (1.0 - ALPHA) * accelAngle; 

    float error = angle - BALANCE_ANGLE; 

    Serial.print(angle); Serial.print("  ["); Serial.print(BALANCE_ANGLE); Serial.print("]  "); Serial.print("("); Serial.print(error);Serial.print(")"); 
    Serial.print(" | "); 
    Serial.print(Kp); Serial.print(", "); Serial.print(Kd); Serial.print(", "); Serial.print(Ki); 
    Serial.println(0);  

    integral += error * DT;
    integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    float derivative = (error - lastError) / DT;
    lastError = error;

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    setMotorSpeed((int) output);   

    delay(LOOP_DELAY_MS); 
}
