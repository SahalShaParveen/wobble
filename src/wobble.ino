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
}

void loop() {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); 
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); 
    delay(3000); 

    digitalWrite(IN2, HIGH); digitalWrite(IN1, LOW); 
    digitalWrite(IN4, HIGH); digitalWrite(IN3, LOW); 
    delay(3000); 
}