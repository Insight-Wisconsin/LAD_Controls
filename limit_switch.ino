// Motor Pins
const int ENApin = 6;
const int IN1pin = 7;
const int IN2pin = 8;

// Potentiometer and Limit Switch Pins
const int potPin      = A0;  // Potentiometer (arm angle)
const int limitTopPin = 2;   // Limit switch - arm fully raised
const int limitBotPin = 3;   // Limit switch - arm fully extended

// Angle and Speed Settings
const int MIN_ANGLE   = 100;  // Pot value for fully extended arm
const int MAX_ANGLE   = 900;  // Pot value for fully bent arm
const int MOTOR_SPEED = 150;  // Assist speed (0–255)
void setup() {
  pinMode(ENApin, OUTPUT);
  pinMode(IN1pin, OUTPUT);
  pinMode(IN2pin, OUTPUT);
  pinMode(limitTopPin, INPUT_PULLUP);
  pinMode(limitBotPin, INPUT_PULLUP);

  Serial.begin(9600);
}

void stopMotor() {
  digitalWrite(IN1pin, LOW);
  digitalWrite(IN2pin, LOW);
  analogWrite(ENApin, 0);
}

void motorUp() {
  digitalWrite(IN1pin, HIGH);
  digitalWrite(IN2pin, LOW);
  analogWrite(ENApin, MOTOR_SPEED);
}

void motorDown() {
  digitalWrite(IN1pin, LOW);
  digitalWrite(IN2pin, HIGH);
  analogWrite(ENApin, MOTOR_SPEED);
}

void loop() {
  int potValue = analogRead(potPin);


  // Hard stops
  if (digitalRead(limitTopPin) == LOW) {
    stopMotor();
    Serial.println("LIMIT: Top switch hit.");
    return;
  }
  if (digitalRead(limitBotPin) == LOW) {
    stopMotor();
    Serial.println("LIMIT: Bottom switch hit.");
    return;
  }

  // Soft stops based on pot angle
  if (potValue >= MAX_ANGLE) {
    stopMotor();
    Serial.println("Max angle reached — fully bent.");
  } else if (potValue <= MIN_ANGLE) {
    stopMotor();
    Serial.println("Min angle reached — fully extended.");
  } else {
    motorUp(); // Assist upward while within range
  }
}