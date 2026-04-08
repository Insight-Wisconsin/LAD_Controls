int ENApin = 6;
int IN1pin = 7;
int IN2pin = 8;
 
float speedMultiplier = 0.5;
 
int pressurePin1 = A0;  // Pressure pad 1 (forward)
int pressurePin2 = A1;  // Pressure pad 2 (reverse)
const int potPin = A2;       // Analog pin on the Uno
const int windowSize = 100;   // number of values in averaging filter
int values[windowSize];      // Array to store readings
int readIndex = 0;           // Current index in the array
long total = 0;              // Running total
int avg_emg = 0;             // Moving average
int threshold;               // tbd
int threshold2;              // tbd
int threshold3;              // tbd

bool armUp = false;          // 
 
int pressureValue1 = 0;
int pressureValue2 = 0;
 
void setup() {
  pinMode(ENApin, OUTPUT);
  pinMode(IN1pin, OUTPUT);
  pinMode(IN2pin, OUTPUT);
  Serial.begin(9600);
 
  // Initialize the smoothing array with 0s
  for (int i = 0; i < windowSize; i++) {
    values[i] = 0;
  }
 
}
 
void loop() {
  pressureValue1 = analogRead(pressurePin1);
  pressureValue2 = analogRead(pressurePin2);
 
  Serial.print("Pressure 1 (Forward): ");
  Serial.print(pressureValue1);
  Serial.print("  |  Pressure 2 (Reverse): ");
  Serial.println(pressureValue2);
 
  int speed = 0;
  // EMG CONTROL
  // Subtract the oldest reading, add the new one, and advance the index
  total = total - values[readIndex];      
  values[readIndex] = analogRead(potPin);
  total = total + values[readIndex];      
  readIndex = (readIndex + 1) % windowSize;
 
  avg_emg = total / windowSize;  
 
  Serial.println(avg_emg);
 
  // PRESSURE PAD CONTROL
  // Determine base speed based on the *greater* pressure
  if (pressureValue1 > 750 || pressureValue2 > 750) {
    speed = 255;
  } else if (pressureValue1 > 500 || pressureValue2 > 500) {
    speed = 170;
  } else if (pressureValue1 > 250 || pressureValue2 > 250) {
    speed = 85;
  } else {
    speed = 0;
  }
 
  speed *= speedMultiplier;
 
  if (pressureValue1 > 10 && pressureValue1 > pressureValue2) {
    // Move Forward
    digitalWrite(IN1pin, HIGH);
    digitalWrite(IN2pin, LOW);
    analogWrite(ENApin, speed);
  } else if (pressureValue2 > 10 && pressureValue2 > pressureValue1) {
    // Move Reverse
    digitalWrite(IN1pin, LOW);
    digitalWrite(IN2pin, HIGH);
    analogWrite(ENApin, speed);
  } else {
    // No valid input — stop motor
    digitalWrite(IN1pin, LOW);
    digitalWrite(IN2pin, LOW);
    analogWrite(ENApin, 0);
  }
 

  int speed_emg = 0;
  if (armUp == false) // arm is not activated yet 
  {
    if (avg_emg > threshold) // muscle basically fully flexed
    {
      speed_emg = 255;
      speed_emg *= speedMultiplier;
      digitalWrite(IN1pin, HIGH);
      digitalWrite(IN2pin, LOW);
      analogWrite(ENApin, speed);
    }
    else if (avg_emg > threshold2) // muscle moderatley flexing
    {
      speed_emg = 170;
      speed_emg *= speedMultiplier;
      digitalWrite(IN1pin, HIGH);
      digitalWrite(IN2pin, LOW);
      analogWrite(ENApin, speed);
    } 
    else if (avg_emg > threshold3) // muscle slightly flexing
    {
      speed_emg = 85;
      speed_emg *= speedMultiplier;
      digitalWrite(IN1pin, HIGH);
      digitalWrite(IN2pin, LOW);
      analogWrite(ENApin, speed);
    } 
    else // muscle not flexing
    {
      speed_emg = 0;
      digitalWrite(IN1pin, LOW);
      digitalWrite(IN2pin, LOW);
      analogWrite(ENApin, 0);
    }
  }
  else //arm is activated already
  {
    if (avg_emg < threshold3) // muscles basically fully unflexed
    {
      speed_emg = 255;
      speed_emg *= speedMultiplier;
      digitalWrite(IN1pin, LOW);
      digitalWrite(IN2pin, HIGH);
      analogWrite(ENApin, speed);
    }
    else if (avg_emg < threshold2) // muscle moderatley unflexed
    {
      speed_emg = 170;
      speed_emg *= speedMultiplier;
      digitalWrite(IN1pin, LOW);
      digitalWrite(IN2pin, HIGH);
      analogWrite(ENApin, speed);
    } 
    else if (avg_emg < threshold) // muscle slightly unflexed
    {
      speed_emg = 85;
      speed_emg *= speedMultiplier;
      digitalWrite(IN1pin, LOW);
      digitalWrite(IN2pin, HIGH);
      analogWrite(ENApin, speed);
    } 
    else // muscle still flexing
    {
      speed_emg = 0;
      digitalWrite(IN1pin, LOW);
      digitalWrite(IN2pin, LOW);
      analogWrite(ENApin, 0);
    }

  }

  
  delay(100);
}
 
/*
const int potPin = A0;       // Analog pin on the Uno
const int windowSize = 5;
int values[windowSize];      // Array to store readings
int readIndex = 0;           // Current index in the array
long total = 0;              // Running total
int avg_emg = 0;             // Moving average
 
 
void setup() {
  // put your setup code here, to run once:
 
  Serial.begin(9600);
 
  // Initialize the smoothing array with 0s
  for (int i = 0; i < windowSize; i++) {
    values[i] = 0;
  }
 
}
 
void loop() {
  // put your main code here, to run repeatedly:
 
  // Subtract the oldest reading, add the new one, and advance the index
  total = total - values[readIndex];      
  values[readIndex] = analogRead(potPin);
  total = total + values[readIndex];      
  readIndex = (readIndex + 1) % windowSize;
 
  avg_emg = total / windowSize;  
 
 // Serial.println(500);          
 
  Serial.println(avg_emg);
  delay(50);
}
*/
