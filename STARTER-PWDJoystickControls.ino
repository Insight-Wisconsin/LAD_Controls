/*
 * PWD Controls - Starter Joystick-to-Motor Pipeline
 * Insight Wisconsin | Powered Wheelchair Device
 *
 * Pipeline (runs every LOOP_MS):
 *   raw ADC -> normalize (-1..1) -> low-pass filter (tremor smoothing)
 *           -> dead zone -> speed cap -> acceleration ramp
 *           -> differential drive mix -> motor driver PWM/DIR
 *
 * Safety:
 *   - E-stop wired normally-closed to GND (INPUT_PULLUP). LOW = OK, HIGH = STOP.
 *     A broken wire reads HIGH, so a cut cable fails safe.
 *   - If a joystick axis reads at the ADC rails for FAULT_MS, treat the
 *     joystick as disconnected and stop.
 *   - Output is held at zero until the joystick has been centered after
 *     startup or after any fault (prevents the chair lurching on boot).
 *
 * For ESP32, change ADC_MAX to 4095
 * and replace analogWrite with ledcWrite (see notes at bottom).
 */

// ---------- Pins (change to match wiring) ----------
const int PIN_JOY_X  = A0;  // left/right axis
const int PIN_JOY_Y  = A1;  // forward/back axis
const int PIN_ESTOP  = 2;   // NC e-stop to GND, INPUT_PULLUP

const int PIN_L_PWM  = 5;   // left motor PWM
const int PIN_L_DIR  = 4;   // left motor direction
const int PIN_R_PWM  = 6;   // right motor PWM
const int PIN_R_DIR  = 7;   // right motor direction

// ---------- Tunable parameters (team tests and tunes) ----------
const int   ADC_MAX        = 1023;
const float DEAD_ZONE      = 0.12f;  // fraction of full deflection ignored (raise for more tremor)
const float FILTER_ALPHA   = 0.15f;  // 0..1, smaller = smoother but laggier
const float MAX_SPEED      = 0.50f;  // speed cap, fraction of full power (set speed modes later)
const float TURN_SCALE     = 0.60f;  // limit turning authority relative to throttle
const float ACCEL_LIMIT    = 0.80f;  // output change per second when speeding up
const float DECEL_LIMIT    = 1.60f;  // output change per second when slowing (stop faster than start)
const unsigned long LOOP_MS  = 20;   // 50 Hz control loop
const unsigned long FAULT_MS = 200;  // rail-reading time before declaring a fault
const int   RAIL_MARGIN    = 20;     // ADC counts from 0/max that count as "at the rail"
const float CENTER_WINDOW  = 0.15f;  // joystick must be within this of center to arm

// ---------- State ----------
int   centerX = ADC_MAX / 2;
int   centerY = ADC_MAX / 2;
float filtX = 0, filtY = 0;          // filtered inputs
float outThrottle = 0, outTurn = 0;  // ramped outputs
unsigned long lastLoop = 0;
unsigned long railSince = 0;
bool  armed = false;                 // must see centered stick before driving
bool  faulted = true;                // start faulted until first good, centered read

// ---------- Helpers ----------
float normalize(int raw, int center) {
  // Map raw ADC to -1..1 around the calibrated center.
  float v;
  if (raw >= center) v = (float)(raw - center) / (ADC_MAX - center);
  else               v = (float)(raw - center) / center;
  return constrain(v, -1.0f, 1.0f);
}

float applyDeadZone(float v, float dz) {
  // Ignore small movement, then rescale so output still reaches +/-1.
  float a = fabs(v);
  if (a < dz) return 0.0f;
  float scaled = (a - dz) / (1.0f - dz);
  return (v > 0) ? scaled : -scaled;
}

float lowPass(float prev, float input, float alpha) {
  return prev + alpha * (input - prev);
}

float slew(float current, float target, float dt) {
  // Acceleration limiter. Uses a different rate when speeding up vs slowing down.
  bool speedingUp = fabs(target) > fabs(current);
  float rate = speedingUp ? ACCEL_LIMIT : DECEL_LIMIT;
  float maxStep = rate * dt;
  float step = constrain(target - current, -maxStep, maxStep);
  return current + step;
}

void setMotor(int pwmPin, int dirPin, float value) {
  // value in -1..1. Adjust HIGH/LOW for your driver's direction convention.
  digitalWrite(dirPin, value >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, (int)(fabs(value) * 255.0f));
}

void stopMotors() {
  analogWrite(PIN_L_PWM, 0);
  analogWrite(PIN_R_PWM, 0);
}

void calibrateCenter() {
  // Joystick must be hands-off at startup.
  long sx = 0, sy = 0;
  const int N = 50;
  for (int i = 0; i < N; i++) {
    sx += analogRead(PIN_JOY_X);
    sy += analogRead(PIN_JOY_Y);
    delay(5);
  }
  centerX = sx / N;
  centerY = sy / N;
}

// ---------- Arduino entry points ----------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_L_PWM, OUTPUT); pinMode(PIN_L_DIR, OUTPUT);
  pinMode(PIN_R_PWM, OUTPUT); pinMode(PIN_R_DIR, OUTPUT);
  stopMotors();
  delay(500);
  calibrateCenter();
  Serial.println("rawX,rawY,filtX,filtY,throttle,turn,armed,fault");
}

void loop() {
  unsigned long now = millis();
  if (now - lastLoop < LOOP_MS) return;
  float dt = (now - lastLoop) / 1000.0f;
  lastLoop = now;

  int rawX = analogRead(PIN_JOY_X);
  int rawY = analogRead(PIN_JOY_Y);

  // --- Fault detection ---
  bool estop = (digitalRead(PIN_ESTOP) == HIGH);  // HIGH = pressed OR wire broken
  bool atRail = (rawX < RAIL_MARGIN || rawX > ADC_MAX - RAIL_MARGIN ||
                 rawY < RAIL_MARGIN || rawY > ADC_MAX - RAIL_MARGIN);
  if (atRail) {
    if (railSince == 0) railSince = now;
  } else {
    railSince = 0;
  }
  bool lostSignal = (railSince != 0 && (now - railSince) > FAULT_MS);

  if (estop || lostSignal) {
    faulted = true;
    armed = false;
  }

  // --- Normalize + filter (always run so filter state stays current) ---
  float nx = normalize(rawX, centerX);   // turn axis
  float ny = normalize(rawY, centerY);   // throttle axis
  filtX = lowPass(filtX, nx, FILTER_ALPHA);
  filtY = lowPass(filtY, ny, FILTER_ALPHA);

  // --- Re-arm logic: needs no fault condition and a centered stick ---
  if (!estop && !lostSignal && faulted) {
    if (fabs(filtX) < CENTER_WINDOW && fabs(filtY) < CENTER_WINDOW) {
      faulted = false;
      armed = true;
    }
  }

  // --- Targets ---
  float targetThrottle = 0, targetTurn = 0;
  if (armed && !faulted) {
    targetThrottle = applyDeadZone(filtY, DEAD_ZONE) * MAX_SPEED;
    targetTurn     = applyDeadZone(filtX, DEAD_ZONE) * MAX_SPEED * TURN_SCALE;
  }

  // --- Ramp, or hard stop on fault ---
  if (faulted) {
    outThrottle = 0;
    outTurn = 0;
    stopMotors();   // TODO: also engage the hardware brake here
  } else {
    outThrottle = slew(outThrottle, targetThrottle, dt);
    outTurn     = slew(outTurn, targetTurn, dt);

    // Differential drive mix
    float left  = outThrottle + outTurn;
    float right = outThrottle - outTurn;
    float peak  = max(fabs(left), fabs(right));
    if (peak > 1.0f) { left /= peak; right /= peak; }

    setMotor(PIN_L_PWM, PIN_L_DIR, left);
    setMotor(PIN_R_PWM, PIN_R_DIR, right);
  }

  // --- Log for Serial Plotter / CSV capture ---
  Serial.print(rawX);       Serial.print(',');
  Serial.print(rawY);       Serial.print(',');
  Serial.print(filtX, 3);   Serial.print(',');
  Serial.print(filtY, 3);   Serial.print(',');
  Serial.print(outThrottle, 3); Serial.print(',');
  Serial.print(outTurn, 3);     Serial.print(',');
  Serial.print(armed);      Serial.print(',');
  Serial.println(faulted);
}

/*
 * ---------- Ideas for the team to extend ----------
 * 1. Speed modes: add a +/- keypad that changes MAX_SPEED in steps (the "set speed" idea).
 * 2. Move all tunables into a config struct so they can be changed at runtime.
 * 3. Replace the low-pass filter with a median + low-pass or a notch filter targeting
 *    typical tremor frequencies (roughly 4-12 Hz); measure real data first.
 * 4. Swap the input layer: put the joystick read in one function that returns (x, y)
 *    so an isometric stick or touchpad can drop in later.
 * 5. Add a hardware watchdog timer so a frozen loop stops the chair.
 * 6. Add a Python script that reads the Serial CSV and plots input vs. output.
 *
 * ESP32 notes: ADC_MAX = 4095 (ADC is non-linear, calibrate), use ledcSetup/ledcAttachPin/
 * ledcWrite instead of analogWrite on older cores, and use a 3.3V joystick.
 */
