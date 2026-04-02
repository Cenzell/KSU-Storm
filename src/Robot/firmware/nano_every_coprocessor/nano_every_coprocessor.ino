#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

// ------------------------------
// Configuration
// ------------------------------
static const uint32_t BAUD_RATE = 115200;

static const uint8_t NUM_MOTORS = 2;
static const uint8_t motorPwmPins[NUM_MOTORS] = {9, 10};
static const uint8_t motorDirPins[NUM_MOTORS] = {4, 7};

static const uint8_t NUM_RELAYS = 1;
static const uint8_t relayPins[NUM_RELAYS] = {8};

static const uint8_t NUM_ENCODERS = 2;
// NOTE: Change these to match your wiring. Best practice: A channels on interrupt-capable pins.
static const uint8_t encAPins[NUM_ENCODERS] = {2, 3};
static const uint8_t encBPins[NUM_ENCODERS] = {12, 13};

static const uint8_t NUM_SERVOS = 2;
static const uint8_t servoChannels[NUM_SERVOS] = {0, 1};
static const uint8_t PCA9685_ADDRESS = 0x40;
static const uint16_t PCA9685_FREQ_HZ = 50;
static const uint16_t SERVO_US_MIN = 500;
static const uint16_t SERVO_US_MAX = 2500;

// Safety / telemetry
static uint32_t watchdogTimeoutMs = 400;
static uint16_t telemetryRateHz = 20;

// ------------------------------
// Runtime state
// ------------------------------
volatile int32_t encoderCounts[NUM_ENCODERS] = {0};
volatile uint8_t encoderALast[NUM_ENCODERS] = {0};

int16_t motorCmdSigned[NUM_MOTORS] = {0}; // -255..255
uint8_t relayState[NUM_RELAYS] = {0};     // 0/1

uint32_t lastCommandMs = 0;
uint32_t lastTelemetryMs = 0;

char lineBuf[96];
uint8_t lineLen = 0;

// ------------------------------
// Helpers
// ------------------------------
static int32_t parseIntChecked(const char *s, bool &ok)
{
  if (s == nullptr)
  {
    ok = false;
    return 0;
  }
  char *endPtr = nullptr;
  long v = strtol(s, &endPtr, 10);
  ok = (endPtr != s && *endPtr == '\0');
  return (int32_t)v;
}

static inline int16_t clampS16(int32_t v, int16_t lo, int16_t hi)
{
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return (int16_t)v;
}

static inline uint16_t clampU16(int32_t v, uint16_t lo, uint16_t hi)
{
  if (v < (int32_t)lo)
    return lo;
  if (v > (int32_t)hi)
    return hi;
  return (uint16_t)v;
}

static uint16_t usToPcaTicks(uint16_t pulseUs)
{
  // ticks = pulse_us / period_us * 4096, where period_us = 1e6 / freq
  const uint32_t periodUs = 1000000UL / PCA9685_FREQ_HZ;
  return (uint16_t)((((uint32_t)pulseUs) * 4096UL) / periodUs);
}

static void setServoUs(uint8_t id, uint16_t pulseUs)
{
  if (id >= NUM_SERVOS)
    return;

  pulseUs = clampU16((int32_t)pulseUs, SERVO_US_MIN, SERVO_US_MAX);
  const uint16_t ticks = usToPcaTicks(pulseUs);
  pca.setPWM(servoChannels[id], 0, ticks);
}

static void setServoAngle(uint8_t id, uint8_t angleDeg)
{
  if (id >= NUM_SERVOS)
    return;

  angleDeg = (uint8_t)clampU16((int32_t)angleDeg, 0, 180);
  const uint16_t pulseUs = SERVO_US_MIN + ((uint32_t)angleDeg * (SERVO_US_MAX - SERVO_US_MIN)) / 180UL;
  setServoUs(id, pulseUs);
}

static void disableServo(uint8_t id)
{
  if (id >= NUM_SERVOS)
    return;
  pca.setPWM(servoChannels[id], 0, 0);
}

static void setMotorRaw(uint8_t idx, int16_t signedPwm)
{
  if (idx >= NUM_MOTORS)
    return;

  signedPwm = clampS16(signedPwm, -255, 255);
  motorCmdSigned[idx] = signedPwm;

  bool dir = (signedPwm >= 0);
  uint8_t duty = (uint8_t)abs(signedPwm);

  digitalWrite(motorDirPins[idx], dir ? HIGH : LOW);
  analogWrite(motorPwmPins[idx], duty);
}

static void stopAllMotors()
{
  for (uint8_t i = 0; i < NUM_MOTORS; i++)
  {
    setMotorRaw(i, 0);
  }
}

static void setRelay(uint8_t idx, uint8_t on)
{
  if (idx >= NUM_RELAYS)
    return;
  relayState[idx] = on ? 1 : 0;
  digitalWrite(relayPins[idx], relayState[idx] ? HIGH : LOW);
}

static void sendOk()
{
  Serial.println(F("OK"));
}

static void sendErr(const __FlashStringHelper *msg)
{
  Serial.print(F("ERR,"));
  Serial.println(msg);
}

static void sendEncoders()
{
  noInterrupts();
  int32_t snapshot[NUM_ENCODERS];
  for (uint8_t i = 0; i < NUM_ENCODERS; i++)
  {
    snapshot[i] = encoderCounts[i];
  }
  interrupts();

  Serial.print(F("ENC"));
  for (uint8_t i = 0; i < NUM_ENCODERS; i++)
  {
    Serial.print(',');
    Serial.print(snapshot[i]);
  }
  Serial.println();
}

static void sendStatus()
{
  Serial.print(F("STAT,"));
  Serial.print(millis());
  Serial.print(',');
  Serial.print((uint32_t)(millis() - lastCommandMs));

  for (uint8_t i = 0; i < NUM_MOTORS; i++)
  {
    Serial.print(',');
    Serial.print(motorCmdSigned[i]);
  }

  for (uint8_t i = 0; i < NUM_RELAYS; i++)
  {
    Serial.print(',');
    Serial.print(relayState[i]);
  }

  noInterrupts();
  for (uint8_t i = 0; i < NUM_ENCODERS; i++)
  {
    Serial.print(',');
    Serial.print(encoderCounts[i]);
  }
  interrupts();

  Serial.println();
}

// ------------------------------
// Encoder ISRs (x1 decode on A edge changes)
// ------------------------------
static void encoderUpdate(uint8_t idx)
{
  uint8_t a = (uint8_t)digitalRead(encAPins[idx]);
  if (a == encoderALast[idx])
  {
    return;
  }
  encoderALast[idx] = a;

  uint8_t b = (uint8_t)digitalRead(encBPins[idx]);
  // Direction heuristic for quadrature on A transitions.
  // If this is reversed for your hardware, swap ++ and --.
  if (a == b)
  {
    encoderCounts[idx]++;
  }
  else
  {
    encoderCounts[idx]--;
  }
}

void isrEnc0() { encoderUpdate(0); }
void isrEnc1() { encoderUpdate(1); }

// ------------------------------
// Command parser
// ------------------------------
static void handleCommand(char *line)
{
  // CSV format examples:
  // PING
  // M,<id>,<signed_pwm_-255..255>
  // MD,<id>,<duty_0..255>,<dir_0|1>
  // R,<id>,<0|1>
  // S,<id>,<microseconds_500..2500>
  // SP,<id>,<degrees_0..180>
  // SRVDETACH,<id> (disable PCA9685 channel)
  // Q,ENC | Q,STAT
  // E,RESET
  // W,<timeout_ms>
  // T,<rate_hz>

  char *tok = strtok(line, ",");
  if (tok == nullptr)
  {
    sendErr(F("empty"));
    return;
  }

  if (strcmp(tok, "PING") == 0)
  {
    Serial.println(F("PONG"));
    lastCommandMs = millis();
    return;
  }

  if (strcmp(tok, "M") == 0)
  {
    bool okA = false, okB = false;
    int32_t id = parseIntChecked(strtok(nullptr, ","), okA);
    int32_t pwm = parseIntChecked(strtok(nullptr, ","), okB);
    if (!okA || !okB || id < 0 || id >= NUM_MOTORS)
    {
      sendErr(F("M args"));
      return;
    }
    setMotorRaw((uint8_t)id, clampS16(pwm, -255, 255));
    sendOk();
    lastCommandMs = millis();
    return;
  }

  if (strcmp(tok, "MD") == 0)
  {
    bool okA = false, okB = false, okC = false;
    int32_t id = parseIntChecked(strtok(nullptr, ","), okA);
    int32_t duty = parseIntChecked(strtok(nullptr, ","), okB);
    int32_t dir = parseIntChecked(strtok(nullptr, ","), okC);
    if (!okA || !okB || !okC || id < 0 || id >= NUM_MOTORS)
    {
      sendErr(F("MD args"));
      return;
    }
    int16_t signedPwm = (int16_t)clampU16(duty, 0, 255);
    if (dir == 0)
      signedPwm = -signedPwm;
    setMotorRaw((uint8_t)id, signedPwm);
    sendOk();
    lastCommandMs = millis();
    return;
  }

  if (strcmp(tok, "R") == 0)
  {
    bool okA = false, okB = false;
    int32_t id = parseIntChecked(strtok(nullptr, ","), okA);
    int32_t on = parseIntChecked(strtok(nullptr, ","), okB);
    if (!okA || !okB || id < 0 || id >= NUM_RELAYS)
    {
      sendErr(F("R args"));
      return;
    }
    setRelay((uint8_t)id, on ? 1 : 0);
    sendOk();
    lastCommandMs = millis();
    return;
  }

  if (strcmp(tok, "S") == 0)
  {
    bool okA = false, okB = false;
    int32_t id = parseIntChecked(strtok(nullptr, ","), okA);
    int32_t us = parseIntChecked(strtok(nullptr, ","), okB);
    if (!okA || !okB || id < 0 || id >= NUM_SERVOS)
    {
      sendErr(F("S args"));
      return;
    }
    setServoUs((uint8_t)id, (uint16_t)us);
    sendOk();
    lastCommandMs = millis();
    return;
  }

  if (strcmp(tok, "SP") == 0)
  {
    bool okA = false, okB = false;
    int32_t id = parseIntChecked(strtok(nullptr, ","), okA);
    int32_t deg = parseIntChecked(strtok(nullptr, ","), okB);
    if (!okA || !okB || id < 0 || id >= NUM_SERVOS)
    {
      sendErr(F("SP args"));
      return;
    }
    setServoAngle((uint8_t)id, (uint8_t)deg);
    sendOk();
    lastCommandMs = millis();
    return;
  }

  if (strcmp(tok, "SRVDETACH") == 0)
  {
    bool okA = false;
    int32_t id = parseIntChecked(strtok(nullptr, ","), okA);
    if (!okA || id < 0 || id >= NUM_SERVOS)
    {
      sendErr(F("SRVDETACH args"));
      return;
    }
    disableServo((uint8_t)id);
    sendOk();
    lastCommandMs = millis();
    return;
  }

  if (strcmp(tok, "Q") == 0)
  {
    char *which = strtok(nullptr, ",");
    if (which == nullptr)
    {
      sendErr(F("Q args"));
      return;
    }
    if (strcmp(which, "ENC") == 0)
    {
      sendEncoders();
      return;
    }
    if (strcmp(which, "STAT") == 0)
    {
      sendStatus();
      return;
    }
    sendErr(F("Q type"));
    return;
  }

  if (strcmp(tok, "E") == 0)
  {
    char *what = strtok(nullptr, ",");
    if (what && strcmp(what, "RESET") == 0)
    {
      noInterrupts();
      for (uint8_t i = 0; i < NUM_ENCODERS; i++)
      {
        encoderCounts[i] = 0;
      }
      interrupts();
      sendOk();
      lastCommandMs = millis();
      return;
    }
    sendErr(F("E type"));
    return;
  }

  if (strcmp(tok, "W") == 0)
  {
    bool okA = false;
    int32_t ms = parseIntChecked(strtok(nullptr, ","), okA);
    if (!okA)
    {
      sendErr(F("W args"));
      return;
    }
    watchdogTimeoutMs = clampU16(ms, 50, 5000);
    sendOk();
    lastCommandMs = millis();
    return;
  }

  if (strcmp(tok, "T") == 0)
  {
    bool okA = false;
    int32_t hz = parseIntChecked(strtok(nullptr, ","), okA);
    if (!okA)
    {
      sendErr(F("T args"));
      return;
    }
    telemetryRateHz = clampU16(hz, 0, 100);
    sendOk();
    lastCommandMs = millis();
    return;
  }

  sendErr(F("unknown"));
}

static void serviceSerial()
{
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();

    if (c == '\r')
    {
      continue;
    }

    if (c == '\n')
    {
      lineBuf[lineLen] = '\0';
      if (lineLen > 0)
      {
        handleCommand(lineBuf);
      }
      lineLen = 0;
      continue;
    }

    if (lineLen < sizeof(lineBuf) - 1)
    {
      lineBuf[lineLen++] = c;
    }
    else
    {
      // Overflow: reset line buffer.
      lineLen = 0;
      sendErr(F("line too long"));
    }
  }
}

static void setupPins()
{
  for (uint8_t i = 0; i < NUM_MOTORS; i++)
  {
    pinMode(motorPwmPins[i], OUTPUT);
    pinMode(motorDirPins[i], OUTPUT);
    setMotorRaw(i, 0);
  }

  for (uint8_t i = 0; i < NUM_RELAYS; i++)
  {
    pinMode(relayPins[i], OUTPUT);
    setRelay(i, 0);
  }

  for (uint8_t i = 0; i < NUM_ENCODERS; i++)
  {
    pinMode(encAPins[i], INPUT_PULLUP);
    pinMode(encBPins[i], INPUT_PULLUP);
    encoderALast[i] = (uint8_t)digitalRead(encAPins[i]);
  }

  // Attach up to two encoder A interrupts in this template.
  if (NUM_ENCODERS > 0)
  {
    attachInterrupt(digitalPinToInterrupt(encAPins[0]), isrEnc0, CHANGE);
  }
  if (NUM_ENCODERS > 1)
  {
    attachInterrupt(digitalPinToInterrupt(encAPins[1]), isrEnc1, CHANGE);
  }

  Wire.begin();
  pca = Adafruit_PWMServoDriver(PCA9685_ADDRESS);
  pca.begin();
  pca.setPWMFreq(PCA9685_FREQ_HZ);
  for (uint8_t i = 0; i < NUM_SERVOS; i++)
  {
    disableServo(i);
  }
}

void setup()
{
  Serial.begin(BAUD_RATE);
  setupPins();
  lastCommandMs = millis();
  lastTelemetryMs = millis();

  Serial.println(F("READY,NANO_EVERY_IO_V1"));
}

void loop()
{
  serviceSerial();

  // Watchdog for motors only.
  uint32_t now = millis();
  if ((uint32_t)(now - lastCommandMs) > watchdogTimeoutMs)
  {
    stopAllMotors();
  }

  if (telemetryRateHz > 0)
  {
    uint32_t periodMs = 1000UL / telemetryRateHz;
    if (periodMs < 1)
      periodMs = 1;
    if ((uint32_t)(now - lastTelemetryMs) >= periodMs)
    {
      sendStatus();
      lastTelemetryMs = now;
    }
  }
}
