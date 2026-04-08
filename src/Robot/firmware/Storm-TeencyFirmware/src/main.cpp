#include <Arduino.h>
#include <Servo.h>

// ============================================================
// Teensy 4.1 I/O coprocessor firmware
//
// JSON serial protocol is intentionally compatible with the
// existing nano_every_coprocessor sketch used by this repo.
// Update the pin maps below to match your exact robot wiring.
// ============================================================

static const unsigned long SERIAL_BAUD = 115200;
static const unsigned long TELEMETRY_INTERVAL_MS = 100;
static const unsigned long COMMAND_TIMEOUT_MS = 500;
// Generic PWM+DIR motor drivers are usually happiest with a low-kHz PWM input.
static const uint32_t MOTOR_PWM_FREQUENCY_HZ = 1000;
static const uint32_t LED_PWM_FREQUENCY_HZ = 1200;
static const uint8_t PWM_RESOLUTION_BITS = 8;
static const int MOTOR_PWM_MAX = (1 << PWM_RESOLUTION_BITS) - 1;
static const float MOTOR_ZERO_EPSILON = 0.001f;

static const uint8_t PIN_UNUSED = 0xFF;

static const int DRIVE_MOTOR_COUNT = 4;
static const int MECH_MOTOR_COUNT = 3;
static const int TOTAL_MOTOR_COUNT = DRIVE_MOTOR_COUNT + MECH_MOTOR_COUNT;
static const int RELAY_COUNT = 2;
static const int LED_GROUP_COUNT = 1;
static const int SERVO_COUNT = 3;
static const int MAX_ENCODERS = TOTAL_MOTOR_COUNT;

struct MotorPinConfig
{
  uint8_t pwmPin;
  uint8_t dirPin;
  bool invertDir;
};

struct RelayConfig
{
  uint8_t pin;
  bool activeHigh;
};

struct LedGroupConfig
{
  uint8_t rPin;
  uint8_t gPin;
  uint8_t bPin;
  uint8_t wPin;
  bool commonAnode;
};

struct ServoConfig
{
  uint8_t pin;
  int minUs;
  int maxUs;
};

struct EncoderConfig
{
  uint8_t pinA;
  uint8_t pinB;
  bool invert;
};

const MotorPinConfig DRIVE_MOTORS[DRIVE_MOTOR_COUNT] = {
    {2, 22, false},
    {3, 23, false},
    {4, 28, false},
    {5, 29, false},
};

const MotorPinConfig MECH_MOTORS[MECH_MOTOR_COUNT] = {
    {6, 30, false},
    {7, 31, false},
    {8, 32, false},
};

const RelayConfig RELAYS[RELAY_COUNT] = {
    {33, true},
    {34, true},
};

const LedGroupConfig LED_GROUPS[LED_GROUP_COUNT] = {
    {9, 10, 11, 12, false},
};

const ServoConfig SERVOS[SERVO_COUNT] = {
    {35, 500, 2500},
    {36, 500, 2500},
    {37, 500, 2500},
};

const EncoderConfig ENCODERS[MAX_ENCODERS] = {
    {14, 15, false},
    {16, 17, false},
    {18, 19, false},
    {20, 21, false},
    {24, 25, false},
    {26, 27, false},
    {38, 39, false},
};

static const int8_t QUADRATURE_TABLE[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0,
};

String serialLine;
String robotMode = "STOPPED";

float driveMotorCmd[DRIVE_MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
float mechMotorCmd[MECH_MOTOR_COUNT] = {0.0f, 0.0f, 0.0f};
bool relayState[RELAY_COUNT] = {false, false};
uint8_t ledValue[LED_GROUP_COUNT][4] = {{0, 0, 0, 0}};
float servoValue[SERVO_COUNT] = {0.0f, 0.0f, 0.0f};
bool servoAttached[SERVO_COUNT] = {false, false, false};

volatile long encoderCounts[MAX_ENCODERS] = {0};
volatile uint8_t encoderState[MAX_ENCODERS] = {0};
bool encoderEnabled[MAX_ENCODERS] = {false};
bool encodersAvailable = false;

Servo servoObjects[SERVO_COUNT];

unsigned long lastTelemetryMs = 0;
unsigned long lastCommandMs = 0;

float clampUnit(float value)
{
  if (value > 1.0f)
    return 1.0f;
  if (value < -1.0f)
    return -1.0f;
  return value;
}

int clampByte(int value)
{
  if (value < 0)
    return 0;
  if (value > 255)
    return 255;
  return value;
}

bool pinConfigured(uint8_t pin)
{
  return pin != PIN_UNUSED;
}

bool motorsEnabledByMode()
{
  return robotMode == "TELEOP" || robotMode == "AUTO";
}

bool isValidMode(const String &mode)
{
  return mode == "STOPPED" || mode == "TELEOP" || mode == "AUTO";
}

float parseFloatSafe(const String &s, float fallback = 0.0f)
{
  String trimmed = s;
  trimmed.trim();
  if (trimmed.length() == 0)
    return fallback;

  char first = trimmed[0];
  if (!(isDigit(first) || first == '-' || first == '+' || first == '.'))
    return fallback;

  return trimmed.toFloat();
}

int parseIntSafe(const String &s, int fallback = 0)
{
  char buf[32];
  s.toCharArray(buf, sizeof(buf));
  char *endptr = nullptr;
  long value = strtol(buf, &endptr, 10);
  if (endptr == buf)
    return fallback;
  return (int)value;
}

bool extractStringField(const String &line, const String &key, String &out)
{
  String pattern = "\"" + key + "\":\"";
  int start = line.indexOf(pattern);
  if (start < 0)
    return false;
  start += pattern.length();
  int end = line.indexOf("\"", start);
  if (end < 0)
    return false;
  out = line.substring(start, end);
  return true;
}

bool extractBoolField(const String &line, const String &key, bool &out)
{
  String patternTrue = "\"" + key + "\":true";
  String patternFalse = "\"" + key + "\":false";
  if (line.indexOf(patternTrue) >= 0)
  {
    out = true;
    return true;
  }
  if (line.indexOf(patternFalse) >= 0)
  {
    out = false;
    return true;
  }
  return false;
}

bool extractIntField(const String &line, const String &key, int &out)
{
  String pattern = "\"" + key + "\":";
  int start = line.indexOf(pattern);
  if (start < 0)
    return false;
  start += pattern.length();

  int end = start;
  while (end < (int)line.length() && (isDigit(line[end]) || line[end] == '-'))
  {
    end++;
  }

  out = parseIntSafe(line.substring(start, end));
  return true;
}

bool extractFloatField(const String &line, const String &key, float &out)
{
  String pattern = "\"" + key + "\":";
  int start = line.indexOf(pattern);
  if (start < 0)
    return false;
  start += pattern.length();

  int end = start;
  while (end < (int)line.length())
  {
    char c = line[end];
    if (!(isDigit(c) || c == '-' || c == '+' || c == '.'))
      break;
    end++;
  }

  out = parseFloatSafe(line.substring(start, end));
  return true;
}

bool extractFloatArray(const String &line, const String &key, float *arr, int expectedCount)
{
  String pattern = "\"" + key + "\":[";
  int start = line.indexOf(pattern);
  if (start < 0)
    return false;
  start += pattern.length();

  int end = line.indexOf("]", start);
  if (end < 0)
    return false;

  String body = line.substring(start, end);
  int idx = 0;
  int from = 0;

  while (from < (int)body.length() && idx < expectedCount)
  {
    int comma = body.indexOf(',', from);
    String token;
    if (comma < 0)
    {
      token = body.substring(from);
      from = body.length();
    }
    else
    {
      token = body.substring(from, comma);
      from = comma + 1;
    }

    token.trim();
    arr[idx++] = clampUnit(parseFloatSafe(token, 0.0f));
  }

  return idx == expectedCount;
}

void sendAck(const char *cmd)
{
  Serial.print("{\"type\":\"ack\",\"cmd\":\"");
  Serial.print(cmd);
  Serial.println("\"}");
}

void sendFault(const char *msg)
{
  Serial.print("{\"type\":\"fault\",\"message\":\"");
  Serial.print(msg);
  Serial.println("\"}");
}

void sendPong()
{
  Serial.print("{\"type\":\"pong\",\"timestamp_ms\":");
  Serial.print(millis());
  Serial.println("}");
}

void sendHello()
{
  Serial.print("{\"type\":\"hello\",\"board\":\"teensy_4_1\",\"fw\":\"0.2.0\"");
  Serial.print(",\"encoders_enabled\":");
  Serial.print(encodersAvailable ? "true" : "false");
  Serial.print(",\"motor_pwm_hz\":");
  Serial.print(MOTOR_PWM_FREQUENCY_HZ);
  Serial.print(",\"motor_pwm_bits\":");
  Serial.print(PWM_RESOLUTION_BITS);
  Serial.print(",\"drive_motors\":");
  Serial.print(DRIVE_MOTOR_COUNT);
  Serial.print(",\"mech_motors\":");
  Serial.print(MECH_MOTOR_COUNT);
  Serial.print(",\"relays\":");
  Serial.print(RELAY_COUNT);
  Serial.print(",\"servos\":");
  Serial.print(SERVO_COUNT);
  Serial.print(",\"drive_pwm_pins\":[");
  for (int i = 0; i < DRIVE_MOTOR_COUNT; i++)
  {
    Serial.print(DRIVE_MOTORS[i].pwmPin);
    if (i < DRIVE_MOTOR_COUNT - 1)
      Serial.print(",");
  }
  Serial.print("],\"drive_dir_pins\":[");
  for (int i = 0; i < DRIVE_MOTOR_COUNT; i++)
  {
    Serial.print(DRIVE_MOTORS[i].dirPin);
    if (i < DRIVE_MOTOR_COUNT - 1)
      Serial.print(",");
  }
  Serial.print("],\"encoder_pins\":[");
  for (int i = 0; i < MAX_ENCODERS; i++)
  {
    Serial.print("[");
    Serial.print(ENCODERS[i].pinA);
    Serial.print(",");
    Serial.print(ENCODERS[i].pinB);
    Serial.print("]");
    if (i < MAX_ENCODERS - 1)
      Serial.print(",");
  }
  Serial.println("}");
}

void sendStatus(const char *event, const char *detail)
{
  Serial.print("{\"type\":\"status\",\"event\":\"");
  Serial.print(event);
  Serial.print("\",\"detail\":\"");
  Serial.print(detail);
  Serial.print("\",\"mode\":\"");
  Serial.print(robotMode);
  Serial.println("\"}");
}

void sendTelemetry()
{
  Serial.print("{\"type\":\"telemetry\",\"encoders\":[");
  for (int i = 0; i < TOTAL_MOTOR_COUNT; i++)
  {
    long count = 0;
    if (encoderEnabled[i])
    {
      noInterrupts();
      count = encoderCounts[i];
      interrupts();
    }
    Serial.print(count);
    if (i < TOTAL_MOTOR_COUNT - 1)
      Serial.print(",");
  }

  Serial.print("],\"relay\":");
  Serial.print(RELAY_COUNT > 0 ? (relayState[0] ? 1 : 0) : 0);
  Serial.print(",\"relay_states\":[");
  for (int i = 0; i < RELAY_COUNT; i++)
  {
    Serial.print(relayState[i] ? 1 : 0);
    if (i < RELAY_COUNT - 1)
      Serial.print(",");
  }

  Serial.print("],\"servo\":[");
  for (int i = 0; i < SERVO_COUNT; i++)
  {
    Serial.print(servoValue[i], 3);
    if (i < SERVO_COUNT - 1)
      Serial.print(",");
  }

  Serial.print("],\"led\":[");
  for (int group = 0; group < LED_GROUP_COUNT; group++)
  {
    Serial.print("[");
    for (int ch = 0; ch < 4; ch++)
    {
      Serial.print(ledValue[group][ch]);
      if (ch < 3)
        Serial.print(",");
    }
    Serial.print("]");
    if (group < LED_GROUP_COUNT - 1)
      Serial.print(",");
  }

  Serial.print("],\"mode\":\"");
  Serial.print(robotMode);
  Serial.print("\",\"drive_enabled\":");
  Serial.print(motorsEnabledByMode() ? "true" : "false");
  Serial.print(",\"drive_cmd\":[");
  for (int i = 0; i < DRIVE_MOTOR_COUNT; i++)
  {
    Serial.print(driveMotorCmd[i], 3);
    if (i < DRIVE_MOTOR_COUNT - 1)
      Serial.print(",");
  }

  Serial.print("],\"mech_cmd\":[");
  for (int i = 0; i < MECH_MOTOR_COUNT; i++)
  {
    Serial.print(mechMotorCmd[i], 3);
    if (i < MECH_MOTOR_COUNT - 1)
      Serial.print(",");
  }

  Serial.println("],\"status\":\"ok\"}");
}

void configurePwmPin(uint8_t pin, uint32_t frequency)
{
  if (!pinConfigured(pin))
    return;
  pinMode(pin, OUTPUT);
  analogWriteFrequency(pin, frequency);
  analogWrite(pin, 0);
}

void writeMotor(const MotorPinConfig &cfg, float cmd)
{
  if (!pinConfigured(cfg.pwmPin) || !pinConfigured(cfg.dirPin))
    return;

  cmd = clampUnit(cmd);
  int pwmValue = 0;
  if (fabsf(cmd) > MOTOR_ZERO_EPSILON)
  {
    pwmValue = (int)(fabsf(cmd) * (float)MOTOR_PWM_MAX);
    if (pwmValue > MOTOR_PWM_MAX)
      pwmValue = MOTOR_PWM_MAX;
  }

  bool forward = (cmd >= 0.0f);
  if (cfg.invertDir)
    forward = !forward;

  digitalWrite(cfg.dirPin, forward ? HIGH : LOW);
  analogWrite(cfg.pwmPin, pwmValue);
}

void applyMotorArray(const MotorPinConfig *motors, const float *commands, int count, bool enabled)
{
  for (int i = 0; i < count; i++)
  {
    writeMotor(motors[i], enabled ? commands[i] : 0.0f);
  }
}

void applyDriveMotors()
{
  applyMotorArray(DRIVE_MOTORS, driveMotorCmd, DRIVE_MOTOR_COUNT, motorsEnabledByMode());
}

void applyMechMotors()
{
  applyMotorArray(MECH_MOTORS, mechMotorCmd, MECH_MOTOR_COUNT, motorsEnabledByMode());
}

void stopAllMotors()
{
  for (int i = 0; i < DRIVE_MOTOR_COUNT; i++)
  {
    driveMotorCmd[i] = 0.0f;
    writeMotor(DRIVE_MOTORS[i], 0.0f);
  }

  for (int i = 0; i < MECH_MOTOR_COUNT; i++)
  {
    mechMotorCmd[i] = 0.0f;
    writeMotor(MECH_MOTORS[i], 0.0f);
  }
}

void setRelay(int index, bool on)
{
  if (index < 0 || index >= RELAY_COUNT)
    return;
  if (!pinConfigured(RELAYS[index].pin))
    return;

  relayState[index] = on;
  bool pinLevel = RELAYS[index].activeHigh ? on : !on;
  digitalWrite(RELAYS[index].pin, pinLevel ? HIGH : LOW);
}

void writeLedChannel(uint8_t pin, bool commonAnode, uint8_t value)
{
  if (!pinConfigured(pin))
    return;
  analogWrite(pin, commonAnode ? (255 - value) : value);
}

void applyLedGroup(int index)
{
  if (index < 0 || index >= LED_GROUP_COUNT)
    return;

  const LedGroupConfig &cfg = LED_GROUPS[index];
  writeLedChannel(cfg.rPin, cfg.commonAnode, ledValue[index][0]);
  writeLedChannel(cfg.gPin, cfg.commonAnode, ledValue[index][1]);
  writeLedChannel(cfg.bPin, cfg.commonAnode, ledValue[index][2]);
  writeLedChannel(cfg.wPin, cfg.commonAnode, ledValue[index][3]);
}

void setServoChannel(int index, float value)
{
  if (index < 0 || index >= SERVO_COUNT)
    return;
  if (!pinConfigured(SERVOS[index].pin))
    return;

  value = clampUnit(value);
  float normalized = (value + 1.0f) * 0.5f;
  int us = SERVOS[index].minUs + (int)((SERVOS[index].maxUs - SERVOS[index].minUs) * normalized);

  if (!servoAttached[index])
  {
    servoObjects[index].attach(SERVOS[index].pin, SERVOS[index].minUs, SERVOS[index].maxUs);
    servoAttached[index] = true;
  }

  servoValue[index] = value;
  servoObjects[index].writeMicroseconds(us);
}

void detachServoChannel(int index)
{
  if (index < 0 || index >= SERVO_COUNT)
    return;
  if (!servoAttached[index])
    return;

  servoObjects[index].detach();
  servoAttached[index] = false;
}

void handleEncoderChange(uint8_t index)
{
  if (index >= MAX_ENCODERS || !encoderEnabled[index])
    return;

  uint8_t a = digitalRead(ENCODERS[index].pinA);
  uint8_t b = digitalRead(ENCODERS[index].pinB);
  uint8_t current = (a << 1) | b;
  uint8_t transition = (encoderState[index] << 2) | current;
  int8_t delta = QUADRATURE_TABLE[transition & 0x0F];
  if (ENCODERS[index].invert)
    delta = -delta;

  encoderCounts[index] += delta;
  encoderState[index] = current;
}

#define DEFINE_ENCODER_ISR(index)           \
  void encoder##index##A_ISR()              \
  {                                         \
    handleEncoderChange(index);             \
  }                                         \
  void encoder##index##B_ISR()              \
  {                                         \
    handleEncoderChange(index);             \
  }

DEFINE_ENCODER_ISR(0)
DEFINE_ENCODER_ISR(1)
DEFINE_ENCODER_ISR(2)
DEFINE_ENCODER_ISR(3)
DEFINE_ENCODER_ISR(4)
DEFINE_ENCODER_ISR(5)
DEFINE_ENCODER_ISR(6)

typedef void (*EncoderIsrFn)();

const EncoderIsrFn ENCODER_ISR_A[MAX_ENCODERS] = {
    encoder0A_ISR,
    encoder1A_ISR,
    encoder2A_ISR,
    encoder3A_ISR,
    encoder4A_ISR,
    encoder5A_ISR,
    encoder6A_ISR,
};

const EncoderIsrFn ENCODER_ISR_B[MAX_ENCODERS] = {
    encoder0B_ISR,
    encoder1B_ISR,
    encoder2B_ISR,
    encoder3B_ISR,
    encoder4B_ISR,
    encoder5B_ISR,
    encoder6B_ISR,
};

void initEncoders()
{
  encodersAvailable = false;

  for (int i = 0; i < MAX_ENCODERS; i++)
  {
    encoderEnabled[i] = false;
    if (!pinConfigured(ENCODERS[i].pinA) || !pinConfigured(ENCODERS[i].pinB))
      continue;

    pinMode(ENCODERS[i].pinA, INPUT_PULLUP);
    pinMode(ENCODERS[i].pinB, INPUT_PULLUP);

    uint8_t a = digitalRead(ENCODERS[i].pinA);
    uint8_t b = digitalRead(ENCODERS[i].pinB);
    encoderState[i] = (a << 1) | b;
    encoderCounts[i] = 0;
    encoderEnabled[i] = true;
    encodersAvailable = true;

    attachInterrupt(digitalPinToInterrupt(ENCODERS[i].pinA), ENCODER_ISR_A[i], CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODERS[i].pinB), ENCODER_ISR_B[i], CHANGE);
  }
}

void handleDriveCommand(const String &line)
{
  float values[DRIVE_MOTOR_COUNT];
  if (!extractFloatArray(line, "motors", values, DRIVE_MOTOR_COUNT))
  {
    sendFault("bad_drive_array");
    return;
  }

  for (int i = 0; i < DRIVE_MOTOR_COUNT; i++)
  {
    driveMotorCmd[i] = values[i];
  }

  applyDriveMotors();
  sendAck("drive");
}

void handleMechCommand(const String &line)
{
  float values[MECH_MOTOR_COUNT];
  if (!extractFloatArray(line, "motors", values, MECH_MOTOR_COUNT))
  {
    sendFault("bad_mech_array");
    return;
  }

  for (int i = 0; i < MECH_MOTOR_COUNT; i++)
  {
    mechMotorCmd[i] = values[i];
  }

  applyMechMotors();
  sendAck("mech");
}

void handleServoCommand(const String &line)
{
  int channel = 0;
  float value = 0.0f;

  if (!extractIntField(line, "channel", channel))
  {
    sendFault("missing_servo_channel");
    return;
  }

  if (!extractFloatField(line, "value", value))
  {
    sendFault("missing_servo_value");
    return;
  }

  setServoChannel(channel, value);
  sendAck("servo");
}

void handleServoDetachCommand(const String &line)
{
  int channel = 0;
  if (!extractIntField(line, "channel", channel))
  {
    sendFault("missing_servo_channel");
    return;
  }

  detachServoChannel(channel);
  sendAck("servo_detach");
}

void handleRelayCommand(const String &line)
{
  bool on = false;
  int index = 0;

  extractIntField(line, "index", index);
  if (!extractBoolField(line, "on", on))
  {
    sendFault("missing_relay_bool");
    return;
  }

  if (index < 0 || index >= RELAY_COUNT)
  {
    sendFault("bad_relay_index");
    return;
  }

  setRelay(index, on);
  sendAck("relay");
}

void handleLedCommand(const String &line)
{
  int index = 0;
  int r = 0;
  int g = 0;
  int b = 0;
  int w = 0;

  extractIntField(line, "index", index);
  if (index < 0 || index >= LED_GROUP_COUNT)
  {
    sendFault("bad_led_index");
    return;
  }

  extractIntField(line, "r", r);
  extractIntField(line, "g", g);
  extractIntField(line, "b", b);
  extractIntField(line, "w", w);

  ledValue[index][0] = clampByte(r);
  ledValue[index][1] = clampByte(g);
  ledValue[index][2] = clampByte(b);
  ledValue[index][3] = clampByte(w);
  applyLedGroup(index);
  sendAck("led");
}

void handleModeCommand(const String &line)
{
  String mode;
  if (!extractStringField(line, "mode", mode))
  {
    sendFault("missing_mode");
    return;
  }

  mode.toUpperCase();
  if (!isValidMode(mode))
  {
    sendFault("invalid_mode");
    return;
  }

  robotMode = mode;
  applyDriveMotors();
  applyMechMotors();
  sendStatus("mode_changed", mode.c_str());
  sendAck("mode");
}

void handleResetCommand()
{
  stopAllMotors();
  for (int i = 0; i < RELAY_COUNT; i++)
  {
    setRelay(i, false);
  }

  for (int led = 0; led < LED_GROUP_COUNT; led++)
  {
    for (int ch = 0; ch < 4; ch++)
    {
      ledValue[led][ch] = 0;
    }
    applyLedGroup(led);
  }

  for (int i = 0; i < SERVO_COUNT; i++)
  {
    detachServoChannel(i);
    servoValue[i] = 0.0f;
  }

  noInterrupts();
  for (int i = 0; i < MAX_ENCODERS; i++)
  {
    encoderCounts[i] = 0;
  }
  interrupts();

  robotMode = "STOPPED";
  sendStatus("reset_complete", "ok");
  sendAck("reset");
}

void handleLine(const String &line)
{
  if (line.indexOf("\"type\":\"ping\"") >= 0)
  {
    sendPong();
    return;
  }

  lastCommandMs = millis();

  if (line.indexOf("\"type\":\"drive\"") >= 0)
  {
    handleDriveCommand(line);
    return;
  }

  if (line.indexOf("\"type\":\"mech\"") >= 0)
  {
    handleMechCommand(line);
    return;
  }

  if (line.indexOf("\"type\":\"servo_detach\"") >= 0)
  {
    handleServoDetachCommand(line);
    return;
  }

  if (line.indexOf("\"type\":\"servo\"") >= 0)
  {
    handleServoCommand(line);
    return;
  }

  if (line.indexOf("\"type\":\"relay\"") >= 0)
  {
    handleRelayCommand(line);
    return;
  }

  if (line.indexOf("\"type\":\"led\"") >= 0)
  {
    handleLedCommand(line);
    return;
  }

  if (line.indexOf("\"type\":\"mode\"") >= 0)
  {
    handleModeCommand(line);
    return;
  }

  if (line.indexOf("\"type\":\"reset\"") >= 0)
  {
    handleResetCommand();
    return;
  }

  sendFault("unknown_command");
}

void setup()
{
  analogWriteResolution(PWM_RESOLUTION_BITS);
  Serial.begin(SERIAL_BAUD);

  for (int i = 0; i < DRIVE_MOTOR_COUNT; i++)
  {
    configurePwmPin(DRIVE_MOTORS[i].pwmPin, MOTOR_PWM_FREQUENCY_HZ);
    if (pinConfigured(DRIVE_MOTORS[i].dirPin))
    {
      pinMode(DRIVE_MOTORS[i].dirPin, OUTPUT);
      digitalWrite(DRIVE_MOTORS[i].dirPin, LOW);
    }
  }

  for (int i = 0; i < MECH_MOTOR_COUNT; i++)
  {
    configurePwmPin(MECH_MOTORS[i].pwmPin, MOTOR_PWM_FREQUENCY_HZ);
    if (pinConfigured(MECH_MOTORS[i].dirPin))
    {
      pinMode(MECH_MOTORS[i].dirPin, OUTPUT);
      digitalWrite(MECH_MOTORS[i].dirPin, LOW);
    }
  }

  for (int i = 0; i < RELAY_COUNT; i++)
  {
    if (!pinConfigured(RELAYS[i].pin))
      continue;
    pinMode(RELAYS[i].pin, OUTPUT);
    setRelay(i, false);
  }

  for (int i = 0; i < LED_GROUP_COUNT; i++)
  {
    configurePwmPin(LED_GROUPS[i].rPin, LED_PWM_FREQUENCY_HZ);
    configurePwmPin(LED_GROUPS[i].gPin, LED_PWM_FREQUENCY_HZ);
    configurePwmPin(LED_GROUPS[i].bPin, LED_PWM_FREQUENCY_HZ);
    configurePwmPin(LED_GROUPS[i].wPin, LED_PWM_FREQUENCY_HZ);
    applyLedGroup(i);
  }

  stopAllMotors();
  initEncoders();

  delay(200);
  sendHello();

  lastTelemetryMs = millis();
  lastCommandMs = millis();
}

void loop()
{
  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();

    if (c == '\n')
    {
      serialLine.trim();
      if (serialLine.length() > 0)
      {
        handleLine(serialLine);
      }
      serialLine = "";
    }
    else if (c != '\r')
    {
      serialLine += c;
      if (serialLine.length() > 400)
      {
        serialLine = "";
        sendFault("line_too_long");
      }
    }
  }

  unsigned long now = millis();
  if ((now - lastCommandMs) > COMMAND_TIMEOUT_MS)
  {
    if (robotMode != "STOPPED")
    {
      sendStatus("watchdog_timeout", "forcing_stopped");
    }
    stopAllMotors();
    robotMode = "STOPPED";
    lastCommandMs = now;
  }

  if ((now - lastTelemetryMs) >= TELEMETRY_INTERVAL_MS)
  {
    lastTelemetryMs = now;
    sendTelemetry();
  }
}
