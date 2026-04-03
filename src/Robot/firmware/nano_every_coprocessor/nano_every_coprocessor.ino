#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ============================================================
// Config
// ============================================================

static const unsigned long SERIAL_BAUD = 115200;
static const unsigned long TELEMETRY_INTERVAL_MS = 100;
static const unsigned long COMMAND_TIMEOUT_MS = 500;

static const int DRIVE_MOTOR_COUNT = 4;
static const int MECH_MOTOR_COUNT = 3;
static const int TOTAL_MOTOR_COUNT = 7;

static const uint8_t PIN_UNUSED = 0xFF;

// 4 drive motors: PWM + DIR
// Order: FL, FR, RL, RR
const uint8_t DRIVE_PWM_PINS[DRIVE_MOTOR_COUNT] = {3, 5, 6, 9};
const uint8_t DRIVE_DIR_PINS[DRIVE_MOTOR_COUNT] = {A0, A1, A2, A3};

// 3 mechanism motors: PWM + DIR
const uint8_t MECH_PWM_PINS[MECH_MOTOR_COUNT] = {PIN_UNUSED, PIN_UNUSED, PIN_UNUSED};
const uint8_t MECH_DIR_PINS[MECH_MOTOR_COUNT] = {PIN_UNUSED, PIN_UNUSED, PIN_UNUSED};

const uint8_t RELAY_PIN = PIN_UNUSED;

const uint8_t LED_R_PIN = PIN_UNUSED;
const uint8_t LED_G_PIN = PIN_UNUSED;
const uint8_t LED_B_PIN = PIN_UNUSED;
const uint8_t LED_W_PIN = PIN_UNUSED;

const uint8_t ENC_A_PINS[TOTAL_MOTOR_COUNT] = {
  PIN_UNUSED, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED
};
const uint8_t ENC_B_PINS[TOTAL_MOTOR_COUNT] = {
  PIN_UNUSED, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED, PIN_UNUSED
};

static const uint8_t PCA9685_ADDR = 0x40;
static const uint16_t PCA9685_FREQ = 50;

static const uint16_t SERVO_MIN_TICKS = 102;
static const uint16_t SERVO_MAX_TICKS = 512;

// ============================================================
// Globals
// ============================================================

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(PCA9685_ADDR);

String serialLine;
String robotMode = "STOPPED";

float driveMotorCmd[DRIVE_MOTOR_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
float mechMotorCmd[MECH_MOTOR_COUNT] = {0.0f, 0.0f, 0.0f};

volatile long encoderCounts[TOTAL_MOTOR_COUNT] = {0, 0, 0, 0, 0, 0, 0};
uint8_t lastEncA[TOTAL_MOTOR_COUNT] = {0};
uint8_t lastEncB[TOTAL_MOTOR_COUNT] = {0};

bool relayState = false;
bool encodersEnabled = false;

uint8_t ledR = 0;
uint8_t ledG = 0;
uint8_t ledB = 0;
uint8_t ledW = 0;

unsigned long lastTelemetryMs = 0;
unsigned long lastCommandMs = 0;

// ============================================================
// Helpers
// ============================================================

float clampUnit(float value)
{
  if (value > 1.0f)
    return 1.0f;
  if (value < -1.0f)
    return -1.0f;
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

int clampByte(int value)
{
  if (value < 0)
    return 0;
  if (value > 255)
    return 255;
  return value;
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

uint16_t servoValueToTicks(float value)
{
  if (value < 0.0f)
    value = 0.0f;
  if (value > 1.0f)
    value = 1.0f;
  return SERVO_MIN_TICKS + (uint16_t)((SERVO_MAX_TICKS - SERVO_MIN_TICKS) * value);
}

// ============================================================
// Serial responses
// ============================================================

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
  Serial.print("{\"type\":\"hello\",\"board\":\"nano_every\",\"fw\":\"0.2.0\"");
  Serial.print(",\"encoders_enabled\":");
  Serial.print(encodersEnabled ? "true" : "false");
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
    if (encodersEnabled)
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
  Serial.print(relayState ? 1 : 0);
  Serial.print(",\"mode\":\"");
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
  Serial.println("],\"status\":\"ok\"}");
}

// ============================================================
// Motor / IO control
// ============================================================

void writeMotor(uint8_t pwmPin, uint8_t dirPin, float cmd)
{
  if (!pinConfigured(pwmPin) || !pinConfigured(dirPin))
    return;

  cmd = clampUnit(cmd);
  bool forward = (cmd >= 0.0f);
  int pwm = (int)(fabs(cmd) * 255.0f);

  digitalWrite(dirPin, forward ? HIGH : LOW);
  analogWrite(pwmPin, pwm);
}

void applyDriveMotors()
{
  if (!motorsEnabledByMode())
  {
    for (int i = 0; i < DRIVE_MOTOR_COUNT; i++)
    {
      writeMotor(DRIVE_PWM_PINS[i], DRIVE_DIR_PINS[i], 0.0f);
    }
    return;
  }

  for (int i = 0; i < DRIVE_MOTOR_COUNT; i++)
  {
    writeMotor(DRIVE_PWM_PINS[i], DRIVE_DIR_PINS[i], driveMotorCmd[i]);
  }
}

void applyMechMotors()
{
  if (!motorsEnabledByMode())
  {
    for (int i = 0; i < MECH_MOTOR_COUNT; i++)
    {
      writeMotor(MECH_PWM_PINS[i], MECH_DIR_PINS[i], 0.0f);
    }
    return;
  }

  for (int i = 0; i < MECH_MOTOR_COUNT; i++)
  {
    writeMotor(MECH_PWM_PINS[i], MECH_DIR_PINS[i], mechMotorCmd[i]);
  }
}

void stopAllMotors()
{
  for (int i = 0; i < DRIVE_MOTOR_COUNT; i++)
  {
    driveMotorCmd[i] = 0.0f;
    writeMotor(DRIVE_PWM_PINS[i], DRIVE_DIR_PINS[i], 0.0f);
  }

  for (int i = 0; i < MECH_MOTOR_COUNT; i++)
  {
    mechMotorCmd[i] = 0.0f;
    writeMotor(MECH_PWM_PINS[i], MECH_DIR_PINS[i], 0.0f);
  }
}

void setRelay(bool on)
{
  relayState = on;
  if (pinConfigured(RELAY_PIN))
  {
    digitalWrite(RELAY_PIN, on ? HIGH : LOW);
  }
}

void applyLed()
{
  if (pinConfigured(LED_R_PIN))
    analogWrite(LED_R_PIN, ledR);
  if (pinConfigured(LED_G_PIN))
    analogWrite(LED_G_PIN, ledG);
  if (pinConfigured(LED_B_PIN))
    analogWrite(LED_B_PIN, ledB);
  if (pinConfigured(LED_W_PIN))
    analogWrite(LED_W_PIN, ledW);
}

void setServoChannel(int channel, float value)
{
  if (channel < 0 || channel > 15)
    return;

  uint16_t ticks = servoValueToTicks(value);
  pca.setPWM(channel, 0, ticks);
}

// ============================================================
// Encoder support
// ============================================================

void initEncoders()
{
  encodersEnabled = false;

  for (int i = 0; i < TOTAL_MOTOR_COUNT; i++)
  {
    if (!pinConfigured(ENC_A_PINS[i]) || !pinConfigured(ENC_B_PINS[i]))
      continue;

    encodersEnabled = true;
    pinMode(ENC_A_PINS[i], INPUT_PULLUP);
    pinMode(ENC_B_PINS[i], INPUT_PULLUP);
    lastEncA[i] = digitalRead(ENC_A_PINS[i]);
    lastEncB[i] = digitalRead(ENC_B_PINS[i]);
  }
}

void pollEncoders()
{
  if (!encodersEnabled)
    return;

  for (int i = 0; i < TOTAL_MOTOR_COUNT; i++)
  {
    if (!pinConfigured(ENC_A_PINS[i]) || !pinConfigured(ENC_B_PINS[i]))
      continue;

    uint8_t a = digitalRead(ENC_A_PINS[i]);
    uint8_t b = digitalRead(ENC_B_PINS[i]);

    if (a != lastEncA[i] || b != lastEncB[i])
    {
      uint8_t prev = (lastEncA[i] << 1) | lastEncB[i];
      uint8_t curr = (a << 1) | b;
      uint8_t transition = (prev << 2) | curr;

      switch (transition)
      {
      case 0b0001:
      case 0b0111:
      case 0b1110:
      case 0b1000:
        encoderCounts[i]++;
        break;

      case 0b0010:
      case 0b0100:
      case 0b1101:
      case 0b1011:
        encoderCounts[i]--;
        break;

      default:
        break;
      }

      lastEncA[i] = a;
      lastEncB[i] = b;
    }
  }
}

// ============================================================
// Command handling
// ============================================================

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

  if (!motorsEnabledByMode())
  {
    stopAllMotors();
    sendStatus("drive_ignored", "mode_not_enabled");
    sendAck("drive");
    return;
  }

  applyDriveMotors();
  sendStatus("drive_applied", "ok");
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

  if (!motorsEnabledByMode())
  {
    applyMechMotors();
    sendStatus("mech_ignored", "mode_not_enabled");
    sendAck("mech");
    return;
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

void handleRelayCommand(const String &line)
{
  bool on = false;
  if (!extractBoolField(line, "on", on))
  {
    sendFault("missing_relay_bool");
    return;
  }

  setRelay(on);
  sendAck("relay");
}

void handleLedCommand(const String &line)
{
  int r = 0;
  int g = 0;
  int b = 0;
  int w = 0;

  extractIntField(line, "r", r);
  extractIntField(line, "g", g);
  extractIntField(line, "b", b);
  extractIntField(line, "w", w);

  ledR = clampByte(r);
  ledG = clampByte(g);
  ledB = clampByte(b);
  ledW = clampByte(w);
  applyLed();
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
  if (!motorsEnabledByMode())
  {
    stopAllMotors();
  }
  else
  {
    applyDriveMotors();
    applyMechMotors();
  }

  sendStatus("mode_changed", mode.c_str());
  sendAck("mode");
}

void handleResetCommand()
{
  stopAllMotors();
  setRelay(false);

  ledR = 0;
  ledG = 0;
  ledB = 0;
  ledW = 0;
  applyLed();

  for (int i = 0; i < TOTAL_MOTOR_COUNT; i++)
  {
    noInterrupts();
    encoderCounts[i] = 0;
    interrupts();
  }

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

// ============================================================
// Setup / loop
// ============================================================

void setup()
{
  Serial.begin(SERIAL_BAUD);

  for (int i = 0; i < DRIVE_MOTOR_COUNT; i++)
  {
    if (pinConfigured(DRIVE_PWM_PINS[i]))
      pinMode(DRIVE_PWM_PINS[i], OUTPUT);
    if (pinConfigured(DRIVE_DIR_PINS[i]))
      pinMode(DRIVE_DIR_PINS[i], OUTPUT);
  }

  for (int i = 0; i < MECH_MOTOR_COUNT; i++)
  {
    if (pinConfigured(MECH_PWM_PINS[i]))
      pinMode(MECH_PWM_PINS[i], OUTPUT);
    if (pinConfigured(MECH_DIR_PINS[i]))
      pinMode(MECH_DIR_PINS[i], OUTPUT);
  }

  if (pinConfigured(RELAY_PIN))
    pinMode(RELAY_PIN, OUTPUT);
  if (pinConfigured(LED_R_PIN))
    pinMode(LED_R_PIN, OUTPUT);
  if (pinConfigured(LED_G_PIN))
    pinMode(LED_G_PIN, OUTPUT);
  if (pinConfigured(LED_B_PIN))
    pinMode(LED_B_PIN, OUTPUT);
  if (pinConfigured(LED_W_PIN))
    pinMode(LED_W_PIN, OUTPUT);

  stopAllMotors();
  setRelay(false);
  applyLed();
  initEncoders();

  Wire.begin();
  pca.begin();
  pca.setPWMFreq(PCA9685_FREQ);

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
      if (serialLine.length() > 300)
      {
        serialLine = "";
        sendFault("line_too_long");
      }
    }
  }

  pollEncoders();

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
