#include <WiFiManager.h>
#include <HTTPClient.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>

// ─── Firebase ───────────────────────────────────────────
const String FB_URL = "https://robotarmcontrol-cfc65-default-rtdb.firebaseio.com/.json";

// ─── Pin Configuration ──────────────────────────────────
const int PINS[6] = {13, 33, 32, 27, 26, 25};
// M1-Base(MG996R), M2-Shoulder(MG996R), M3-Elbow(MG996R)
// M4-WristPitch(MG90S), M5-WristRoll(SG90), M6-Gripper(SG90)

// ─── Servo Angle Limits ─────────────────────────────────
struct ServoLimit { int minA; int maxA; };
ServoLimit LIMITS[6] = {
  {0, 120},  // M1 Base        — MG996R
  {0, 120},  // M2 Shoulder    — MG996R
  {0, 120},  // M3 Elbow       — MG996R
  {0, 180},  // M4 Wrist pitch — MG90S
  {0, 180},  // M5 Wrist roll  — SG90
  {0, 180},  // M6 Gripper     — SG90 (test physical limits)
};

// ─── Servo Objects ──────────────────────────────────────
Servo motor[6];
int lastVal[6]   = {-1, -1, -1, -1, -1, -1};
int targetVal[6] = {-1, -1, -1, -1, -1, -1};

// ─── Stagger Control ────────────────────────────────────
const unsigned long STAGGER_MS   = 50;
const unsigned long FETCH_MS     = 100;
unsigned long lastFetch          = 0;
unsigned long lastServoMove      = 0;
int  servoMoveIndex              = 0;
bool movePending                 = false;

// ─── Clamp Helper ───────────────────────────────────────
int clamp(int val, int minA, int maxA) {
  if (val < minA) return minA;
  if (val > maxA) return maxA;
  return val;
}

// ────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Robotic Arm Booting ===");

  // Allocate PWM timers for ESP32Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Attach all servos
  for (int i = 0; i < 6; i++) {
    motor[i].setPeriodHertz(50);
    motor[i].attach(PINS[i], 500, 2400);
    Serial.printf("Servo M%d attached to GPIO %d\n", i + 1, PINS[i]);
  }

  // Move all servos to safe center position on boot
  Serial.println("Moving to home position...");
  for (int i = 0; i < 6; i++) {
    int home = (LIMITS[i].minA + LIMITS[i].maxA) / 2;
    motor[i].write(home);
    lastVal[i]   = home;
    targetVal[i] = home;
    delay(100); // small stagger on boot too
  }
  Serial.println("Home position set.");

  // ── WiFiManager ─────────────────────────────────────
  WiFiManager wm;

  // Uncomment to force config portal (use during testing)
  // wm.resetSettings();

  wm.setConfigPortalTimeout(180); // 3 min timeout
  wm.setConnectTimeout(20);       // 20s to connect before portal opens

  Serial.println("Starting WiFiManager...");
  Serial.println("If no saved WiFi: connect to 'RoboticArm-Setup' hotspot");
  Serial.println("Then open browser -> 192.168.4.1 -> enter WiFi details");

  bool connected = wm.autoConnect("RoboticArm-Setup", "12345678");

  if (!connected) {
    Serial.println("WiFi failed — restarting in 3 seconds...");
    delay(3000);
    ESP.restart();
  }

  Serial.println("WiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("=== Arm Ready ===\n");
}

// ────────────────────────────────────────────────────────
void scheduleMovement(int vals[6]) {
  bool anyChange = false;
  for (int i = 0; i < 6; i++) {
    int safe = clamp(vals[i], LIMITS[i].minA, LIMITS[i].maxA);
    if (safe != lastVal[i] && vals[i] >= 0) {
      targetVal[i] = safe;
      anyChange = true;
    }
  }
  if (anyChange) {
    servoMoveIndex = 0;
    movePending    = true;
  }
}

// ────────────────────────────────────────────────────────
void processStaggeredMove() {
  if (!movePending) return;

  unsigned long now = millis();
  if (now - lastServoMove < STAGGER_MS) return;

  while (servoMoveIndex < 6) {
    int i = servoMoveIndex++;
    if (targetVal[i] != lastVal[i] && targetVal[i] >= 0) {
      motor[i].write(targetVal[i]);
      lastVal[i]    = targetVal[i];
      lastServoMove = now;
      Serial.printf("  M%d -> %d deg\n", i + 1, targetVal[i]);
      return;
    }
  }
  movePending = false;
}

// ────────────────────────────────────────────────────────
void fetchFromFirebase() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost — reconnecting...");
    WiFi.reconnect();
    delay(500);
    return;
  }

  HTTPClient http;
  http.begin(FB_URL);
  http.setTimeout(3000);

  int code = http.GET();

  if (code == 200) {
    String payload = http.getString();

    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, payload);

    if (!err) {
      int vals[6];
      vals[0] = doc["m1"] | -1;
      vals[1] = doc["m2"] | -1;
      vals[2] = doc["m3"] | -1;
      vals[3] = doc["m4"] | -1;
      vals[4] = doc["m5"] | -1;
      vals[5] = doc["m6"] | -1;

      Serial.printf("Firebase: M1:%d M2:%d M3:%d M4:%d M5:%d M6:%d\n",
        vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]);

      scheduleMovement(vals);

    } else {
      Serial.print("JSON error: ");
      Serial.println(err.c_str());
    }

  } else if (code < 0) {
    Serial.printf("HTTP failed: %s\n", http.errorToString(code).c_str());
  } else {
    Serial.printf("HTTP error code: %d\n", code);
  }

  http.end();
}

// ────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  if (now - lastFetch >= FETCH_MS) {
    lastFetch = now;
    fetchFromFirebase();
  }

  processStaggeredMove();
}