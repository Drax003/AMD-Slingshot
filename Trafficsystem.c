// ╔══════════════════════════════════════════════════════════════╗
// ║   SMART AMBULANCE SYSTEM — SIDE 2: TRAFFIC SIGNAL UNIT      ║
// ║   Board   : ESP32 DevKit V1                                  ║
// ║   Author  : Smart Ambulance IoT Project                      ║
// ║                                                              ║
// ║   HARDWARE CONNECTED:                                        ║
// ║   • TSOP1738 IR Receiver    (GPIO 4)                         ║
// ║   • E32 / AS32 LoRa Module  (UART2 — GPIO 16, 17)           ║
// ║   • 4-Channel Relay Module  (GPIO 27, 26, 25, 33)           ║
// ║   • OLED SSD1306 Display    (I2C — GPIO 21, 22)             ║
// ║   • Buzzer                  (GPIO 32)                        ║
// ║   • WiFi + MQTT             (Built-in)                       ║
// ║                                                              ║
// ║   INSTALL THESE LIBRARIES (Sketch → Manage Libraries):       ║
// ║   1. LoRa_E32             by Renzo Mischianti               ║
// ║   2. Adafruit SSD1306     by Adafruit                        ║
// ║   3. Adafruit GFX Library by Adafruit                        ║
// ║   4. PubSubClient         by Nick O'Leary                    ║
// ║   5. IRremoteESP8266      by David Conran                    ║
// ╚══════════════════════════════════════════════════════════════╝

// ── LIBRARIES ──────────────────────────────────────────────────
#include "LoRa_E32.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <HardwareSerial.h>




const char* WIFI_SSID    = "YOUR_WIFI_NAME";       // ← WiFi name
const char* WIFI_PASS    = "YOUR_WIFI_PASSWORD";   // ← WiFi password
const char* MQTT_SERVER  = "192.168.1.100";         // ← Server IP
const int   MQTT_PORT    = 1883;


const char* SIGNAL_ID    = "SIGNAL_001";  // ← Change per intersection
const char* CMD_TOPIC    = "signal/001";  // ← Match with signal ID number
const char* STATUS_TOPIC = "signal/status";


// ══════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ══════════════════════════════════════════════════════════════

// E32 / AS32 LoRa Module (UART2)
#define E32_RX          17    // E32 TXD → ESP32 GPIO 17  (receive from module)
#define E32_TX          16    // E32 RXD ← ESP32 GPIO 16  (send to module)
#define E32_AUX         19    // E32 AUX → ESP32 GPIO 19  (busy pin)
#define E32_M0           5    // E32 M0  → ESP32 GPIO  5
#define E32_M1          18    // E32 M1  → ESP32 GPIO 18

// IR Receiver
#define IR_PIN           4    // TSOP1738 OUT → GPIO 4

// Relay Module (Active LOW — LOW turns relay ON)
#define RELAY_RED       27    // IN1 → Controls 🔴 RED  light
#define RELAY_GREEN     26    // IN2 → Controls 🟢 GREEN light
#define RELAY_YELLOW    25    // IN3 → Controls 🟡 YELLOW light
#define RELAY_PED       33    // IN4 → Controls 🚶 Pedestrian signal

// Buzzer
#define BUZZER          32    // Buzzer signal pin

// OLED (I2C)
#define OLED_SDA        21
#define OLED_SCL        22
#define OLED_ADDR     0x3C
#define SCREEN_W       128
#define SCREEN_H        64


// ══════════════════════════════════════════════════════════════
//  TRAFFIC LIGHT TIMING (milliseconds)
// ══════════════════════════════════════════════════════════════
#define RED_DURATION      30000   // 30 seconds RED
#define GREEN_DURATION    25000   // 25 seconds GREEN
#define YELLOW_DURATION    3000   //  3 seconds YELLOW
#define EMERGENCY_TIMEOUT 15000   // Auto-reset after 15 seconds
#define DISPLAY_REFRESH     500   // OLED refresh rate


// ══════════════════════════════════════════════════════════════
//  OBJECTS
// ══════════════════════════════════════════════════════════════

// E32 LoRa on UART2
LoRa_E32 e32(&Serial2, E32_AUX, E32_M0, E32_M1);

// IR receiver
IRrecv   irRecv(IR_PIN);
decode_results irData;

// WiFi + MQTT
WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

// OLED
Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, -1);


// ══════════════════════════════════════════════════════════════
//  STATE MACHINE
// ══════════════════════════════════════════════════════════════
enum State { NORMAL, EMERGENCY, RESETTING };
State currentState = NORMAL;

int  normalPhase   = 0;    // 0=RED  1=GREEN  2=YELLOW
String phaseNames[]= {"RED", "GREEN", "YELLOW"};

unsigned long tPhaseStart     = 0;
unsigned long tEmergencyStart = 0;
unsigned long tLastDisplay    = 0;
unsigned long tLastMQTTRetry  = 0;
unsigned long tLastBuzz       = 0;

String detectedAmbulance = "";
String detectionSource   = "";


// ╔══════════════════════════════════════════════════════════════╗
// ║                          SETUP                               ║
// ╚══════════════════════════════════════════════════════════════╝
void setup() {
  Serial.begin(115200);
  delay(500);
  printBanner();

  // ── Relay Pins — default all OFF (HIGH = relay OFF for active-low module)
  pinMode(RELAY_RED,    OUTPUT);  digitalWrite(RELAY_RED,    HIGH);
  pinMode(RELAY_GREEN,  OUTPUT);  digitalWrite(RELAY_GREEN,  HIGH);
  pinMode(RELAY_YELLOW, OUTPUT);  digitalWrite(RELAY_YELLOW, HIGH);
  pinMode(RELAY_PED,    OUTPUT);  digitalWrite(RELAY_PED,    HIGH);
  pinMode(BUZZER,       OUTPUT);  digitalWrite(BUZZER,       LOW);

  // Start safe — turn RED on immediately
  lightRed();
  Serial.println("[RLY ] Starting with RED ON (safe state)");

  // ── OLED ──────────────────────────────────────────────────────
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] FAILED — Check SDA=GPIO21 SCL=GPIO22");
  } else {
    Serial.println("[OLED] OK");
    bootScreen();
  }

  // ── IR Receiver ───────────────────────────────────────────────
  irRecv.enableIRIn();
  Serial.println("[IR  ] TSOP1738 ready on GPIO " + String(IR_PIN));

  // ── E32 LoRa (UART2) ──────────────────────────────────────────
  Serial2.begin(9600, SERIAL_8N1, E32_RX, E32_TX);
  e32.begin();
  ResponseStructContainer c = e32.getConfiguration();
  if (c.status.code == 1) {
    Serial.println("[E32 ] LoRa module OK");
  } else {
    Serial.println("[E32 ] WARNING: LoRa module not responding!");
  }
  c.close();

  // ── WiFi ──────────────────────────────────────────────────────
  connectWiFi();

  // ── MQTT ──────────────────────────────────────────────────────
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  connectMQTT();

  tPhaseStart = millis();
  Serial.println("[SYS ] === SIGNAL UNIT " + String(SIGNAL_ID) + " READY ===");
  showOLED("SIGNAL UNIT", "ID: " + String(SIGNAL_ID), "State: NORMAL-RED", "Waiting...");
}


// ╔══════════════════════════════════════════════════════════════╗
// ║                        MAIN LOOP                             ║
// ╚══════════════════════════════════════════════════════════════╝
void loop() {

  // Keep MQTT connection alive
  if (!mqtt.connected()) {
    if (millis() - tLastMQTTRetry > 5000) {
      connectMQTT();
      tLastMQTTRetry = millis();
    }
  }
  mqtt.loop();

  // Check all 3 input channels
  checkIR();      // Short range — IR from ambulance
  checkLoRa();    // Medium range — LoRa RF from ambulance
  // MQTT handled via callback above — Long range from cloud

  // Run state machine
  runStateMachine();

  // Update OLED display
  if (millis() - tLastDisplay > DISPLAY_REFRESH) {
    updateOLED();
    tLastDisplay = millis();
  }
}


// ╔══════════════════════════════════════════════════════════════╗
// ║               CHANNEL 1 — IR RECEIVER                       ║
// ╚══════════════════════════════════════════════════════════════╝
void checkIR() {
  if (!irRecv.decode(&irData)) return;   // No signal received

  uint32_t code = irData.value;
  Serial.print("[IR  ] Received: 0x");
  Serial.println(code, HEX);

  // Valid ambulance IR codes (0x01=AMB_001, 0x02=AMB_002, etc.)
  if (code == 0x01 || code == 0x02 || code == 0x03 || code == 0x04) {
    String ambID = "AMB_00" + String(code);
    triggerEmergency("IR", ambID);

  } else if (code == 0xFFFFFFFF) {
    // Repeat code = ambulance still in range, refresh timer
    if (currentState == EMERGENCY) {
      tEmergencyStart = millis();
      Serial.println("[IR  ] Ambulance still in range — timer refreshed");
    }
  }

  irRecv.resume();   // Ready for next IR signal
}


// ╔══════════════════════════════════════════════════════════════╗
// ║               CHANNEL 2 — E32 LoRa RECEIVE                  ║
// ╚══════════════════════════════════════════════════════════════╝
void checkLoRa() {
  if (!e32.available()) return;   // Nothing received

  ResponseContainer rc = e32.receiveMessage();

  if (rc.status.code != 1) {
    Serial.println("[E32 ] Receive error: " + String(rc.status.code));
    return;
  }

  String packet = rc.data;
  packet.trim();
  Serial.println("[E32 ] Received: " + packet);

  // Validate and parse packet
  if (isValidPacket(packet)) {
    String ambID = packet.substring(0, packet.indexOf('|'));
    Serial.println("[E32 ] Valid packet from: " + ambID);
    triggerEmergency("LORA", ambID);
  } else {
    Serial.println("[E32 ] Invalid packet — ignored");
  }
}

// Validate packet format: AMB_001|lat|lng|EMERGENCY|checksum
bool isValidPacket(String p) {
  // Must contain EMERGENCY
  if (p.indexOf("EMERGENCY") < 0) return false;

  // Count pipe separators — need at least 4
  int pipes = 0;
  for (char c : p) if (c == '|') pipes++;
  if (pipes < 4) return false;

  // Verify checksum
  String ambID = p.substring(0, p.indexOf('|'));
  String rxCS  = p.substring(p.lastIndexOf('|') + 1);
  rxCS.trim();

  byte cs = 0;
  for (char c : ambID) cs ^= c;
  String calcCS = String(cs, HEX);

  if (rxCS != calcCS) {
    Serial.println("[E32 ] Checksum mismatch! rx=" + rxCS + " calc=" + calcCS);
    return false;
  }
  return true;
}


// ╔══════════════════════════════════════════════════════════════╗
// ║               CHANNEL 3 — MQTT CLOUD COMMAND                ║
// ╚══════════════════════════════════════════════════════════════╝
/*
 * Expected MQTT message formats:
 * {"cmd":"GREEN","amb":"AMB_001"}  → Turn ambulance lane green
 * {"cmd":"RED"}                    → Hold cross road red
 * {"cmd":"NORMAL"}                 → Resume normal cycle
 */
void mqttCallback(char* topic, byte* payload, unsigned int len) {
  String msg = "";
  for (int i = 0; i < len; i++) msg += (char)payload[i];

  Serial.println("[MQTT] Topic: " + String(topic));
  Serial.println("[MQTT] Message: " + msg);

  if (msg.indexOf("\"cmd\":\"GREEN\"") >= 0) {
    // Extract ambulance ID from message
    String amb = "CLOUD";
    int s = msg.indexOf("\"amb\":\"");
    if (s >= 0) {
      s += 7;
      amb = msg.substring(s, msg.indexOf('"', s));
    }
    triggerEmergency("MQTT", amb);

  } else if (msg.indexOf("\"cmd\":\"RED\"") >= 0) {
    // Cross road — force stay RED
    Serial.println("[MQTT] Force RED command received");
    allLightsOff();
    lightRed();

  } else if (msg.indexOf("\"cmd\":\"NORMAL\"") >= 0) {
    // Cloud says emergency over — go back to normal
    Serial.println("[MQTT] NORMAL command — resetting");
    resetToNormal();
  }
}


// ╔══════════════════════════════════════════════════════════════╗
// ║                    STATE MACHINE                             ║
// ╚══════════════════════════════════════════════════════════════╝
void runStateMachine() {
  unsigned long now = millis();

  switch (currentState) {

    // ── EMERGENCY MODE ──────────────────────────────────────────
    case EMERGENCY:
      // Ambulance lane → GREEN
      allLightsOff();
      lightGreen();

      // Buzz every 1 second to warn pedestrians
      if (now - tLastBuzz > 1000) {
        tone(BUZZER, 1000, 200);   // 1kHz beep for 200ms
        tLastBuzz = now;
      }

      // Auto-reset after timeout if ambulance doesn't pass
      if (now - tEmergencyStart > EMERGENCY_TIMEOUT) {
        Serial.println("[SYS ] Emergency TIMEOUT — returning to normal");
        resetToNormal();
      }
      break;

    // ── NORMAL CYCLE ────────────────────────────────────────────
    case NORMAL:
      runNormalCycle();
      break;

    // ── RESETTING (yellow flash transition) ─────────────────────
    case RESETTING:
      allLightsOff();
      lightYellow();

      if (now - tPhaseStart > YELLOW_DURATION) {
        currentState = NORMAL;
        normalPhase  = 0;       // Start with RED
        tPhaseStart  = millis();
        lightRed();
        Serial.println("[SYS ] Normal cycle resumed");
      }
      break;
  }
}

void runNormalCycle() {
  unsigned long elapsed = millis() - tPhaseStart;

  switch (normalPhase) {
    case 0:  // RED phase
      allLightsOff();
      lightRed();
      if (elapsed > RED_DURATION) {
        normalPhase = 1;
        tPhaseStart = millis();
        Serial.println("[RLY ] Normal: RED → GREEN");
      }
      break;

    case 1:  // GREEN phase
      allLightsOff();
      lightGreen();
      if (elapsed > GREEN_DURATION) {
        normalPhase = 2;
        tPhaseStart = millis();
        Serial.println("[RLY ] Normal: GREEN → YELLOW");
      }
      break;

    case 2:  // YELLOW phase
      allLightsOff();
      lightYellow();
      if (elapsed > YELLOW_DURATION) {
        normalPhase = 0;
        tPhaseStart = millis();
        Serial.println("[RLY ] Normal: YELLOW → RED");
      }
      break;
  }
}


// ╔══════════════════════════════════════════════════════════════╗
// ║               EMERGENCY TRIGGER & RESET                      ║
// ╚══════════════════════════════════════════════════════════════╝
void triggerEmergency(String source, String ambID) {
  // Already in emergency — just refresh the timeout timer
  if (currentState == EMERGENCY) {
    tEmergencyStart = millis();
    Serial.println("[SYS ] Emergency timer refreshed (" + source + ")");
    return;
  }

  // New emergency — activate!
  currentState       = EMERGENCY;
  tEmergencyStart    = millis();
  detectedAmbulance  = ambID;
  detectionSource    = source;

  Serial.println("╔══════════════════════════════╗");
  Serial.println("║  *** EMERGENCY ACTIVATED ***  ║");
  Serial.println("║  Source   : " + source);
  Serial.println("║  Ambulance: " + ambID);
  Serial.println("╚══════════════════════════════╝");

  // Immediately switch to GREEN
  allLightsOff();
  lightGreen();

  // Notify cloud via MQTT
  publishStatus("EMERGENCY", source, ambID);
}

void resetToNormal() {
  currentState      = RESETTING;
  tPhaseStart       = millis();
  detectedAmbulance = "";
  detectionSource   = "";

  noTone(BUZZER);
  digitalWrite(BUZZER, LOW);

  publishStatus("NORMAL", "", "");
  Serial.println("[SYS ] Resetting — YELLOW flash then NORMAL");
}


// ╔══════════════════════════════════════════════════════════════╗
// ║              RELAY CONTROL HELPERS                           ║
// ║  Active LOW relay: LOW = relay ON = light ON                 ║
// ╚══════════════════════════════════════════════════════════════╝
void allLightsOff() {
  digitalWrite(RELAY_RED,    HIGH);  // OFF
  digitalWrite(RELAY_GREEN,  HIGH);  // OFF
  digitalWrite(RELAY_YELLOW, HIGH);  // OFF
  digitalWrite(RELAY_PED,    HIGH);  // OFF
}

void lightRed()    { allLightsOff(); digitalWrite(RELAY_RED,    LOW); }   // RED ON
void lightGreen()  { allLightsOff(); digitalWrite(RELAY_GREEN,  LOW); }   // GREEN ON
void lightYellow() { allLightsOff(); digitalWrite(RELAY_YELLOW, LOW); }   // YELLOW ON
void lightPed()    {                 digitalWrite(RELAY_PED,    LOW); }   // PED ON (additive)


// ╔══════════════════════════════════════════════════════════════╗
// ║               MQTT — STATUS PUBLISHER                        ║
// ╚══════════════════════════════════════════════════════════════╝
void publishStatus(String status, String src, String amb) {
  if (!mqtt.connected()) return;

  String msg = "{\"signal\":\"" + String(SIGNAL_ID) + "\","
               "\"status\":\""  + status             + "\","
               "\"source\":\""  + src                + "\","
               "\"amb\":\""     + amb                + "\"}";

  mqtt.publish(STATUS_TOPIC, msg.c_str());
  Serial.println("[MQTT] Published: " + msg);
}


// ╔══════════════════════════════════════════════════════════════╗
// ║               WiFi + MQTT CONNECTION                         ║
// ╚══════════════════════════════════════════════════════════════╝
void connectWiFi() {
  Serial.print("[WiFi] Connecting to " + String(WIFI_SSID));
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" CONNECTED!");
    Serial.println("[WiFi] IP: " + WiFi.localIP().toString());
  } else {
    Serial.println(" FAILED — running in offline mode (IR + LoRa only)");
  }
}

void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;

  String clientID = "SignalUnit_" + String(SIGNAL_ID);
  Serial.print("[MQTT] Connecting as " + clientID + "...");

  if (mqtt.connect(clientID.c_str())) {
    Serial.println(" CONNECTED!");
    mqtt.subscribe(CMD_TOPIC);       // Own command topic e.g. signal/001
    mqtt.subscribe("signal/ALL");    // Broadcast topic for all signals
    Serial.println("[MQTT] Subscribed to: " + String(CMD_TOPIC) + " + signal/ALL");
  } else {
    Serial.println(" FAILED (state=" + String(mqtt.state()) + ")");
    Serial.println("[MQTT] Will retry in 5s...");
  }
}


// ╔══════════════════════════════════════════════════════════════╗
// ║                   OLED DISPLAY                               ║
// ╚══════════════════════════════════════════════════════════════╝
void bootScreen() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);

  oled.setCursor(10,  0); oled.println("SMART SIGNAL UNIT");
  oled.setCursor( 5, 12); oled.println("ID: " + String(SIGNAL_ID));
  oled.setCursor( 5, 28); oled.println("IR + LoRa + MQTT");
  oled.setCursor( 5, 42); oled.println("4-Relay Controller");
  oled.setCursor(25, 54); oled.println("Starting...");

  oled.display();
  delay(2500);
}

void updateOLED() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);

  // Line 1 — Signal ID
  oled.setCursor(0, 0);
  oled.println("ID: " + String(SIGNAL_ID));

  // Line 2 — Current state
  oled.setCursor(0, 13);
  if (currentState == EMERGENCY) {
    oled.println("> EMERGENCY GREEN <");
  } else if (currentState == RESETTING) {
    oled.println("Resetting...");
  } else {
    oled.println("Normal: " + phaseNames[normalPhase]);
  }

  // Line 3 — Detected ambulance
  oled.setCursor(0, 26);
  if (detectedAmbulance != "") {
    oled.println(detectedAmbulance + " (" + detectionSource + ")");
  } else {
    oled.println("No ambulance");
  }

  // Line 4 — WiFi + MQTT status
  oled.setCursor(0, 39);
  String ws = (WiFi.status() == WL_CONNECTED) ? "W:OK" : "W:NO";
  String ms = mqtt.connected() ? " M:OK" : " M:NO";
  oled.println(ws + ms);

  // Line 5 — Emergency countdown
  oled.setCursor(0, 52);
  if (currentState == EMERGENCY) {
    long rem = ((long)EMERGENCY_TIMEOUT - (long)(millis() - tEmergencyStart)) / 1000;
    if (rem < 0) rem = 0;
    oled.println("Auto-reset: " + String(rem) + "s");
  } else {
    long rem = 0;
    String dur = "";
    if (normalPhase == 0)      { rem = RED_DURATION;    dur = "R:"; }
    else if (normalPhase == 1) { rem = GREEN_DURATION;  dur = "G:"; }
    else                       { rem = YELLOW_DURATION; dur = "Y:"; }
    long remaining = ((long)rem - (long)(millis() - tPhaseStart)) / 1000;
    if (remaining < 0) remaining = 0;
    oled.println(dur + String(remaining) + "s");
  }

  oled.display();
}


// ── SERIAL BANNER ─────────────────────────────────────────────
void showOLED(String l1, String l2, String l3, String l4) {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0,  0); oled.println(l1);
  oled.setCursor(0, 16); oled.println(l2);
  oled.setCursor(0, 32); oled.println(l3);
  oled.setCursor(0, 48); oled.println(l4);
  oled.display();
}

void printBanner() {
  Serial.println("╔════════════════════════════════╗");
  Serial.println("║  SMART AMBULANCE — SIDE 2      ║");
  Serial.println("║  ESP32 + E32/AS32 LoRa         ║");
  Serial.println("║  Traffic Signal Receiver Unit   ║");
  Serial.println("╚════════════════════════════════╝");
}
