// ╔══════════════════════════════════════════════════════════════╗
// ║   SMART AMBULANCE SYSTEM — SIDE 1: AMBULANCE TRANSMITTER    ║
// ║   Board   : ESP32 DevKit V1                                  ║
// ║   Author  : Smart Ambulance IoT Project                      ║
// ║                                                              ║
// ║   HARDWARE CONNECTED:                                        ║
// ║   • NEO-8M GPS Module       (UART2 — GPIO 16, 17)           ║
// ║   • E32 / AS32 LoRa Module  (UART1 — GPIO 19, 27)           ║
// ║   • SIM7600 4G LTE Module   (UART — GPIO 32, 33)            ║
// ║   • IR Transmitter LED      (GPIO 26)                        ║
// ║   • OLED SSD1306 Display    (I2C — GPIO 21, 22)             ║
// ║   • Siren Sensor / Switch   (GPIO 39)                        ║
// ║                                                              ║
// ║   INSTALL THESE LIBRARIES (Sketch → Manage Libraries):       ║
// ║   1. TinyGPSPlus          by Mikal Hart                      ║
// ║   2. LoRa_E32             by Renzo Mischianti               ║
// ║   3. Adafruit SSD1306     by Adafruit                        ║
// ║   4. Adafruit GFX Library by Adafruit                        ║
// ╚══════════════════════════════════════════════════════════════╝

// ── LIBRARIES ──────────────────────────────────────────────────
#include <TinyGPS++.h>
#include "LoRa_E32.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HardwareSerial.h>



String  AMBULANCE_ID  = "AMB_001";         
String  SERVER_IP     = "192.168.1.100";    
int     SERVER_PORT   = 3000;               
// ══════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ══════════════════════════════════════════════════════════════

// GPS — NEO-8M (UART2)
#define GPS_RX          16    // GPS TX  →  ESP32 GPIO 16
#define GPS_TX          17    // GPS RX  ←  ESP32 GPIO 17

// E32 / AS32 LoRa Module (UART1)
#define E32_RX          27    // E32 TXD →  ESP32 GPIO 27  (receive from module)
#define E32_TX          19    // E32 RXD ←  ESP32 GPIO 19  (send to module)
#define E32_AUX          4    // E32 AUX →  ESP32 GPIO  4  (busy pin)
#define E32_M0           5    // E32 M0  →  ESP32 GPIO  5  (mode select)
#define E32_M1          18    // E32 M1  →  ESP32 GPIO 18  (mode select)

// SIM7600 4G LTE Module
#define SIM_RX          32    // SIM TX  →  ESP32 GPIO 32
#define SIM_TX          33    // SIM RX  ←  ESP32 GPIO 33

// IR Transmitter LED
#define IR_LED          26    // IR LED Anode → GPIO 26 (via 100Ω resistor)

// Others
#define SIREN_PIN       39    // Siren sensor → GPIO 39 (INPUT ONLY pin)
#define ONBOARD_LED      2    // ESP32 onboard LED

// OLED (I2C)
#define OLED_SDA        21
#define OLED_SCL        22
#define OLED_ADDR     0x3C
#define SCREEN_W       128
#define SCREEN_H        64

// ══════════════════════════════════════════════════════════════
//  TIMING INTERVALS (milliseconds)
// ══════════════════════════════════════════════════════════════
#define GPS_SEND_INTERVAL    500    // Send GPS to cloud every 500ms
#define IR_BLAST_INTERVAL    300    // Fire IR every 300ms
#define LORA_SEND_INTERVAL  1000    // LoRa broadcast every 1s
#define DISPLAY_INTERVAL     500    // Update OLED every 500ms

// ══════════════════════════════════════════════════════════════
//  OBJECTS
// ══════════════════════════════════════════════════════════════

// Hardware serial ports
HardwareSerial gpsSerial(2);    // UART2 for GPS
HardwareSerial simSerial(1);    // UART1 for SIM7600

// GPS parser
TinyGPSPlus gps;

// E32 LoRa — uses Serial0 internally via begin()
// We pass UART0 and control pins
LoRa_E32 e32(&Serial, E32_AUX, E32_M0, E32_M1);

// OLED display
Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &Wire, -1);

// ══════════════════════════════════════════════════════════════
//  GLOBAL STATE VARIABLES
// ══════════════════════════════════════════════════════════════
float   lat           = 0.0;
float   lng           = 0.0;
bool    gpsFixed      = false;
bool    sirenOn       = false;
int     totalSent     = 0;
int     gpsAge        = 0;

unsigned long tLastGPS      = 0;
unsigned long tLastIR       = 0;
unsigned long tLastLoRa     = 0;
unsigned long tLastDisplay  = 0;


// ╔══════════════════════════════════════════════════════════════╗
// ║                          SETUP                               ║
// ╚══════════════════════════════════════════════════════════════╝
void setup() {

  // Debug serial (USB)
  Serial.begin(115200);
  delay(500);
  printBanner();

  // ── GPIO Setup ────────────────────────────────────────────────
  pinMode(IR_LED,     OUTPUT);
  pinMode(ONBOARD_LED,OUTPUT);
  pinMode(SIREN_PIN,  INPUT);      // GPIO 39 = input only, no pullup
  digitalWrite(IR_LED,      LOW);
  digitalWrite(ONBOARD_LED, LOW);

  // ── OLED ──────────────────────────────────────────────────────
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] FAILED — Check SDA/SCL wiring");
  } else {
    Serial.println("[OLED] OK");
    bootScreen();
  }

  // ── GPS Serial ────────────────────────────────────────────────
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("[GPS ] UART2 started on GPIO " + String(GPS_RX) + "/" + String(GPS_TX));

  // ── SIM7600 Serial ────────────────────────────────────────────
  simSerial.begin(115200, SERIAL_8N1, SIM_RX, SIM_TX);
  Serial.println("[GSM ] UART1 started on GPIO " + String(SIM_RX) + "/" + String(SIM_TX));

  // ── E32 LoRa ──────────────────────────────────────────────────
  // Reassign Serial0 to E32 UART pins
  Serial.end();
  Serial.begin(9600, SERIAL_8N1, E32_RX, E32_TX);
  e32.begin();
  ResponseStructContainer c = e32.getConfiguration();
  if (c.status.code == 1) {
    Serial.println("[E32 ] LoRa module found OK");
  } else {
    Serial.println("[E32 ] WARNING: LoRa module not responding — check wiring");
  }
  c.close();

  // ── GSM Init ──────────────────────────────────────────────────
  initGSM();

  Serial.println("[SYS ] === AMBULANCE UNIT READY ===");
  showOLED("SYSTEM READY", "ID: " + AMBULANCE_ID, "Waiting GPS...", "Siren: OFF");
}


// ╔══════════════════════════════════════════════════════════════╗
// ║                        MAIN LOOP                             ║
// ╚══════════════════════════════════════════════════════════════╝
void loop() {

  // 1. Always read GPS data
  readGPS();

  // 2. Check siren status
  sirenOn = (digitalRead(SIREN_PIN) == HIGH);  // HIGH = siren active

  // 3. If emergency active AND GPS is fixed — do everything
  if (sirenOn && gpsFixed) {

    digitalWrite(ONBOARD_LED, HIGH);  // LED on during emergency

    // Send GPS coordinates to cloud server
    if (millis() - tLastGPS >= GPS_SEND_INTERVAL) {
      sendGPSCloud();
      tLastGPS = millis();
    }

    // Blast IR signal to nearby traffic signals
    if (millis() - tLastIR >= IR_BLAST_INTERVAL) {
      sendIR();
      tLastIR = millis();
    }

    // Broadcast LoRa packet via E32 module
    if (millis() - tLastLoRa >= LORA_SEND_INTERVAL) {
      sendLoRa();
      tLastLoRa = millis();
    }

    // Update display with live GPS data
    if (millis() - tLastDisplay >= DISPLAY_INTERVAL) {
      showOLED(
        "!! EMERGENCY !!",
        "Lat:" + String(lat, 5),
        "Lng:" + String(lng, 5),
        "Sent: " + String(totalSent)
      );
      tLastDisplay = millis();
    }

  } else {
    // Not in emergency — idle
    digitalWrite(ONBOARD_LED, LOW);

    if (millis() - tLastDisplay >= DISPLAY_INTERVAL) {
      String gpsStr  = gpsFixed ? "GPS: FIXED " : "GPS: Searching";
      String sirenStr= sirenOn  ? "Siren: ON"   : "Siren: OFF";
      showOLED("-- IDLE --", sirenStr, gpsStr, "ID: " + AMBULANCE_ID);
      tLastDisplay = millis();
    }
  }
}


// ╔══════════════════════════════════════════════════════════════╗
// ║                      GPS FUNCTIONS                           ║
// ╚══════════════════════════════════════════════════════════════╝

void readGPS() {
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);
  }
  if (gps.location.isValid() && gps.location.age() < 2000) {
    lat      = gps.location.lat();
    lng      = gps.location.lng();
    gpsFixed = true;
  } else {
    gpsFixed = false;
  }
}


// ╔══════════════════════════════════════════════════════════════╗
// ║                   IR TRANSMITTER                             ║
// ╚══════════════════════════════════════════════════════════════╝

/*
 * Sends NEC-like IR protocol at 38kHz carrier
 * Header: 9ms burst + 4.5ms space
 * Bit 1 : 560us burst + 1690us space
 * Bit 0 : 560us burst +  560us space
 * Encodes 1 byte ambulance code (0x01 = AMB_001)
 */
void sendIR() {
  // Header
  irBurst(9000);
  delayMicroseconds(4500);

  // Send ambulance code byte (0x01)
  byte code = 0x01;
  for (int i = 7; i >= 0; i--) {
    irBurst(560);
    delayMicroseconds((code >> i & 1) ? 1690 : 560);
  }

  // Stop bit
  irBurst(560);
}

// Generate 38kHz carrier wave for given microseconds
void irBurst(int us) {
  long stop = micros() + us;
  while (micros() < stop) {
    digitalWrite(IR_LED, HIGH);
    delayMicroseconds(13);   // 13us HIGH
    digitalWrite(IR_LED, LOW);
    delayMicroseconds(13);   // 13us LOW → ~38kHz
  }
}


// ╔══════════════════════════════════════════════════════════════╗
// ║                   E32 LORA BROADCAST                         ║
// ╚══════════════════════════════════════════════════════════════╝

/*
 * Packet format:
 * AMB_001|12.97160|77.59460|EMERGENCY|a1
 * Fields: ID | Latitude | Longitude | Status | Checksum
 */
void sendLoRa() {
  if (!gpsFixed) {
    Serial.println("[E32 ] Skipping — no GPS fix");
    return;
  }

  String packet = buildPacket();

  ResponseStatus rs = e32.sendMessage(packet);

  if (rs.code == 1) {
    totalSent++;
    Serial.println("[E32 ] Sent OK: " + packet);
  } else {
    Serial.println("[E32 ] Send FAILED. Code: " + String(rs.code));
  }
}

String buildPacket() {
  return AMBULANCE_ID + "|" +
         String(lat, 5)  + "|" +
         String(lng, 5)  + "|" +
         "EMERGENCY"     + "|" +
         checksum(AMBULANCE_ID);
}

// XOR checksum of a string → hex string
String checksum(String s) {
  byte cs = 0;
  for (int i = 0; i < s.length(); i++) cs ^= s[i];
  return String(cs, HEX);
}


// ╔══════════════════════════════════════════════════════════════╗
// ║               GSM / 4G — CLOUD HTTP POST                    ║
// ╚══════════════════════════════════════════════════════════════╝

void initGSM() {
  Serial.println("[GSM ] Initializing...");
  delay(3000);  // Give SIM7600 time to power up

  sendAT("AT",             1000);   // Basic check
  sendAT("AT+CPIN?",       1000);   // SIM card status
  sendAT("AT+CSQ",         1000);   // Signal quality
  sendAT("AT+CREG?",       1000);   // Network registration
  sendAT("AT+CGATT=1",     3000);   // Attach to GPRS

  Serial.println("[GSM ] Init complete");
}

void sendGPSCloud() {
  if (!gpsFixed) return;

  // JSON payload
  String json = "{\"id\":\""     + AMBULANCE_ID       + "\","
                "\"lat\":"       + String(lat, 6)      + ","
                "\"lng\":"       + String(lng, 6)      + ","
                "\"status\":\"EMERGENCY\","
                "\"sats\":"      + String(gps.satellites.value()) + ","
                "\"speed\":"     + String(gps.speed.kmph(), 1)    + "}";

  // HTTP POST via AT commands
  sendAT("AT+HTTPINIT",                                         500);
  sendAT("AT+HTTPPARA=\"CID\",1",                              300);
  sendAT("AT+HTTPPARA=\"URL\",\"http://" + SERVER_IP + ":" +
          String(SERVER_PORT) + "/ambulance/location\"",        300);
  sendAT("AT+HTTPPARA=\"CONTENT\",\"application/json\"",       300);
  sendAT("AT+HTTPDATA=" + String(json.length()) + ",5000",     500);
  simSerial.print(json);
  delay(600);
  sendAT("AT+HTTPACTION=1",  4000);  // POST
  sendAT("AT+HTTPTERM",       300);

  Serial.println("[GSM ] Cloud POST: " + json);
}

String sendAT(String cmd, int timeout) {
  simSerial.println(cmd);
  String resp = "";
  unsigned long t = millis();
  while (millis() - t < timeout) {
    if (simSerial.available()) resp += (char)simSerial.read();
  }
  if (resp.length() > 0) {
    Serial.println("[AT  ] " + cmd + " >> " + resp);
  }
  return resp;
}


// ╔══════════════════════════════════════════════════════════════╗
// ║                   OLED DISPLAY                               ║
// ╚══════════════════════════════════════════════════════════════╝

void bootScreen() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  oled.setTextSize(1);
  oled.setCursor(15, 0);  oled.println("SMART AMBULANCE");

  oled.setTextSize(1);
  oled.setCursor(20, 12); oled.println("IoT SYSTEM v2.0");

  oled.setTextSize(1);
  oled.setCursor(10, 28); oled.println("ID: " + AMBULANCE_ID);
  oled.setCursor(5,  40); oled.println("E32 LoRa | GPS | 4G");
  oled.setCursor(25, 52); oled.println("Starting...");

  oled.display();
  delay(2500);
}

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


// ── SERIAL BANNER ─────────────────────────────────────────────
void printBanner() {
  Serial.println("╔════════════════════════════════╗");
  Serial.println("║  SMART AMBULANCE — SIDE 1      ║");
  Serial.println("║  ESP32 + E32/AS32 LoRa         ║");
  Serial.println("║  Ambulance Transmitter Unit     ║");
  Serial.println("╚════════════════════════════════╝");
}
