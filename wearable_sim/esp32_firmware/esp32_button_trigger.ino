/*
  VisionNav ESP32 Tactical Pushbutton & Haptic Hub
  ------------------------------------------------
  Connect one leg of your pushbutton to D12 (GPIO 12) and the other leg to GND (Ground).
  When pressed, transmits "TRIGGER_DESCRIBE" over Wi-Fi UDP and USB Serial (115200 baud)!
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ── WI-FI SETTINGS (Activated for Wireless Operation!) ──
const bool USE_WIFI       = true; 
const char* WIFI_SSID     = "Sudil's Pixel 7";
const char* WIFI_PASSWORD = "1234567898";
const char* LAPTOP_IP     = "10.42.79.253"; // Laptop IP on Wi-Fi network
const int   UDP_PORT      = 9090;

WiFiUDP udp;
const int BUTTON_PIN = 12; // GPIO 12 (D12)
bool last_button_state = HIGH;
unsigned long last_debounce_time = 0;
const unsigned long debounce_delay = 50;

void setup() {
  Serial.begin(115200);
  
  // Activate ESP32 internal pull-up resistor (no external resistor needed!)
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  if (USE_WIFI) {
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 30) {
      delay(300);
      Serial.print(".");
      retries++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n✅ WiFi Connected! Wireless UDP Active!");
    } else {
      Serial.println("\n⚠️ WiFi timeout - running in USB Serial Mode!");
    }
  } else {
    Serial.println("✅ ESP32 USB Tactical Hub Ready! Waiting for button press on D12 (GPIO 12)...");
  }
}

void loop() {
  int reading = digitalRead(BUTTON_PIN);
  
  if (reading != last_button_state) {
    last_debounce_time = millis();
  }
  
  if ((millis() - last_debounce_time) > debounce_delay) {
    // Detect transition from HIGH (unpressed) to LOW (pressed to GND)
    if (reading == LOW && last_button_state == HIGH) {
      Serial.println("\n==============================================");
      Serial.println("🔘 [BUTTON PRESSED ON PIN D12 (GPIO 12)]");
      Serial.println("TRIGGER_DESCRIBE");
      
      if (USE_WIFI && WiFi.status() == WL_CONNECTED) {
        udp.beginPacket(LAPTOP_IP, UDP_PORT);
        udp.write((const uint8_t*)"TRIGGER_DESCRIBE", 16);
        udp.endPacket();
        Serial.printf("📡 [Wi-Fi UDP Broadcast Sent] -> Target: %s:%d\n", LAPTOP_IP, UDP_PORT);
      } else {
        Serial.println("🔌 [USB Serial Broadcast Sent] (Wi-Fi offline)");
      }
      Serial.println("==============================================");
      
      // 500ms delay to prevent rapid consecutive triggers
      delay(500);
    }
  }
  last_button_state = reading;
}
