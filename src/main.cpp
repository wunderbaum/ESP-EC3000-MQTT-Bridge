#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <U8g2lib.h>
#include <ESPAsyncWebServer.h>


#include "wifidata.h"
#include "whitelist.h"
#include "pins.h"
#include "fonts.h"

// LED Eigenschaften
#define LED_VALID_DURATION 3      // 3ms for valid packets
#define LED_INVALID_DURATION 60   // 10ms for invalid packets
#define LED_VALID_BRIGHTNESS 250  // ~2% brightness for valid (0 = brightest, 255 = off)
#define LED_INVALID_BRIGHTNESS 0  // Full brightness for invalid
#define LED_PWM_FREQ 5000         // 5kHz PWM frequency
#define LED_PWM_RESOLUTION 8      // 8-bit resolution (0-255)

#define FREQUENCY_KHZ  868300
#define DATA_RATE      20000
#define REG_OPMODE     0x01
#define REG_PAYLOADLENGTH 0x38
#define REG_IRQFLAGS2  0x28

#define PAYLOAD_SIZE 47
#define FRAME_LENGTH 38
#define EPSILON 0.00001

// Behandlung des "BO0" Knopfes
#define LONG_PRESS_DURATION 500   // 500ms for font change

// Anzeigelänge des Popups beim Fontwechsel
#define FONT_POPUP_DURATION 1250  // 1250ms popup with the current font array number

// Global variables for LED timing
unsigned long ledValidOnTime = 0;
unsigned long ledInvalidOnTime = 0;
bool ledValidIsOn = false;
bool ledInvalidIsOn = false;

// Fontcounter popup
unsigned long fontPopupStart = 0;  // Popup timer
bool showFontPopup = false;        // Popup state

// Display initialization
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 6, 5);
// Display buffer dimensions
const unsigned int BufferWidth = 132;
const unsigned int BufferHeight = 64;
const unsigned int ScreenWidth = 72;
const unsigned int ScreenHeight = 40;
const unsigned int xOffset = (BufferWidth - ScreenWidth) / 2;
const unsigned int yOffset = (BufferHeight - ScreenHeight) / 2;

// Define constants and variables
const int buttonPin = 9;                  // GPIO pin for the button
const unsigned long debounceDelay = 250;  // Debounce delay in milliseconds

// Logging toggle
bool logOnlyFailed = true; // true = log only failed, false = log all

unsigned long lastStatusPublish = 0;


// Payload buffer
uint8_t m_payload[64];
uint8_t m_payloadPointer = 0;
bool m_payloadReady = false;

// Reconnect-counter
uint8_t reconcount = 0;

AsyncWebServer webserver(80);

// Frame structure
struct Frame {
  uint16_t ID;
  uint32_t TotalSeconds;
  uint32_t OnSeconds;
  double Consumption;
  float Power;
  float MaximumPower;
  uint16_t NumberOfResets;
  bool IsOn;
  uint8_t Reception;
  uint16_t CRC;
};

// Reset and Consumption tracking (max 32 devices)
#define MAX_IDS 32
struct ResetTracker {
  uint16_t ID;
  uint16_t LastResets;
  double LastConsumption;
  bool Initialized;
  unsigned long LastSeen; // Timestamp in milliseconds
} resetTrackers[MAX_IDS] = {0};

struct Tracker {
  uint16_t ID;
  uint16_t LastResets;
  double LastConsumption;
  bool Initialized;
  unsigned long LastSeen;  // Timestamp in milliseconds
} trackers[MAX_IDS] = { 0 };

// WiFi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);

// Display page management
struct DisplayPage {
  uint16_t ID;
  float Power;
  double Consumption;
  unsigned long lastUpdate;
  bool active;
};

DisplayPage displayPages[MAX_IDS];
uint8_t currentPage = 0;
uint8_t totalPages = 0;

// Fontswitching
uint8_t currentFontIndex = 0;        // Start with first font
unsigned long buttonPressStart = 0;  // Track press start time
bool buttonIsPressed = false;        // Track press state

// Variables to track button state and timing
bool lastButtonState = HIGH;         // Previous state of the button (HIGH due to INPUT_PULLUP)
bool currentButtonState = HIGH;      // Current state of the button
unsigned long lastDebounceTime = 0;  // Last time the button state changed

void debugLog(const String& msg) {
  // TODO: Make it actually worship the login bool more as it currently is a bit messy in the console.... :/
  Serial.println(msg);
  if (client.connected()) {
    client.publish("EC3000/debug", msg.c_str(), true);
  }
}

// Track which IDs have had discovery messages sent
bool discoverySent[MAX_IDS] = {0};

void WriteReg(uint8_t addr, uint8_t value) {
  digitalWrite(RFM69_CS, LOW);
  SPI.transfer(addr | 0x80);
  SPI.transfer(value);
  digitalWrite(RFM69_CS, HIGH);
}

uint8_t ReadReg(uint8_t addr) {
  digitalWrite(RFM69_CS, LOW);
  SPI.transfer(addr & 0x7F);
  uint8_t result = SPI.transfer(0);
  digitalWrite(RFM69_CS, HIGH);
  return result;
}

uint8_t GetByteFromFifo() {
  return ReadReg(0x00);
}

byte Count1bits(uint32_t v) {
  byte c;
  for (c = 0; v; c++) {
    v &= v - 1;
  }
  return c;
}

// EC3000 decoding functions (unchanged)
void DescramblePayload(byte* payload) {
  byte ctr = PAYLOAD_SIZE;
  uint8_t inpbyte, outbyte = 0;
  uint32_t scramshift = 0xF185D3AC;
  while (ctr--) {
    inpbyte = *payload;
    for (byte bit = 0; bit < 8; ++bit) {
      byte ibit = (inpbyte & 0x80) >> 7;
      byte obit = ibit ^ (Count1bits(scramshift & 0x31801) & 0x01);
      scramshift = scramshift << 1 | ibit;
      inpbyte <<= 1;
      outbyte = outbyte << 1 | obit;
    }
    *payload++ = outbyte ^ 0xFF;
  }
}

uint16_t UpdateCRC(uint16_t crc, byte data) {
  data ^= crc & 0xFF;
  data ^= data << 4;
  return ((((uint16_t)data << 8) | (crc >> 8)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));
}

void Del0BitsAndRevBits(byte * payload, byte blen) {
  uint8_t sval, dval, bit;
  uint8_t si, sbi, di, dbi, n1bits;

  di = dval = dbi = n1bits = 0;
  for (si = 0; si < blen; ++si) {
    sval = payload[si];
    for (sbi = 0; sbi < 8; ++sbi) {
      bit = sval & 0x80;
      sval <<= 1;
      if (n1bits >= 5 && bit == 0) {
        n1bits = 0;
        continue;
      }
      if (bit) n1bits++;
      else n1bits = 0;
      dval = dval >> 1 | bit;
      dbi++;
      if (dbi == 8) {
        payload[di++] = dval;
        dval = dbi = 0;
      }
    }
  }
  if (dbi) payload[di] = dval >> (uint8_t)(8 - dbi);
}

void ShiftLeft(byte * payload, byte blen, byte shift) {
  uint8_t offs, bits, slen, i;
  uint16_t wbuf;

  if (shift == 0) return;
  offs = shift / 8;
  bits = 8 - shift % 8;
  slen = blen - offs - 1;
  wbuf = payload[offs];
  for (i = 0; i < slen; ++i) {
    wbuf = wbuf << 8 | payload[i + offs + 1];
    payload[i] = wbuf >> bits;
  }
  payload[slen] = wbuf << (uint8_t)(8 - bits);
}

uint16_t ShiftReverse(byte *payload) {
  byte rblen = 47;
  uint16_t i, ec3klen;
  uint16_t crc;

  ec3klen = rblen - 1;
  Del0BitsAndRevBits(payload + 1, ec3klen);
  crc = 0xFFFF;
  if (ec3klen >= FRAME_LENGTH) {
    for (i = 0; i < FRAME_LENGTH; ++i) {
      crc = UpdateCRC(crc, payload[1 + i]);
    }
  }
  ShiftLeft(payload, rblen, 4 + 8);
  return crc;
}

void DecodeFrame(byte *payload, struct Frame *frame) {
  DescramblePayload(payload);
  frame->CRC = ShiftReverse(payload);
  
  frame->ID = (payload[0] << 8) | payload[1];
  frame->TotalSeconds = (uint32_t)payload[29] << 20 | (uint32_t)payload[30] << 12 | (uint32_t)payload[2] << 8 | (uint32_t)payload[3];
  frame->OnSeconds = (uint32_t)payload[35] << 20 | (uint32_t)payload[36] << 12 | (uint32_t)payload[6] << 8 | (uint32_t)payload[7];
  
  uint64_t cons = 0;
  cons |= payload[14];
  cons |= (uint16_t)payload[13] << 8;
  cons |= (uint16_t)payload[12] << 16;
  cons |= (uint32_t)(payload[11] & 0x0F) << 24;
  cons |= (uint64_t)payload[34] << 28;
  cons |= (uint64_t)payload[33] << 36;

  frame->Consumption = round(cons / 3600000.0 * 1000.0) / 1000.0; // force round to 3 decimal places (123.456)
  frame->Power = round(((uint16_t)payload[15] << 8 | payload[16]) / 10.0 * 10) / 10.0; // force round to 1 decimal place (123.4)
  frame->MaximumPower = round(((uint16_t)payload[17] << 8 | payload[18]) / 10.0 * 10) / 10.0 ; // force round to 1 decimal place (432.1)

  frame->NumberOfResets = (payload[36] << 4) | (payload[37] >> 4);
  frame->IsOn = payload[37] & 0x08;
  frame->Reception = 0;
}

// New function to publish Home Assistant discovery messages
void publishDiscoveryMessages(uint16_t id) {
  char device_id[16];
  snprintf(device_id, sizeof(device_id), "ec3000_%04X", id);
  char state_topic[32];
  snprintf(state_topic, sizeof(state_topic), "EC3000/%04X", id);

  // Device info shared across all entities
  String device_info = "{\"identifiers\": [\"" + String(device_id) + "\"],"
                       "\"name\": \"EC3000 Device " + String(id, HEX) + "\","
                       "\"manufacturer\": \"DIY\","
                       "\"model\": \"EC3000 Energy Monitor\"}";
  // Power sensor
  char power_topic[64];
  
  snprintf(power_topic, sizeof(power_topic), "homeassistant/sensor/%s/power/config", device_id);
  String power_payload = "{\"name\": \"Power\","
                 "\"state_topic\": \"" + String(state_topic) + "\","
                 "\"unique_id\": \"" + String(id, HEX) + "_power\","
                 "\"unit_of_measurement\": \"W\","
                 "\"value_template\": \"{{ value_json.Power }}\","
                 "\"device_class\": \"power\","
                 "\"device\": " + device_info + "}";
 // String power_payload = "{\"name\": \"Test\", \"state_topic\": \"EC3000/Test\"}";
  // client.publish(power_topic, power_payload.c_str(), true);
  if (!client.publish(power_topic, power_payload.c_str(), true)) {
    Serial.println("Fehler beim Senden der MQTT-Nachricht.");
    Serial.println(power_topic);
    Serial.println(power_payload);
    Serial.print("Payload Länge: ");
    Serial.println(power_payload.length());
  }

  // Consumption sensor
  char consumption_topic[64];
  snprintf(consumption_topic, sizeof(consumption_topic), "homeassistant/sensor/%s/energy/config", device_id);
  String consumption_payload = "{\"name\": \"Energy\","
                               "\"state_topic\": \"" + String(state_topic) + "\","
                               "\"unique_id\": \"" + String(id, HEX) + "_energy\","
                               "\"unit_of_measurement\": \"kWh\","
                               "\"value_template\": \"{{ value_json.Consumption }}\","
                               "\"device_class\": \"energy\","
                               "\"state_class\": \"total_increasing\","
                               "\"device\": " + device_info + "}";
  client.publish(consumption_topic, consumption_payload.c_str(), true);

  // Total Seconds sensor
  char total_seconds_topic[64];
  snprintf(total_seconds_topic, sizeof(total_seconds_topic), "homeassistant/sensor/%s/total_seconds/config", device_id);
  String total_seconds_payload = "{\"name\": \"Total Seconds\","
                                 "\"state_topic\": \"" + String(state_topic) + "\","
                                 "\"unique_id\": \"" + String(id, HEX) + "_total_seconds\","
                                 "\"unit_of_measurement\": \"s\","
                                 "\"value_template\": \"{{ value_json.TotalSeconds }}\","
                                 "\"device_class\": \"duration\","
                                 "\"state_class\": \"total_increasing\","
                                 "\"device\": " + device_info + "}";
  client.publish(total_seconds_topic, total_seconds_payload.c_str(), true);

  // On Seconds sensor
  char on_seconds_topic[64];
  snprintf(on_seconds_topic, sizeof(on_seconds_topic), "homeassistant/sensor/%s/on_seconds/config", device_id);
  String on_seconds_payload = "{\"name\": \"On Seconds\","
                              "\"state_topic\": \"" + String(state_topic) + "\","
                              "\"unique_id\": \"" + String(id, HEX) + "_on_seconds\","
                              "\"unit_of_measurement\": \"s\","
                              "\"value_template\": \"{{ value_json.OnSeconds }}\","
                              "\"device_class\": \"duration\","
                              "\"state_class\": \"total_increasing\","
                              "\"device\": " + device_info + "}";
  client.publish(on_seconds_topic, on_seconds_payload.c_str(), true);

  // Maximum Power sensor
  char max_power_topic[64];
  snprintf(max_power_topic, sizeof(max_power_topic), "homeassistant/sensor/%s/max_power/config", device_id);
  String max_power_payload = "{\"name\": \"Maximum Power\","
                             "\"state_topic\": \"" + String(state_topic) + "\","
                             "\"unique_id\": \"" + String(id, HEX) + "_max_power\","
                             "\"unit_of_measurement\": \"W\","
                             "\"value_template\": \"{{ value_json.MaximumPower }}\","
                             "\"device\": " + device_info + "}";
  client.publish(max_power_topic, max_power_payload.c_str(), true);

  // Number of Resets sensor
  char resets_topic[64];
  snprintf(resets_topic, sizeof(resets_topic), "homeassistant/sensor/%s/resets/config", device_id);
  String resets_payload = "{\"name\": \"Number of Resets\","
                          "\"state_topic\": \"" + String(state_topic) + "\","
                          "\"unique_id\": \"" + String(id, HEX) + "_resets\","
                          "\"value_template\": \"{{ value_json.NumberOfResets }}\","
                          "\"device\": " + device_info + "}";
  client.publish(resets_topic, resets_payload.c_str(), true);

  // Is On binary sensor
  char is_on_topic[64];
  snprintf(is_on_topic, sizeof(is_on_topic), "homeassistant/binary_sensor/%s/is_on/config", device_id);
  String is_on_payload = "{\"name\": \"Is On\","
                         "\"state_topic\": \"" + String(state_topic) + "\","
                         "\"unique_id\": \"" + String(id, HEX) + "_is_on\","
                         "\"value_template\": \"{{ value_json.IsOn }}\","
                         "\"payload_on\": \"1\","
                         "\"payload_off\": \"0\","
                         "\"device\": " + device_info + "}";
  client.publish(is_on_topic, is_on_payload.c_str(), true);

  // RSSI sensor
  char rssi_topic[64];
  snprintf(rssi_topic, sizeof(rssi_topic), "homeassistant/sensor/%s/rssi/config", device_id);
  String rssi_payload = "{\"name\": \"RSSI\","
                        "\"state_topic\": \"" + String(state_topic) + "\","
                        "\"unique_id\": \"" + String(id, HEX) + "_rssi\","
                        "\"unit_of_measurement\": \"dBm\","
                        "\"value_template\": \"{{ value_json.RSSI }}\","
                        "\"device\": " + device_info + "}";
  client.publish(rssi_topic, rssi_payload.c_str(), true);

  Serial.print("Published discovery messages for ID: 0x");
  Serial.println(id, HEX);
 
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    String clientId = "ESP32C3-EC3000-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("re-connected");
      reconcount++;
      // Publish discovery messages for known IDs
      for (int i = 0; i < whitelistSize; i++) {
        uint16_t id = strtol(whitelist[i], nullptr, 16);  // Umwandlung von Hex-String zu uint16_t
        publishDiscoveryMessages(id);
      }
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

void printTimeBreakdown(uint32_t seconds) {
  uint32_t minutes = seconds / 60;
  uint32_t hours = minutes / 60;
  uint32_t days = hours / 24;
  uint32_t years = days / 365;

  minutes %= 60;
  hours %= 24;
  days %= 365;

  Serial.print(" (");
  if (years) Serial.print(years), Serial.print("y ");
  if (days || years) Serial.print(days), Serial.print("d ");
  Serial.print(hours), Serial.print("h ");
  Serial.print(minutes), Serial.print("m)");
}

bool updateResetTracker(uint16_t id, uint16_t resets, double consumption, uint16_t* lastResets, double* lastConsumption) {
  unsigned long now = millis();
  for (int i = 0; i < MAX_IDS; i++) {
    if (resetTrackers[i].Initialized && resetTrackers[i].ID == id) {
      *lastResets = resetTrackers[i].LastResets;
      *lastConsumption = resetTrackers[i].LastConsumption;
      if (resets == resetTrackers[i].LastResets || resets == resetTrackers[i].LastResets + 1) {
        resetTrackers[i].LastResets = resets;
        resetTrackers[i].LastConsumption = consumption;
        resetTrackers[i].LastSeen = now;
        Serial.print("Tracker Update: ID=0x"); Serial.print(id, HEX);
        Serial.print(", Resets="); Serial.print(resets);
        Serial.print(", Cons="); Serial.println(consumption, 3);
        return true;
      }
      Serial.print("Tracker Failed (Resets): ID=0x"); Serial.print(id, HEX);
      Serial.print(", Expected="); Serial.print(*lastResets + 1);
      Serial.print(", Got="); Serial.println(resets);
      return false;
    }
    if (!resetTrackers[i].Initialized) {
      resetTrackers[i].ID = id;
      resetTrackers[i].LastResets = resets;
      resetTrackers[i].LastConsumption = consumption;
      resetTrackers[i].Initialized = true;
      resetTrackers[i].LastSeen = now;
      *lastResets = resets;
      *lastConsumption = consumption;
      Serial.print("Tracker Init: ID=0x"); Serial.print(id, HEX);
      Serial.print(", Resets="); Serial.print(resets);
      Serial.print(", Cons="); Serial.println(consumption, 3);
      return true;
    }
  }
  Serial.println("Too many IDs! Increase MAX_IDS.");
  return false;
}

bool isWhitelisted(String id) {
  for (int i = 0; i < whitelistSize; i++) {
    if (id.equalsIgnoreCase(whitelist[i])) {
      return true;
    }
  }
  return false;
}

bool checkConsumption(uint16_t id, double consumption, double* lastConsumption) {
  unsigned long now = millis();

  for (int i = 0; i < MAX_IDS; i++) {
    if (trackers[i].Initialized && trackers[i].ID == id) {
      *lastConsumption = trackers[i].LastConsumption;

      double delta = consumption - *lastConsumption;

      if (!trackers[i].LastConsumption) {
        trackers[i].LastConsumption = consumption;
        trackers[i].LastSeen = now;
//        Serial.println("Cons Init: ID=0x" + String(id, HEX) + ", Cons=" + String(consumption, 3));
        return true;
      }

      if (fabs(delta) < EPSILON) {
//        Serial.println("Cons Same: ID=0x" + String(id, HEX) + ", Last=" + String(*lastConsumption, 3) + ", New=" + String(consumption, 3) + ", Delta=" + String(delta, 3));
        trackers[i].LastSeen = now;
        trackers[i].LastConsumption = consumption;
        return true;
      }

      if (delta < 0.0) {
//        Serial.println("Cons Invalid: ID=0x" + String(id, HEX) + ", Last=" + String(*lastConsumption, 3) + ", New=" + String(consumption, 3) + ", Delta=" + String(delta, 3));
        trackers[i].LastSeen = now;
        return false;
      }

      if (delta > 0.025) {
//        Serial.println("Cons Failed: ID=0x" + String(id, HEX) + ", Last=" + String(*lastConsumption, 3) + ", New=" + String(consumption, 3) + ", Delta=" + String(delta, 3));
        trackers[i].LastSeen = now;
        return false;
      }

      trackers[i].LastConsumption = consumption;
      trackers[i].LastSeen = now;
//        Serial.println("Cons OK: ID=0x" + String(id, HEX) + ", Cons=" + String(consumption, 3));
      return true;
    }
  }

  for (int i = 0; i < MAX_IDS; i++) {
    if (!trackers[i].Initialized) {
      trackers[i].ID = id;
      trackers[i].LastResets = 0;
      trackers[i].LastConsumption = consumption;
      trackers[i].Initialized = true;
      trackers[i].LastSeen = now;
      *lastConsumption = consumption;
//        Serial.println("Cons Fallback Init: ID=0x" + String(id, HEX) + ", Cons=" + String(consumption, 3));
      return true;
    }
  }

  return false;
}

void updateDisplayPage(uint16_t id, float power, double consumption) {
  unsigned long now = millis();

  // Find existing page or create new
  for (int i = 0; i < MAX_IDS; i++) {
    if (displayPages[i].active && displayPages[i].ID == id) {
      displayPages[i].Power = power;
      displayPages[i].Consumption = consumption;
      displayPages[i].lastUpdate = now;
      return;
    }
  }

  // Create new page if space available
  for (int i = 0; i < MAX_IDS; i++) {
    if (!displayPages[i].active) {
      displayPages[i].ID = id;
      displayPages[i].Power = power;
      displayPages[i].Consumption = consumption;
      displayPages[i].lastUpdate = now;
      displayPages[i].active = true;
      totalPages++;
      return;
    }
  }
}

void drawDisplay() {
  u8g2.clearBuffer();
  if (showFontPopup && millis() - fontPopupStart < FONT_POPUP_DURATION) {
    // Popup: show font index only
    u8g2.setFont(u8g2_font_fub30_tn);  // Large numeric font (~30px)
    char popup[3];
    snprintf(popup, sizeof(popup), "%d", currentFontIndex);
    int width = u8g2.getStrWidth(popup);
    int height = u8g2.getFontAscent() - u8g2.getFontDescent();
    int x = xOffset + (ScreenWidth - width) / 2;    // Center horizontally
    int y = yOffset + (ScreenHeight - height) / 2;  // Center vertically
    u8g2.setCursor(x, y);
    u8g2.print(popup);
    Serial.println("Drawing popup: " + String(popup));
  } else {
    // Normal display
    showFontPopup = false;  // Hide popup after 3s
    u8g2.setFont(fonts[currentFontIndex]);
    if (totalPages == 0) {
      u8g2.setCursor(xOffset, yOffset);
      u8g2.print("...");  // Show that we are still Waiting for the very first packets ... only seen at system startup until the first packet comes in
    } else {
      // Find the actual page index
      int displayIndex = 0;
      for (int i = 0, count = 0; i < MAX_IDS; i++) {
        if (displayPages[i].active) {
          if (count == currentPage) {
            displayIndex = i;
            break;
          }
          count++;
        }
      }
      // Calculate loading bar progress (5 seconds total)
      unsigned long elapsed = millis() - displayPages[displayIndex].lastUpdate;
      int barWidth = (elapsed * ScreenWidth) / 5000;
      if (barWidth > ScreenWidth) {
        barWidth = ScreenWidth;
      }

      // Draw content with current font
      u8g2.setFont(fonts[currentFontIndex]);
      char buffer[20];

      // ID
      snprintf(buffer, sizeof(buffer), "%04X ID", displayPages[displayIndex].ID);
      u8g2.setCursor(xOffset, yOffset);
      u8g2.print(buffer);

      // Loading bar
      u8g2.drawHLine(xOffset, yOffset + 13, barWidth);

      // Power
      snprintf(buffer, sizeof(buffer), "%.1f W", displayPages[displayIndex].Power);
      u8g2.setCursor(xOffset, yOffset + 14);
      u8g2.print(buffer);

      // Consumption
      snprintf(buffer, sizeof(buffer), "%.3f KWH", displayPages[displayIndex].Consumption);
      u8g2.setCursor(xOffset, yOffset + 28);
      u8g2.print(buffer);
    }
  }
  u8g2.sendBuffer();
}

void handleButton() {
  int reading = digitalRead(buttonPin);
  // Serial.print("Button reading: ");
  // Serial.println(reading); // Show the button state (must be "1" !!!)

  if (reading == LOW && lastButtonState == HIGH) {
    // Button just pressed
    buttonPressStart = millis();
    buttonIsPressed = true;
    Serial.println("Button pressed - starting timer");
  } else if (reading == HIGH && lastButtonState == LOW) {
    // Button released
    unsigned long pressDuration = millis() - buttonPressStart;
    buttonIsPressed = false;
    if (pressDuration < LONG_PRESS_DURATION) {
      // Short press: cycle page
      if (totalPages > 0) {
        currentPage = (currentPage + 1) % totalPages;
        Serial.println("Short press - Cycling to page: " + String(currentPage));
        drawDisplay();
      } else {
        Serial.println("Short press - No pages to cycle");
      }
    }
    Serial.println("Button released - duration: " + String(pressDuration) + "ms");
  } else if (buttonIsPressed && reading == LOW) {
    // Button held
    unsigned long pressDuration = millis() - buttonPressStart;
    if (pressDuration >= LONG_PRESS_DURATION) {
      // Long press: cycle font
      currentFontIndex = (currentFontIndex + 1) % numFonts;
      fontPopupStart = millis();
      showFontPopup = true;
      Serial.println("Long press - Cycling to font index: " + String(currentFontIndex));
      drawDisplay();
      buttonIsPressed = false;  // Prevent repeat triggers
    }
  }
  lastButtonState = reading;
}

void cleanStaleIDs() {
  unsigned long now = millis();
  for (int i = 0; i < MAX_IDS; i++) {
    if (resetTrackers[i].Initialized && (now - resetTrackers[i].LastSeen > 66000)) {
      Serial.print("Removing stale ID: 0x"); Serial.println(resetTrackers[i].ID, HEX);
      client.publish("EC3000/debug", "Removing stale ID");
      resetTrackers[i].Initialized = false;
      discoverySent[i] = false; // Allow rediscovery if ID reappears
    }
  }
}

void printFrame(struct Frame* frame, float rssi) {
  Serial.print("RSSI: "); Serial.print(rssi); Serial.println(" dBm");
  Serial.print("ID: 0x"); Serial.println(frame->ID, HEX);
  Serial.print("Total Seconds: "); Serial.print(frame->TotalSeconds);
  printTimeBreakdown(frame->TotalSeconds);
  Serial.println();
  Serial.print("On Seconds: "); Serial.print(frame->OnSeconds);
  printTimeBreakdown(frame->OnSeconds);
  Serial.println();
  Serial.print("Consumption: "); Serial.print(frame->Consumption); Serial.println(" kWh");
  Serial.print("Power: "); Serial.print(frame->Power); Serial.println(" W");
  Serial.print("Max Power: "); Serial.print(frame->MaximumPower); Serial.println(" W");
  Serial.print("Resets: "); Serial.println(frame->NumberOfResets);
  Serial.print("Is On: "); Serial.println(frame->IsOn ? "Yes" : "No");
  Serial.print("CRC: 0x"); Serial.println(frame->CRC, HEX);
}

void publishSystemStatus(uint16_t id) {
  char topic[32];
  snprintf(topic, sizeof(topic), "EC3000/system", id);

  // Systemdaten sammeln
  unsigned long uptime = millis() / 1000; // Sekunden
  int32_t rssi = WiFi.RSSI();             // dBm
  uint32_t freeHeap = ESP.getFreeHeap();  // Bytes

  // JSON-Nachricht bauen
  String payload = "{";
  payload += "\"Uptime\":" + String(uptime) + ",";
  payload += "\"WiFiRSSI\":" + String(rssi) + ",";
  payload += "\"IP\":" + String(WiFi.localIP().toString().c_str()) + ",";
  payload += "\"FreeHeap\":" + String(freeHeap) + ",";
  payload += "\"Reconnects\":" + String(reconcount);
  payload += "}";
  Serial.println(payload);
  client.publish(topic, payload.c_str());
}

void setup() {
  Serial.begin(115200);
  // while (!Serial) delay(10);
  delay(2000);

    // Initialize display
    u8g2.begin();
    // u8g2.setFont(u8g2_font_8bitclassic_tr); // Set a readable font
    u8g2.setFont(u8g2_font_missingplanet_tr);
    // u8g2.setFont(u8g2_font_questgiver_tr);
    // u8g2.setFont(u8g2_font_crox1tb_tr);
    // u8g2.setFont(u8g2_font_lastapprenticethin_tr);
    // u8g2.setFont(u8g2_font_eckpixel_tr);
    // u8g2.setFont(u8g2_font_tenthinnerguys_tr);
    // u8g2.setFont(u8g2_font_NokiaSmallBold_tr);
    // u8g2.setFontRefHeightExtendedText();
    u8g2.setDrawColor(1);
    u8g2.setBusClock(25000000);
    u8g2.setFontPosTop();
    u8g2.setContrast(255);
    // u8g2.setFontDirection(0);
    u8g2.clearBuffer();
    u8g2.setCursor(xOffset, yOffset);
    u8g2.print("EC3000 MQTT Bridge");
    u8g2.sendBuffer();
  
    // Configure LED PWM with new ESP-IDE > v3 API (in short: it's now ledcAttach instead of ledcAttachSetup and ledcAttachPin)
    Serial.println(LED_PIN);
    delay(5000);
    u8g2.print(LED_PIN);
    u8g2.sendBuffer();
    // pinMode(LED_PIN, OUTPUT);
    ledcAttach(LED_PIN, LED_PWM_FREQ, LED_PWM_RESOLUTION);  // Auto-assigns channel
    ledcWrite(LED_PIN, 255);                                // Off for active-low

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());
  u8g2.setCursor(xOffset, yOffset + 14);
  u8g2.print("WLAN verbunden!");
  u8g2.sendBuffer();

  client.setBufferSize(1024);  
  client.setServer(mqtt_server, mqtt_port);
  if (!client.connected()) {
    if (client.connect("ESP32C3Client",mqtt_user,mqtt_password)) {
      Serial.println("Verbunden mit dem MQTT-Server!");
    } else {
      Serial.print("Verbindung fehlgeschlagen, rc=");
      Serial.println(client.state());
      delay(5000);
    }
  }
  if (!client.publish("EC3000/debug", "EC3000 MQTT Bridge initialized successfully!")) {
    Serial.println("Fehler beim Senden der DEBUG-Nachricht.");
  }

  pinMode(RFM69_CS, OUTPUT);
  digitalWrite(RFM69_CS, HIGH);

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  SPI.begin(RFM69_SCK, RFM69_MISO, RFM69_MOSI, RFM69_CS);
  delay(10);

  WriteReg(REG_OPMODE, 0x04);
  delay(10);

  WriteReg(REG_PAYLOADLENGTH, 0x0A);
  if (ReadReg(REG_PAYLOADLENGTH) != 0x0A) {
    Serial.println("RFM69 detection failed at step 1!");
    u8g2.setCursor(xOffset, yOffset + 14);
    u8g2.print("No RFM69");
    u8g2.setCursor(xOffset, yOffset + 28);
    u8g2.print("found!!");
    u8g2.sendBuffer();
    while (1);
  }
  WriteReg(REG_PAYLOADLENGTH, 0x40);
  if (ReadReg(REG_PAYLOADLENGTH) != 0x40) {
    Serial.println("RFM69 detection failed at step 2!");
    u8g2.setCursor(xOffset, yOffset + 14);
    u8g2.print("No RFM69");
    u8g2.setCursor(xOffset, yOffset + 28);
    u8g2.print("found!!");
    u8g2.sendBuffer();
    while (1);
  }
  Serial.println("RFM69 detected!");

  // Set RFM69 Frequency to 868.3 MhZ
  unsigned long f = (((FREQUENCY_KHZ * 1000UL) << 2) / (32000000UL >> 11)) << 6;
  WriteReg(0x07, f >> 16);
  WriteReg(0x08, f >> 8);
  WriteReg(0x09, f);

  // Set RFM69 Datarate to 20000 baud (20kbps)
  uint16_t r = ((32000000UL + (DATA_RATE / 2)) / DATA_RATE);
  WriteReg(0x03, r >> 8);
  WriteReg(0x04, r & 0xFF);
  
  // Set EC3000 Preamble Bytes and Modulation and so on...
  WriteReg(0x02, 0x00);
  WriteReg(0x05, 0x01);
  WriteReg(0x06, 0x48);
  WriteReg(0x11, 0x9F);
  WriteReg(0x13, 0x0F);
  WriteReg(0x18, 0x80);
  WriteReg(0x19, 0x42);
  WriteReg(0x28, 0x10);
  WriteReg(0x29, 220);
  WriteReg(0x2E, 0xA0);
  WriteReg(0x2F, 0x13);
  WriteReg(0x30, 0xF1);
  WriteReg(0x31, 0x85);
  WriteReg(0x32, 0xD3);
  WriteReg(0x33, 0xAC);
  WriteReg(0x37, 0x08);
  WriteReg(0x38, 64);
  WriteReg(0x3C, 0x8F);
  WriteReg(0x3D, 0x12);
  WriteReg(0x6F, 0x30);

  WriteReg(REG_OPMODE, (ReadReg(REG_OPMODE) & 0xE3) | 0x10);

  Serial.println("RFM69 initialized successfully!");
  u8g2.setCursor(xOffset, yOffset + 28);
  u8g2.print("Found RFM69");
  u8g2.sendBuffer();

  for (int i = 0; i < whitelistSize; i++) {
    uint16_t id = strtol(whitelist[i], nullptr, 16);  // Umwandlung von Hex-String zu uint16_t
    publishDiscoveryMessages(id);
  }

  uint8_t reg07 = ReadReg(0x07);
  uint8_t reg08 = ReadReg(0x08);
  uint8_t reg09 = ReadReg(0x09);
  uint32_t freqVal = ((uint32_t)reg07 << 16) | (reg08 << 8) | reg09;
  float freqMHz = (freqVal * 32000000.0) / (1UL << 19) / 1000000.0;

  uint8_t reg03 = ReadReg(0x03);
  uint8_t reg04 = ReadReg(0x04);
  uint16_t bitrateVal = (reg03 << 8) | reg04;
  float bitrate = 32000000.0 / bitrateVal / 1000.0;

  webserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<html><body><h2>ESP EC3000</h2>";
    // html += "<p>Power: " + String(currentPower) + " W</p>";
    html += "<p>RSSI: " + String(WiFi.RSSI()) + " dBm</p>";
    html += "<p>Uptime: " + String(millis() / 1000) + " s</p>";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });

  webserver.begin();
  Serial.print("Listening at ");
  Serial.print(freqMHz);
  Serial.print(" MHz with ");
  Serial.print(bitrate);
  Serial.println(" kbps...");

  }
  
void loop() {
  if (!client.connected()) {
    reconnect();
  }

  if (millis() - lastStatusPublish > 30000) {
    publishSystemStatus(0xDEAD); // Beispiel-ID
    lastStatusPublish = millis();
  }

  client.loop();

  cleanStaleIDs();

  handleButton();

  if (ReadReg(REG_IRQFLAGS2) & 0x04) {
    m_payloadPointer = 0;
    for (int i = 0; i < 64; i++) {
      uint8_t bt = GetByteFromFifo();
      m_payload[i] = bt;
      m_payloadPointer++;
    }
    m_payloadReady = true;

    uint8_t ec3k_payload[PAYLOAD_SIZE];
    for (int i = 0; i < PAYLOAD_SIZE; i++) {
      ec3k_payload[i] = m_payload[i];
    }

    struct Frame frame;
    DecodeFrame(ec3k_payload, &frame);

    float rssi = -(ReadReg(0x27) / 2.0);

    bool valid = true;
    String reason = "";
    
    String idStr = String(frame.ID, HEX);  // wandelt z. B. 0xBF8C → "bf8c"
    idStr.toUpperCase();                   // jetzt: "BF8C"

    if (!isWhitelisted(idStr)) {
      valid = false;
      reason += "wrong ID" + idStr;
      return;
    }

    if (frame.OnSeconds > frame.TotalSeconds) {
      valid = false;
      reason += "OnSeconds > TotalSeconds; ";
    }

    if (!frame.IsOn && frame.Power != 0) {
      valid = false;
      reason += "IsOn=No but Power>0; ";
    }

    if (frame.Power > 3500) {
      valid = false;
      reason += "Power too high; ";
    }

    double lastConsumption;
    if (!checkConsumption(frame.ID, frame.Consumption, &lastConsumption)) {
      valid = false;
      reason += "Consumption invalid (last=" + String(lastConsumption, 3) + "); ";
    }

    if (logOnlyFailed) {
      if (!valid) {
        printFrame(&frame, rssi);
        Serial.print("Discarded: ");
        client.publish("EC3000/debug", "Discarded");
        Serial.println(reason);
        char debtopic[32];
        char debpayload[256];
        snprintf(debtopic, sizeof(debtopic), "EC3000/debug");
        snprintf(debpayload, sizeof(debpayload), reason.c_str());
        client.publish(debtopic, debpayload);
        Serial.println();
      }
    } else {
      printFrame(&frame, rssi);
      if (!valid) {
        Serial.print("Discarded: ");
        Serial.println(reason);
      }
      Serial.println();
    }

    if (valid) {
      char topic[32];
      char payload[256];
      snprintf(topic, sizeof(topic), "EC3000/%04X", frame.ID);
      snprintf(payload, sizeof(payload),
               "{\"TotalSeconds\":%lu,\"OnSeconds\":%lu,\"Consumption\":%.3f,\"Power\":%.1f,\"MaximumPower\":%.1f,\"NumberOfResets\":%u,\"IsOn\":%d,\"CRC\":\"0x%04X\",\"RSSI\":%.2f}",
               frame.TotalSeconds, frame.OnSeconds, frame.Consumption, frame.Power, frame.MaximumPower, frame.NumberOfResets, frame.IsOn, frame.CRC, rssi);
      client.publish(topic, payload);
    }
  } else {
    // Full brightness for invalid packet
    /* ledcWrite(LED_PIN, LED_INVALID_BRIGHTNESS);  // Full brightness
    ledInvalidOnTime = millis();
    ledInvalidIsOn = true;
    ledValidIsOn = false;  // Cancel any valid flash
    */
    // Serial.println("Invalid packet received - LED ON (full)");
  }
  WriteReg(REG_IRQFLAGS2, 0x04);
  m_payloadReady = false;

  /* Turn LED off after respective durations
  if (ledValidIsOn && millis() - ledValidOnTime >= LED_VALID_DURATION) {
    ledcWrite(LED_PIN, 255);  // Off for active-low
    ledValidIsOn = false;
    // Serial.println("LED OFF (valid)");
  }
  if (ledInvalidIsOn && millis() - ledInvalidOnTime >= LED_INVALID_DURATION) {
    ledcWrite(LED_PIN, 255);  // Off for active-low
    ledInvalidIsOn = false;
    // Serial.println("LED OFF (invalid)");
  }
  */

  // Periodically refresh display to maintain loading bar animation
  /* static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 100) {
    drawDisplay();
    lastDisplayUpdate = millis();
  }
  */
}