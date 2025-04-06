#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Pin definitions
#define RFM69_CS   34  // NSS (Chip Select)
#define RFM69_INT  5   // DIO0 (Interrupt) NOT USED - you do NOT need to connect it!
#define RFM69_MOSI 35  // SPI MOSI
#define RFM69_MISO 37  // SPI MISO
#define RFM69_SCK  36  // SPI SCK
#define RFM69_RST  18  // RESET - USED; but it also works without because the RFM69 is in working state when this pin is not connected.

#define FREQUENCY_KHZ  868300
#define DATA_RATE      20000
#define REG_OPMODE     0x01
#define REG_PAYLOADLENGTH 0x38
#define REG_IRQFLAGS2  0x28

#define PAYLOAD_SIZE 47
#define FRAME_LENGTH 38

// WiFi and MQTT settings (EDIT THESE)
const char* ssid = "YOURWIFI";
const char* password = "YOURWIFIPASSWORD";
const char* mqtt_server = "192.168.0.254";
const int mqtt_port = 1883;
const char* mqtt_user = ""; // put your mqtt username here
const char* mqtt_password = ""; // put your mqtt password here

// Logging toggle
bool logOnlyFailed = true; // true = log only failed, false = log all

// Payload buffer
uint8_t m_payload[64];
uint8_t m_payloadPointer = 0;
bool m_payloadReady = false;

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

// WiFi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);

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

// EC3000 decoding functions
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

byte Count1bits(uint32_t v) {
  byte c;
  for (c = 0; v; c++) {
    v &= v - 1;
  }
  return c;
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
  frame->Consumption = cons / 3600000.0;

  frame->Power = ((uint16_t)payload[15] << 8 | payload[16]) / 10.0;
  frame->MaximumPower = ((uint16_t)payload[17] << 8 | payload[18]) / 10.0;
  frame->NumberOfResets = (payload[36] << 4) | (payload[37] >> 4);
  frame->IsOn = payload[37] & 0x08;
  frame->Reception = 0;
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    String clientId = "ESP32C3-EC3000-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");
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

void cleanStaleIDs() {
  unsigned long now = millis();
  for (int i = 0; i < MAX_IDS; i++) {
    if (resetTrackers[i].Initialized && (now - resetTrackers[i].LastSeen > 66000)) { // behalte die IDs länger um Probleme mit echten IDs zu vermeiden die aus welchen Gründen auch immer nicht regelmässig empfangen werden.
      Serial.print("Removing stale ID: 0x"); Serial.println(resetTrackers[i].ID, HEX);
      client.publish("EC3000/debug", "Removing stale ID");
      resetTrackers[i].Initialized = false; // Mark as free
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

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(2000);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  client.setServer(mqtt_server, mqtt_port);

  pinMode(RFM69_CS, OUTPUT);
  digitalWrite(RFM69_CS, HIGH);

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  SPI.begin(RFM69_SCK, RFM69_MISO, RFM69_MOSI, RFM69_CS);
  // SPI.setClockDivider(SPI_CLOCK_DIV4);
  // SPI.setDataMode(SPI_MODE0);
  delay(10);

  WriteReg(REG_OPMODE, 0x04);
  delay(10);


  // Write some random stuff into registers and read it back to verify SPI connection works and we have a RFM60
  WriteReg(REG_PAYLOADLENGTH, 0x0A);
  if (ReadReg(REG_PAYLOADLENGTH) != 0x0A) {
    Serial.println("RFM69 detection failed at step 1!");
    while (1);
  }
  WriteReg(REG_PAYLOADLENGTH, 0x40);
  if (ReadReg(REG_PAYLOADLENGTH) != 0x40) {
    Serial.println("RFM69 detection failed at step 2!");
    while (1);
  }
  Serial.println("RFM69 detected!");

  unsigned long f = (((FREQUENCY_KHZ * 1000UL) << 2) / (32000000UL >> 11)) << 6;
  WriteReg(0x07, f >> 16);
  WriteReg(0x08, f >> 8);
  WriteReg(0x09, f);

  uint16_t r = ((32000000UL + (DATA_RATE / 2)) / DATA_RATE);
  WriteReg(0x03, r >> 8);
  WriteReg(0x04, r & 0xFF);

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
  client.publish("EC3000/debug", "EC3000 MQTT Bridge initialized successfully!");

  uint8_t reg07 = ReadReg(0x07);
  uint8_t reg08 = ReadReg(0x08);
  uint8_t reg09 = ReadReg(0x09);
  uint32_t freqVal = ((uint32_t)reg07 << 16) | (reg08 << 8) | reg09;
  float freqMHz = (freqVal * 32000000.0) / (1UL << 19) / 1000000.0;

  uint8_t reg03 = ReadReg(0x03);
  uint8_t reg04 = ReadReg(0x04);
  uint16_t bitrateVal = (reg03 << 8) | reg04;
  float bitrate = 32000000.0 / bitrateVal / 1000.0;

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
  client.loop();

  // Clean stale IDs every loop (could be optimized with a timer)
  cleanStaleIDs();

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

    // Sanity checks
    bool valid = true;
    String reason = "";

    if (frame.OnSeconds > frame.TotalSeconds) {
      valid = false;
      reason += "OnSeconds > TotalSeconds; ";
    }

    if (!frame.IsOn && frame.Power != 0) {
      valid = false;
      reason += "IsOn=No but Power>0; ";
    }

    if (frame.Power > 2000) {
      valid = false;
      reason += "Power too high; ";
    }

    // The Reset does more complex things than that for unknown reasons but it's not really worth it messing around with it because all bad packets are sorted out with the other checks already.
    // uint16_t lastResets;
    // double lastConsumption;
    // if (!updateResetTracker(frame.ID, frame.NumberOfResets, frame.Consumption, &lastResets, &lastConsumption)) {
    //   valid = false;
    //   reason += "Resets not +1 (last=" + String(lastResets) + "); ";
    // } 
    
    if (!updateResetTracker(frame.ID, frame.NumberOfResets, frame.Consumption, &lastResets, &lastConsumption)) {
      // Consumption check
      double delta = frame.Consumption - lastConsumption;
      if (delta < 0 || delta > 0.025) {
        valid = false;
        reason += "Consumption invalid (last=" + String(lastConsumption, 3) + ", delta=" + String(delta, 3) + "); ";
      }
    }

    // Logging logic
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

    // MQTT publish only valid packets
    if (valid) {
      char topic[32];
      char payload[256];
      snprintf(topic, sizeof(topic), "EC3000/%04X", frame.ID);
      snprintf(payload, sizeof(payload),
               "{\"TotalSeconds\":%lu,\"OnSeconds\":%lu,\"Consumption\":%.3f,\"Power\":%.1f,\"MaximumPower\":%.1f,\"NumberOfResets\":%u,\"IsOn\":%d,\"CRC\":\"0x%04X\",\"RSSI\":%.2f}",
               frame.TotalSeconds, frame.OnSeconds, frame.Consumption, frame.Power, frame.MaximumPower, frame.NumberOfResets, frame.IsOn, frame.CRC, rssi);
      client.publish(topic, payload);
    }

    WriteReg(REG_IRQFLAGS2, 0x04);
    m_payloadReady = false;
  }
}
