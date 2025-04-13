// benutzte Pins auf ESP32-C3 Board mit 0.42" OLED
// https://www.aliexpress.com/item/1005008125231916.html
// oder suche "ESP32-C3 oled 0 42"
// (nur zur Übersicht; werden später im Code initialisiert)
//
// PIN 5 ist SDA; I2C; Display (und evtl. sonstige I2C Sensoren)
// PIN 6 ist SCL; I2C; Display (und evtl. sonstige I2C Sensoren)
// PIN 8 ist die LED auf der Platine (blau auf den günstigen AliExpress Boards und an PIN 8; es gibt wohl auch welche mit einer RGB LED an PIN 2?)
// PIN 9 ist der "BO0" Knopf

// Pin definitions RFM69
#define RFM69_CS   34  // NSS (Chip Select)
#define RFM69_INT  5   // DIO0 (Interrupt) NOT USED - you do NOT need to connect it!
#define RFM69_MOSI 35  // SPI MOSI
#define RFM69_MISO 37  // SPI MISO
#define RFM69_SCK  36  // SPI SCK
#define RFM69_RST  18  // RESET - USED; but it also works without because the RFM69 is in working state when this pin is not connected.

// Pin Definitionen RFM69 (frei wählbar) an ESP32-C3 mit 0,42" OLED
/* 
#define RFM69_CS 7
#define RFM69_MOSI 3
#define RFM69_MISO 10
#define RFM69_SCK 4
#define RFM69_RST 2
*/

// Hersteller Pin Definitionen 0.42" OLED auf dem günstigen ESP32-C3 Board
#define OLED_SCL 6  // (fest auf der Platine zum Display verdrahtet)
#define OLED_SDA 5  // (fest auf der Platine zum Display verdrahtet)

// Hersteller Pin Definitionen der auf dem Board verbauten LED
#define LED_PIN 8                 // (fest auf der Platine verdrahtet)