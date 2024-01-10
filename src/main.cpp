/*
 * Interface com LCD 2.2" SPI TFT Module
 *
 * Ref.: https://simple-circuit.com/esp8266-nodemcu-ili9341-tft-display/
 *
 * Pinagem:
 * TFT Pinout | ESP12c
 * 1 VCC      | 3v3
 * 2 GND      | GND
 * 3 CS       | D2 (ESP8266EX GPIO4)
 * 4 RST      | D3 (ESP8266EX GPIO0)
 * 5 D/C      | D4 (ESP8266EX GPIO2)
 * 6 MOSI     | D7 (ESP8266EX GPIO13)
 * 7 SCK      | D5 (ESP8266EX GPIO14)
 * 8 BL       | 3v3
 * 9 MISO     | Não conectado
 *
 * Pins D5 (GPIO14) and D7 (GPIO13) are hardware SPI module pins of the ESP8266EX microcontroller
 * respectively for SCK (serial clock) and MOSI (master-out slave-in).
 */

#include "Arduino.h"

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WifiCredentials.h>

#include <Adafruit_ILI9341.h>   // include Adafruit ILI9341 TFT library

#define TFT_CS    D2     // TFT CS  pin is connected to NodeMCU pin D2
#define TFT_RST   D3     // TFT RST pin is connected to NodeMCU pin D3
#define TFT_DC    D4     // TFT DC  pin is connected to NodeMCU pin D4
// initialize ILI9341 TFT library with hardware SPI module
// SCK (CLK) ---> NodeMCU pin D5 (GPIO14)
// MOSI(DIN) ---> NodeMCU pin D7 (GPIO13)
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
String printedTime = String();

#define DISPLAY_W 320
#define DISPLAY_H 240
#define MARGIN_W 3
#define MARGIN_H 1

int16_t time_x, time_y, time_w, time_h;

WiFiUDP wifiUDP = WiFiUDP();
IPAddress broadcastIP;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
long udpNum = 0;
long lastUdpNum = 0;
char sensors[4];

boolean OTA_Updating = false;

unsigned long testFillScreen();
unsigned long testText();

void blinkLed();
unsigned long blinkLastMillis = millis();
uint8_t blinkLevel = LOW;

int16_t progress_x;
int16_t progress_y;

void conectaWifi();

void verifyChar(Stream &stream, int c);

void setup()
{
    Serial.begin(115200);
    Serial.println();
    Serial.println("Show me all!");

    // initialize LED digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    tft.begin();
    tft.setRotation(1);

    conectaWifi();

    // OTA SETUP
    // Port defaults to 8266
    // ArduinoOTA.setPort(8266);

    // Hostname defaults to esp8266-[ChipID]
    ArduinoOTA.setHostname("showmeall");

    // No authentication by default
    ArduinoOTA.setPasswordHash("26a4b3151ec19724b0c29513bce9aa50");

    // Password can be set with it's md5 value as well
    // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

    ArduinoOTA.onStart([]() {
        OTA_Updating = true;
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else {  // U_FS
            type = "filesystem";
        }

        // NOTE: if updating FS this would be the place to unmount FS using FS.end()
        Serial.println("Start updating " + type);
        
        tft.setCursor(0,0);
        tft.fillScreen(ILI9341_BLACK);
        tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
        tft.println();
        tft.print("Start updating " + type);

        tft.setTextColor(ILI9341_WHITE, ILI9341_BLUE); tft.setTextSize(2);
        tft.println();
        //tft.print("Progress : ");
        progress_x = tft.getCursorX();
        progress_y = tft.getCursorY();
    });
    ArduinoOTA.onEnd([]() {
        tft.setCursor(0,0);
        tft.fillScreen(ILI9341_BLACK);
        Serial.println("\nEnd");
        OTA_Updating = false;
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        unsigned int perc = progress / (total / 100);
        Serial.printf("Progress: %u%%\r", perc);
        if (perc%10==0) {
            tft.setCursor(progress_x, progress_y);
            tft.printf("Progress: %u%%", perc);
        }
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        tft.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
            Serial.println("Auth Failed");
            tft.println("Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
            Serial.println("Begin Failed");
            tft.println("Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
            Serial.println("Connect Failed");
            tft.println("Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
            Serial.println("Receive Failed");
            tft.println("Receive Failed");
        } else if (error == OTA_END_ERROR) {
            Serial.println("End Failed");
            tft.println("End Failed");
        }
    });
    ArduinoOTA.begin();

    // UDP client
    broadcastIP = WiFi.broadcastIP();
    wifiUDP.begin(19020);

    // water sensors
    sensors[0] = 'x';
    sensors[1] = 'x';
    sensors[2] = 'x';
    sensors[3] = 'x';

    // inicia a escuta do UDP

    // Initialize a NTPClient to get time
    timeClient.begin();
    // Set offset time in seconds to adjust for your timezone, for example:
    // GMT +1 = 3600
    // GMT +8 = 28800
    // GMT -1 = -3600
    // GMT 0 = 0
    timeClient.setTimeOffset(-3600*3); // GMT-3

    delay(4000);
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(3);

    time_w = (18*8)-3;
    time_h = 21;
    time_x = DISPLAY_W - time_w - MARGIN_W;
    time_y = 1;
}

void loop()
{
    ArduinoOTA.handle();
    timeClient.update();

    if (!OTA_Updating) {

        const String &formattedTime = timeClient.getFormattedTime();
        if (formattedTime != printedTime) {
            printedTime = formattedTime;
            tft.fillRect((short) (time_x - MARGIN_W), (short) (time_y - MARGIN_H),
                         (short) (time_w + (2 * MARGIN_W)), (short) (time_h + (2 * MARGIN_H)),
                         ILI9341_DARKGREEN);
            tft.setCursor(time_x, time_y);

            tft.println(formattedTime);
        }

        if (udpNum != lastUdpNum) {
            lastUdpNum = udpNum;
            int16_t linSensor = 0;
            tft.fillRect(0, linSensor,
                         (6 * 18) + 2 * MARGIN_W, (short) (2 * time_h + 4 * MARGIN_H),
                         ILI9341_DARKGREY);
            tft.setCursor(0, linSensor);
            tft.write(sensors, 4);

            tft.setCursor(0, (short) (linSensor + time_h + 2 * MARGIN_H));
            tft.print(udpNum);
        }
    }

    int packetSize = wifiUDP.parsePacket();
    if (packetSize) {
        try {
            verifyChar(wifiUDP, 'N');
            verifyChar(wifiUDP, '=');
            udpNum = wifiUDP.parseInt();
            verifyChar(wifiUDP, '-');
            wifiUDP.readBytes(sensors, 4);
        } catch (int error_code) {
            Serial.print(F("Falha:"));
            Serial.println(error_code);
        }

    }

    blinkLed();
}

void verifyChar(Stream &stream, int c) {
    int r = stream.read();
    if (r!=c) {
        throw (c);
    }
}

void blinkLed() {

    unsigned long now = millis();
    if (((blinkLastMillis + 500) <= now) || (now < blinkLastMillis)) {
        blinkLastMillis = now;

        blinkLevel = blinkLevel == LOW ? HIGH : LOW;
        digitalWrite(LED_BUILTIN, blinkLevel);
    }
}

unsigned long testFillScreen() {
    unsigned long start = micros();
    tft.fillScreen(ILI9341_BLACK);
    tft.fillScreen(ILI9341_RED);
    tft.fillScreen(ILI9341_GREEN);
    tft.fillScreen(ILI9341_BLUE);
    tft.fillScreen(ILI9341_BLACK);
    return micros() - start;
}

unsigned long testText() {
    tft.fillScreen(ILI9341_BLACK);
    unsigned long start = micros();
    tft.setCursor(0, 0);
//    tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
//    tft.println("Recado");
//    tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
//    tft.println(10.23);
//    tft.setTextColor(ILI9341_RED);    tft.setTextSize(3);
//    tft.println(0xDEADBEEF, HEX);
//    tft.println();
//    tft.setTextColor(ILI9341_GREEN);
//    tft.setTextSize(5);
//    tft.println("TIAGO");
//    tft.setTextSize(2);
//    tft.println("Eu te amo papai!");
//    tft.setTextSize(1);
//    tft.println("Abraço.");

    tft.setTextColor(ILI9341_WHITE); tft.setTextSize(3);
    tft.println("Luciene,");
    tft.println();
    tft.setTextColor(ILI9341_RED); tft.setTextSize(5);
    tft.println("EU TE AMO!");
    tft.setTextColor(ILI9341_BLUE); tft.setTextSize(3);
    tft.println();
    tft.println("Sergio Marcelo");

    return micros() - start;
}

void conectaWifi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    Serial.print(F("Aguardando WIFI"));

    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
    tft.println();
    tft.print(F("Conectando WIFI"));

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        tft.print(".");
    }
    tft.setTextColor(ILI9341_WHITE); tft.setTextSize(2);
    tft.println();
    tft.print(F("Connected to "));
    tft.println(ssid);
    tft.println(F("Local IP address: "));
    tft.println(WiFi.localIP());
    tft.println(F("Broadcast IP address: "));
    tft.println(WiFi.broadcastIP());
}