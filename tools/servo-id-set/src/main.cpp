#include <Arduino.h>
#include <M5Unified.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>

WebServer server(80);
const char ssid[] = "servoOffset";
const char pass[] = "password";
const IPAddress ip(192, 168, 55, 28);
const IPAddress subnet(255, 255, 255, 0);

Preferences preferences;

int offset[] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned long lastDisplayUpdate = 0;
unsigned long lastSerialUpdate = 0;
unsigned long txCount = 0;
int lastPosition[] = {0, 0, 0, 0, 0, 0, 0, 0};

void handleRoot();
void handleOffset(int id, int delta);
void scs_moveToPos(byte id, int position);
void servo_write(int ch, float ang);

void drawStatus(const char* status) {
  M5.Display.fillScreen(0x0000);
  M5.Display.setTextColor(0xFFFF, 0x0000);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(0, 0);
  M5.Display.println("SCS0009 CTRL");
  M5.Display.println(status);
  M5.Display.println();
  M5.Display.print("AP: ");
  M5.Display.println(ssid);
  M5.Display.print("IP: ");
  M5.Display.println(WiFi.softAPIP());
  M5.Display.print("STA: ");
  M5.Display.println(WiFi.softAPgetStationNum());
  M5.Display.print("TX: ");
  M5.Display.println(txCount);
  M5.Display.println();
  M5.Display.println("Open:");
  M5.Display.println(WiFi.softAPIP());
}

void handleRoot() {
  Serial.println("[WEB] GET /");
  String temp = "<!DOCTYPE html>\n<html lang=\"ja\">";
  temp += "<head>";
  temp += "<meta charset=\"utf-8\">";
  temp += "<title>servoOffset</title>";
  temp += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  temp += "<style>";
  temp += ".container{ margin:auto; text-align:center; font-size:1.2rem; }";
  temp += "span,.pm{ display:inline-block; border:1px solid #ccc; width:50px; height:30px; vertical-align:middle; margin-bottom:8px; }";
  temp += "span{ width:120px; }";
  temp += "button{ width:100px; height:40px; font-weight:bold; margin-bottom:8px; }";
  temp += "button.on{ background:lime; color:white; }";
  temp += "</style>";
  temp += "</head><body><div class=\"container\">";
  temp += "<h3>servoOffset</h3>";

  for (int i = 0; i < 8; i++) {
    temp += "ID" + String(i) + "<br>";
    temp += "<a class=\"pm\" href=\"/id" + String(i) + "M\">-</a>";
    temp += "<span>" + String(offset[i]) + "</span>";
    temp += "<a class=\"pm\" href=\"/id" + String(i) + "P\">+</a><br>";
  }

  temp += "</div></body>";
  server.send(200, "text/html", temp);
}

void handleOffset(int id, int delta) {
  Serial.print("[WEB] ID");
  Serial.print(id);
  Serial.print(delta > 0 ? " +" : " -");
  Serial.print(" before=");
  Serial.println(offset[id]);

  if (offset[id] + delta >= -511 && offset[id] + delta <= 511) {
    offset[id] += delta;
    preferences.putInt(("offset" + String(id)).c_str(), offset[id]);
  }

  Serial.print("[WEB] ID");
  Serial.print(id);
  Serial.print(" after=");
  Serial.println(offset[id]);
  handleRoot();
}

#define MAKE_HANDLER(ID) \
void handleId##ID##M(){ handleOffset(ID, -20); } \
void handleId##ID##P(){ handleOffset(ID, +20); }

MAKE_HANDLER(0)
MAKE_HANDLER(1)
MAKE_HANDLER(2)
MAKE_HANDLER(3)
MAKE_HANDLER(4)
MAKE_HANDLER(5)
MAKE_HANDLER(6)
MAKE_HANDLER(7)

void scs_moveToPos(byte id, int position) {
  byte message[13];
  message[0] = 0xFF;
  message[1] = 0xFF;
  message[2] = id;
  message[3] = 9;
  message[4] = 3;
  message[5] = 42;
  message[6] = (position >> 8) & 0xFF;
  message[7] = position & 0xFF;
  message[8] = 0x00;
  message[9] = 0x00;
  message[10] = 0x00;
  message[11] = 0x00;

  byte checksum = 0;
  for (int i = 2; i < 12; i++) checksum += message[i];
  message[12] = ~checksum;

  Serial1.write(message, 13);
}

void servo_write(int ch, float ang) {
  int sig = 511 + offset[ch] + int((512.0 / 150.0) * ang);
  sig = constrain(sig, 0, 1023);
  lastPosition[ch] = sig;
  scs_moveToPos(ch, sig);
}

void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);

  delay(300);
  Serial.println();
  Serial.println("[BOOT] SCS0009 controller starting");
  M5.Display.setRotation(0);
  drawStatus("Booting...");

  Serial1.begin(1000000, SERIAL_8N1, -1, 2);
  Serial.println("[UART] Serial1 started: baud=1000000 rx=-1 tx=GPIO2(G2)");
  drawStatus("UART TX=G2");

  preferences.begin("parameter", false);
  for (int i = 0; i < 8; i++) {
    offset[i] = preferences.getInt(("offset" + String(i)).c_str(), offset[i]);
    Serial.print("[NVS] offset ID");
    Serial.print(i);
    Serial.print("=");
    Serial.println(offset[i]);
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ip, ip, subnet);
  WiFi.softAP(ssid, pass);
  delay(100);
  Serial.print("[WIFI] AP SSID=");
  Serial.println(ssid);
  Serial.print("[WIFI] AP IP=");
  Serial.println(WiFi.softAPIP());
  drawStatus("WiFi AP ready");

  server.on("/", handleRoot);
  server.on("/id0M", handleId0M); server.on("/id0P", handleId0P);
  server.on("/id1M", handleId1M); server.on("/id1P", handleId1P);
  server.on("/id2M", handleId2M); server.on("/id2P", handleId2P);
  server.on("/id3M", handleId3M); server.on("/id3P", handleId3P);
  server.on("/id4M", handleId4M); server.on("/id4P", handleId4P);
  server.on("/id5M", handleId5M); server.on("/id5P", handleId5P);
  server.on("/id6M", handleId6M); server.on("/id6P", handleId6P);
  server.on("/id7M", handleId7M); server.on("/id7P", handleId7P);

  server.begin();
  Serial.println("[WEB] server started on port 80");
  drawStatus("Web server ready");
}

void loop() {
  M5.update();
  for (int i = 0; i < 8; i++) {
    servo_write(i, 0);
    txCount++;
  }
  server.handleClient();

  if (millis() - lastDisplayUpdate > 500) {
    lastDisplayUpdate = millis();
    drawStatus("Running");
  }

  if (millis() - lastSerialUpdate > 1000) {
    lastSerialUpdate = millis();
    Serial.print("[RUN] sta=");
    Serial.print(WiFi.softAPgetStationNum());
    Serial.print(" tx=");
    Serial.print(txCount);
    Serial.print(" pos=");
    for (int i = 0; i < 8; i++) {
      Serial.print(i);
      Serial.print(":");
      Serial.print(lastPosition[i]);
      if (i < 7) Serial.print(",");
    }
    Serial.println();
  }

  delay(300);
}
