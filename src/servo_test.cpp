#include <Arduino.h>

// ── 設定 ─────────────────────────────────────────────────────
#define TX_PIN    38
#define CENTER    511
#define POS_PLUS  614   // +30°
#define POS_MINUS 408   // -30°

// ── raw UART 送信（SCServoライブラリ不使用） ──────────────────
void sendPos(uint8_t id, int pos, uint16_t time_ms) {
  pos = constrain(pos, 0, 1023);
  byte msg[13];
  msg[0]=0xFF; msg[1]=0xFF;
  msg[2]=id;   msg[3]=9;
  msg[4]=3;    msg[5]=42;           // INST_WRITE, GOAL_POSITION_L
  msg[6]=(pos>>8)&0xFF;
  msg[7]=pos&0xFF;
  msg[8]=(time_ms>>8)&0xFF;
  msg[9]=time_ms&0xFF;
  msg[10]=0; msg[11]=0;             // speed=0
  byte sum=0;
  for(int i=2;i<12;i++) sum+=msg[i];
  msg[12]=~sum;
  Serial1.write(msg, 13);
  Serial1.flush();
}

void swing(uint8_t id, int ms) {
  sendPos(id, POS_PLUS,  ms); delay(ms + 200);
  sendPos(id, CENTER,    ms); delay(ms + 200);
  sendPos(id, POS_MINUS, ms); delay(ms + 200);
  sendPos(id, CENTER,    ms); delay(ms + 200);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== servo_test start (raw UART, no library) ===");

  // TX ピンをまず HIGH (UART アイドル状態) にしてから UART 開始
  pinMode(TX_PIN, OUTPUT);
  digitalWrite(TX_PIN, HIGH);
  delay(10);
  Serial1.begin(1000000, SERIAL_8N1, -1, TX_PIN);
  delay(100);
  Serial.printf("UART ready: baud=1000000 TX=GPIO%d\n", TX_PIN);

  // ── ブロードキャストでセンターへ ──
  Serial.println("[1] broadcast -> center (500ms)");
  sendPos(0xFE, CENTER, 500);
  delay(1500);

  // ── ID 0〜7 個別確認 ──
  Serial.println("[2] individual ID swing (0..7, 400ms)");
  for (int id = 0; id <= 7; id++) {
    Serial.printf("  ID=%d\n", id);
    swing(id, 400);
    delay(300);
  }

  Serial.println("[3] all IDs done -> speed test (broadcast)");
  delay(500);
}

void loop() {
  static int ms = 600;
  static int cycle = 0;

  Serial.printf("[cycle %2d] broadcast move_time=%dms\n", cycle, ms);

  sendPos(0xFE, POS_PLUS,  ms); delay(ms + 200);
  sendPos(0xFE, CENTER,    ms); delay(ms + 200);
  sendPos(0xFE, POS_MINUS, ms); delay(ms + 200);
  sendPos(0xFE, CENTER,    ms); delay(ms + 200);

  cycle++;
  if      (ms > 200) ms -= 50;
  else if (ms > 80)  ms -= 20;
  else if (ms > 30)  ms -= 10;
}
