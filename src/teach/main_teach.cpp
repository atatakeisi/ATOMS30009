// ATOMS30009 Teaching / 計測再生ファーム — Phase 0 検証ハーネス
// 目的: Teaching全体の前提を実機で確認する。
//   (1) torque OFF 中に present position を READ できるか
//   (2) torque ON 復帰をジャークなく行えるか（現在位置READ→ゴール書込→enable）
//   (3) GPIO41(M5.BtnA) の短押し/長押し/ダブルクリック判定
//
// ★ 本ファームは prod(src/main.cpp) とは独立。env: m5stack-atoms3-teach で書き込む。
//   Phase1〜4 は prod に触れないため、SCS/IK 等のコードは本ファイル側に重複する
//   （較正offsetは同一NVS名前空間 "parameter" を共有してデータ二重化は避ける）。
//
// 配線（HW改造済み）: サーボTX=GPIO5(1kΩ直列), RX=GPIO6(バス直結)。
//   GPIO38/39 は内蔵IMUのI2Cのため絶対使用禁止。
#include <M5Unified.h>
#include <Preferences.h>

#define TX_PIN 5
#define RX_PIN 6

// SCS0009 レジスタ
#define REG_GOAL_POSITION 0x2A  // 42: 位置(2)・時間(2)・速度(2)の先頭
#define REG_TORQUE_ENABLE 0x28  // 40: 1=ON / 0=OFF
#define REG_PRESENT_POS   0x38  // 56: 現在位置(2, low byte first)

// 起動時の安全のため低速で。0=最高速(危険)。
int servoSpeed = 200;

Preferences preferences;

//           ID     0    1    2    3    4    5    6    7   （prodと同じ較正値）
int offset[] = { -35,   0,  13,  -7,  53, -44,  15,  34 };
const int POS_MIN[8] = {60, 60, 60, 60, 60, 60, 60, 60};
const int POS_MAX[8] = {960, 960, 960, 960, 960, 960, 960, 960};

bool torqueOn = true;  // 現在のトルク状態（全軸一括管理）

// プロトタイプ（既存prodの慣習に合わせ上部にまとめる）
void scs_moveToPos(byte id, int position);
bool scs_readPosition(byte id, int &outRaw);
void scs_setTorque(byte id, bool on);
void setTorqueAll(bool on);
void readAllPositions();
void torqueOnNoJerk();
void diagReadDump(byte id);
void drawHeader();
void drawStatus(const char* line2);


void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, RX_PIN, TX_PIN);

  // 較正offsetは prod と同じ NVS 名前空間から読む（データ二重化を避ける）
  preferences.begin("parameter", true);  // read-only
  for(int i = 0; i < 8; i++){
    char key[6];
    snprintf(key, sizeof(key), "off%d", i);
    offset[i] = preferences.getInt(key, offset[i]);
  }
  servoSpeed = preferences.getInt("servoSpeed", servoSpeed);
  preferences.end();

  M5.Display.setBrightness(60);
  M5.Display.fillScreen(TFT_BLACK);

  Serial.println("\n=== Teaching Phase0 verify ===");
  Serial.println(" short  : READ all present positions");
  Serial.println(" hold   : TORQUE OFF (脱力)");
  Serial.println(" double : TORQUE ON (無ジャーク復帰)");
  drawStatus("ready");
}


void loop() {
  M5.update();  // ★ prodと違い teach ではボタン判定のため必須

  if(M5.BtnA.wasClicked()){
    Serial.println("[btn] short -> read all");
    readAllPositions();   // 全8軸の現在位置をLCDへ（diagReadDumpは診断用に残置）
  }
  if(M5.BtnA.wasHold()){
    Serial.println("[btn] hold -> torque OFF");
    setTorqueAll(false);
    drawStatus("TORQUE OFF");
  }
  if(M5.BtnA.wasDoubleClicked()){
    Serial.println("[btn] double -> torque ON (no jerk)");
    torqueOnNoJerk();   // 内部でLCDに "ON n/8" を表示する
  }

  delay(10);
}


// ===== SCS0009 通信 =====
// 参考 https://qiita.com/Ninagawa123/items/7b79c5f5117dd1470ac9
void scs_moveToPos(byte id, int position) {
  byte message[13];
  message[0] = 0xFF; message[1] = 0xFF;
  message[2] = id;
  message[3] = 9;                  // データ長
  message[4] = 3;                  // WRITE
  message[5] = REG_GOAL_POSITION;
  message[6] = (position >> 8) & 0xFF;  // posH
  message[7] = position & 0xFF;         // posL
  message[8] = 0x00;  message[9] = 0x00;             // 時間(0)=速度制御
  message[10] = (servoSpeed >> 8) & 0xFF;            // spdH
  message[11] = servoSpeed & 0xFF;                   // spdL
  byte checksum = 0;
  for (int i = 2; i < 12; i++) checksum += message[i];
  message[12] = ~checksum;
  for (int i = 0; i < 13; i++) Serial1.write(message[i]);
}


// トルクON/OFF。レジスタ0x28に 1/0 を書き込む。
//   FF FF ID 04 03 28 EN CS   CS = ~(ID + 04 + 03 + 28 + EN)
void scs_setTorque(byte id, bool on) {
  byte msg[8];
  msg[0] = 0xFF; msg[1] = 0xFF;
  msg[2] = id;
  msg[3] = 0x04;              // データ長
  msg[4] = 0x03;             // WRITE
  msg[5] = REG_TORQUE_ENABLE;
  msg[6] = on ? 0x01 : 0x00;
  byte cs = 0;
  for (int i = 2; i < 7; i++) cs += msg[i];
  msg[7] = ~cs;
  for (int i = 0; i < 8; i++) Serial1.write(msg[i]);
}


// 現在位置(Present Position 0x38, 2byte, low byte first)を読む。
// 半二重シングルワイヤのため送信8byteが自RXに自己エコーする。
// 送信前にflush→ヘッダ(FF FF)探索→checksum検証で本応答のみ採用。タイムアウト20ms。
bool scs_readPosition(byte id, int &outRaw) {
  byte req[8];
  req[0] = 0xFF; req[1] = 0xFF;
  req[2] = id;
  req[3] = 0x04;            // データ長
  req[4] = 0x02;           // READ DATA
  req[5] = REG_PRESENT_POS;
  req[6] = 0x02;           // 2byte
  byte cs = 0;
  for (int i = 2; i < 7; i++) cs += req[i];
  req[7] = ~cs;

  while (Serial1.available()) Serial1.read();   // 古いデータ/自己エコー残りを掃除
  for (int i = 0; i < 8; i++) Serial1.write(req[i]);
  Serial1.flush();                              // 送信完了待ち

  // ★ 自己エコー対策の核心：送信した8byteは必ずRXに返る。READ要求のエコーは
  //   応答用checksum式 ~(id+len+err+dataL+dataH) を偶然満たし 0x238=568 を返すため、
  //   ヘッダ＋checksumでは区別できない。よって先に8byteを「バイト数で」読み捨てる。
  int skipped = 0;
  unsigned long te = millis();
  while (skipped < 8 && millis() - te < 20) {
    if (Serial1.available()) { Serial1.read(); skipped++; }
  }

  unsigned long start = millis();
  int state = 0;  // FF FF 検出
  while (millis() - start < 20) {
    if (!Serial1.available()) continue;
    byte b = Serial1.read();
    if (state < 2) { state = (b == 0xFF) ? state + 1 : 0; continue; }

    byte rid = b;            // ヘッダ直後 = id のはず
    byte rest[5]; int got = 0;
    while (got < 5 && millis() - start < 20) {
      if (Serial1.available()) rest[got++] = Serial1.read();
    }
    if (got < 5) return false;

    // 応答: FF FF id len err posH posL cs（★位置は上位バイト先＝big-endian。
    //   実機ダンプで確定。プロンプトの仕様書「low byte first」は誤りだった）
    byte len = rest[0], err = rest[1], posH = rest[2], posL = rest[3], rcs = rest[4];
    byte calc = (~(rid + len + err + posH + posL)) & 0xFF;
    if (rid == id && len == 0x04 && calc == rcs) {
      int v = (posH << 8) | posL;
      if (v < 0 || v > 1023) return false;  // 範囲外＝フレーム不正として弾く
      outRaw = v;
      return true;
    }
    state = 0;  // 自己エコー等 → ヘッダ探索やり直し
  }
  return false;
}


// ===== 高レベル操作 =====
void setTorqueAll(bool on) {
  for (int id = 0; id < 8; id++) {
    scs_setTorque(id, on);
    delayMicroseconds(1500);  // prod のペーシングに合わせる
  }
  torqueOn = on;
}


// 全8サーボの現在位置をREADしてLCDへ全軸表示（Serialは補助）。
// ★ 筐体組み込み=バッテリー駆動時はUSB不可のため、検証は必ずLCDで完結させる。
//   torque OFF 中に手で動かしながら短押しし、8軸の値が追従すれば「OFF中READ可」の確認。
void readAllPositions() {
  int raw[8];
  bool ok[8];
  Serial.print("present:");
  for (int id = 0; id < 8; id++) {
    ok[id] = scs_readPosition(id, raw[id]);
    if (ok[id]) Serial.printf(" %d=%d", id, raw[id]);
    else        Serial.printf(" %d=FAIL", id);
  }
  Serial.println();

  // LCD: 8軸を2列×4行で表示（左列 ID0-3 / 右列 ID4-7）。
  // 値は最大1023(4桁)になり得るので、桁あふれしないよう文字サイズ1・余裕ある列幅にする。
  M5.Display.fillScreen(TFT_BLACK);
  drawHeader();
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  for (int id = 0; id < 8; id++) {
    int xx = 4 + (id / 4) * 66;   // 左 x=4 / 右 x=70（4桁でも重ならない）
    int yy = 32 + (id % 4) * 18;  // 4行
    M5.Display.setCursor(xx, yy);
    if (ok[id]) M5.Display.printf("%d:%4d", id, raw[id]);
    else        M5.Display.printf("%d:----", id);
  }
}


// 【診断】ID へ READ要求を出し、受信した生バイト列をそのままHEXでLCDに出す。
// echo(送信8byte) + サーボ応答 が連続して見えるはず。これで本当のフレーム構造
// （FF FF の位置・lengthバイト・位置データの並び）を目視確認し、パーサを確定する。
void diagReadDump(byte id) {
  byte req[8];
  req[0]=0xFF; req[1]=0xFF; req[2]=id; req[3]=0x04;
  req[4]=0x02; req[5]=REG_PRESENT_POS; req[6]=0x02;
  byte cs=0; for(int i=2;i<7;i++) cs+=req[i]; req[7]=~cs;

  while (Serial1.available()) Serial1.read();   // 受信バッファ掃除
  for (int i=0;i<8;i++) Serial1.write(req[i]);
  Serial1.flush();

  // 受信を全部キャプチャ（skipせず、echoも応答もまとめて）
  byte buf[32]; int n=0;
  unsigned long t0=millis();
  while (millis()-t0 < 25 && n < 32) {
    if (Serial1.available()) buf[n++]=Serial1.read();
  }

  // Serial（つながっていれば）
  Serial.printf("diag id%d n=%d:", id, n);
  for (int i=0;i<n;i++) Serial.printf(" %02X", buf[i]);
  Serial.println();

  // LCD: 上に件数、以下 6byte/行 でHEX
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Display.setCursor(2,2);
  M5.Display.printf("READ id%d  n=%d", id, n);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  for (int i=0;i<n;i++) {
    int col=i%6, row=i/6;
    M5.Display.setCursor(2 + col*21, 16 + row*13);
    M5.Display.printf("%02X", buf[i]);
  }
}


// 無ジャーク torque ON：①全軸READ → ②現在位置をゴール書込 → ③torque ENABLE。
// ★ READとWRITEを「フェーズ分離」する。1軸ずつ交互にやると書き込みの自己エコーが
//   次のREADに混入して誤った位置をゴールに書き、跳ねる（実機で確認された）。
// ★ READ失敗軸はトルクを入れない（旧ゴールへスナップして跳ねるのを防ぐ安全策）。
void torqueOnNoJerk() {
  int raw[8];
  bool ok[8];
  int nok = 0;

  // ① 全軸READ（読みフェーズを独立させてエコー混入を防ぐ）
  for (int id = 0; id < 8; id++) { ok[id] = scs_readPosition(id, raw[id]); if (ok[id]) nok++; }

  // ② 読めた軸だけ、現在位置をそのままゴールに書く
  for (int id = 0; id < 8; id++) {
    if (!ok[id]) continue;
    int g = constrain(raw[id], POS_MIN[id], POS_MAX[id]);
    scs_moveToPos(id, g);
    delayMicroseconds(1500);
  }

  // ③ 読めた軸だけトルクON（読めない軸は脱力のまま＝跳ね防止）
  for (int id = 0; id < 8; id++) {
    if (!ok[id]) continue;
    scs_setTorque(id, true);
    delayMicroseconds(1500);
  }

  torqueOn = true;
  Serial.printf("torqueOnNoJerk: %d/8 enabled\n", nok);
  char buf[24];
  snprintf(buf, sizeof(buf), "ON %d/8", nok);
  drawStatus(buf);
}


// ===== LCD =====
// トルク状態ヘッダ（緑=ON / オレンジ=OFF）
void drawHeader() {
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(torqueOn ? TFT_GREEN : TFT_ORANGE, TFT_BLACK);
  M5.Display.setCursor(2, 2);
  M5.Display.print(torqueOn ? "TORQUE ON " : "TORQUE OFF");
}

void drawStatus(const char* line2) {
  M5.Display.fillScreen(TFT_BLACK);
  drawHeader();
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(2, 30);
  M5.Display.print("Phase0 verify");
  M5.Display.setCursor(2, 50);
  M5.Display.print(line2);
}
