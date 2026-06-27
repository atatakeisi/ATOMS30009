// ATOMS30009 Teaching ファーム — Phase 1（キーフレームTeaching中核 ＋ WiFi WebUI）
// 目的: 手で姿勢を作り（torque OFF）、コマ(raw[8])を記録し、Linear補間で再生する。
//   ATOMの画面は小さいので、iPad等のブラウザから WiFi AP 経由でUIを見て操作する。
//
// ★ prod(src/main.cpp) とは独立。env: m5stack-atoms3-teach で書き込む。
//   SCS/IK等のコードは重複するが、較正offsetは同一NVS名前空間 "parameter" を共有。
//
// 設計方針（docs/DESIGN.md）:
//   - 操作系は全てPOST（GET先読みでの誤発火防止）。ページ表示 / と /status のみGET。
//   - ライブ表示は /status(JSON) を数Hzポーリング。
//   - サーボバス(半二重1Mbps)アクセスは loop 1スレッドに集約して競合を避ける。
//   - 形(raw)だけ記録。重力下の力学は将来 Phase1.5 の計測再生で別取得する。
//
// 配線(HW改造済み): TX=GPIO5(1kΩ直列), RX=GPIO6(バス直結)。GPIO38/39は禁止。
#include <M5Unified.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>

#define TX_PIN 5
#define RX_PIN 6

// SCS0009 レジスタ
#define REG_GOAL_POSITION 0x2A  // 42
#define REG_TORQUE_ENABLE 0x28  // 40 (1=ON / 0=OFF)
#define REG_PRESENT_POS   0x38  // 56 (応答は上位バイト先=big-endian)

int servoSpeed = 200;  // 0=最高速(危険)。低速で安全に。

Preferences preferences;

//           ID     0    1    2    3    4    5    6    7  （prodと同じ較正値）
int offset[] = { -35,   0,  13,  -7,  53, -44,  15,  34 };
const int POS_MIN[8] = {60, 60, 60, 60, 60, 60, 60, 60};
const int POS_MAX[8] = {960, 960, 960, 960, 960, 960, 960, 960};
const char* LBL[8] = {"RL肩","RR肩","RL膝","RR膝","FR肩","FL肩","FR膝","FL膝"};

bool torqueOn = true;

// 現在位置キャッシュ（loopで定期READし、/statusはこれを返す＝バス連射を防ぐ）
int  curRaw[8] = {511,511,511,511,511,511,511,511};
bool curOk[8]  = {false,false,false,false,false,false,false,false};

// キーフレーム（raw[8]）。Phase2でLittleFS永続化予定。今はRAMのみ。
#define MAX_FRAMES 64
uint16_t frames[MAX_FRAMES][8];
int frameTransMs[MAX_FRAMES];   // 各コマへ「前コマ(or現在位置)から移行する時間」(ms)
int frameCount = 0;
int defTransMs = 600;           // 新規コマ記録時の既定移行時間(ms)

// 再生（ノンブロッキング状態機械）。server応答とライブ表示を止めないため、
// 1ステップずつ loop で進める。これにより「再生中コマ」をiPadへ反映できる。
bool playing = false;
int  playSeg = 0;               // 現在の移行先コマindex（=ハイライト対象）
int  playStep = 0, playSteps = 0;
uint16_t playFrom[8];
unsigned long playLast = 0;
const int PLAY_STEP_MS = 15;

// WiFi AP（prodと同IP。SSIDはteachと分かるよう別名）
WebServer server(80);
const char ssid[] = "robot0009-teach";
const char pass[] = "password";
const IPAddress apIp(192, 168, 55, 27);
const IPAddress apSubnet(255, 255, 255, 0);

// プロトタイプ
void scs_moveToPos(byte id, int position);
bool scs_readPosition(byte id, int &outRaw);
void scs_setTorque(byte id, bool on);
void setTorqueAll(bool on);
void torqueOnNoJerk();
void refreshCache();
void recordFrame();
void undoFrame();
void clearFrames();
void deleteFrame(int idx);
void startPlay();
void tickPlay();
void stopPlay();
void drawLcd();
// web
void handleRoot();
void handleStatus();
void handleTorqueOff();
void handleTorqueOn();
void handleRecord();
void handleUndo();
void handleClear();
void handlePlay();
void handleTransM();
void handleTransP();
void handleFrameTransM();
void handleFrameTransP();
void handleDeleteFrame();
void handleStop();
void handleExport();


void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, RX_PIN, TX_PIN);

  preferences.begin("parameter", true);  // prodと同じ名前空間からoffsetを読む
  for (int i = 0; i < 8; i++) {
    char key[6];
    snprintf(key, sizeof(key), "off%d", i);
    offset[i] = preferences.getInt(key, offset[i]);
  }
  servoSpeed = preferences.getInt("servoSpeed", servoSpeed);
  preferences.end();

  M5.Display.setBrightness(60);
  M5.Display.fillScreen(TFT_BLACK);

  WiFi.softAP(ssid, pass);
  delay(100);
  WiFi.softAPConfig(apIp, apIp, apSubnet);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/torqueOff", HTTP_POST, handleTorqueOff);
  server.on("/torqueOn", HTTP_POST, handleTorqueOn);
  server.on("/record", HTTP_POST, handleRecord);
  server.on("/undo", HTTP_POST, handleUndo);
  server.on("/clear", HTTP_POST, handleClear);
  server.on("/play", HTTP_POST, handlePlay);
  server.on("/transM", HTTP_POST, handleTransM);
  server.on("/transP", HTTP_POST, handleTransP);
  server.on("/frameTransM", HTTP_POST, handleFrameTransM);
  server.on("/frameTransP", HTTP_POST, handleFrameTransP);
  server.on("/deleteFrame", HTTP_POST, handleDeleteFrame);
  server.on("/stop", HTTP_POST, handleStop);
  server.on("/export", HTTP_GET, handleExport);  // GET=ダウンロード（副作用なし）
  server.begin();

  Serial.printf("Teaching Phase1 up. AP=%s IP=%s\n", ssid, apIp.toString().c_str());
  drawLcd();
}


void loop() {
  M5.update();  // ボタン判定に必須

  // 物理ボタン（iPadが無くても基本操作ができるよう残す）。再生中は無効。
  if (!playing) {
    if (M5.BtnA.wasClicked())       { recordFrame(); drawLcd(); }   // 短押し: コマ記録
    if (M5.BtnA.wasHold())          { setTorqueAll(false); drawLcd(); }  // 長押し: 脱力
    if (M5.BtnA.wasDoubleClicked()) { torqueOnNoJerk(); drawLcd(); }     // ダブル: 無ジャークON
  }

  server.handleClient();
  tickPlay();   // 再生中なら1ステップ進める（serverを止めない）

  // 現在位置キャッシュを定期更新（約250ms）。iPadのライブ表示用。再生中はバスを再生に使うので休止。
  static unsigned long lastRead = 0;
  if (!playing && millis() - lastRead >= 250) {
    lastRead = millis();
    refreshCache();
    drawLcd();
  }

  delay(2);
}


// ===== SCS0009 通信 =====
void scs_moveToPos(byte id, int position) {
  byte m[13];
  m[0]=0xFF; m[1]=0xFF; m[2]=id; m[3]=9; m[4]=3; m[5]=REG_GOAL_POSITION;
  m[6]=(position>>8)&0xFF; m[7]=position&0xFF;       // posH, posL
  m[8]=0x00; m[9]=0x00;                              // 時間0=速度制御
  m[10]=(servoSpeed>>8)&0xFF; m[11]=servoSpeed&0xFF; // spdH, spdL
  byte cs=0; for(int i=2;i<12;i++) cs+=m[i]; m[12]=~cs;
  for(int i=0;i<13;i++) Serial1.write(m[i]);
}

void scs_setTorque(byte id, bool on) {
  byte m[8];
  m[0]=0xFF; m[1]=0xFF; m[2]=id; m[3]=0x04; m[4]=0x03; m[5]=REG_TORQUE_ENABLE;
  m[6]= on ? 0x01 : 0x00;
  byte cs=0; for(int i=2;i<7;i++) cs+=m[i]; m[7]=~cs;
  for(int i=0;i<8;i++) Serial1.write(m[i]);
}

// 現在位置READ。応答 FF FF id 04 err posH posL cs（位置は上位バイト先=big-endian）。
// 半二重の自己エコー(送信8byte)を先にバイト数で読み捨ててから応答を探す。タイムアウト20ms。
bool scs_readPosition(byte id, int &outRaw) {
  byte req[8];
  req[0]=0xFF; req[1]=0xFF; req[2]=id; req[3]=0x04;
  req[4]=0x02; req[5]=REG_PRESENT_POS; req[6]=0x02;
  byte cs=0; for(int i=2;i<7;i++) cs+=req[i]; req[7]=~cs;

  while (Serial1.available()) Serial1.read();
  for (int i=0;i<8;i++) Serial1.write(req[i]);
  Serial1.flush();

  // 自己エコー8byteを読み捨て（READ要求のエコーは応答checksum式を偶然満たすため必須）
  int skipped=0; unsigned long te=millis();
  while (skipped<8 && millis()-te<20) { if(Serial1.available()){Serial1.read();skipped++;} }

  unsigned long start=millis();
  int state=0;
  while (millis()-start < 20) {
    if (!Serial1.available()) continue;
    byte b=Serial1.read();
    if (state<2) { state=(b==0xFF)?state+1:0; continue; }
    byte rid=b; byte rest[5]; int got=0;
    while (got<5 && millis()-start<20) { if(Serial1.available()) rest[got++]=Serial1.read(); }
    if (got<5) return false;
    byte len=rest[0], err=rest[1], posH=rest[2], posL=rest[3], rcs=rest[4];
    byte calc=(~(rid+len+err+posH+posL))&0xFF;
    if (rid==id && len==0x04 && calc==rcs) {
      int v=(posH<<8)|posL;          // big-endian
      if (v<0 || v>1023) return false;
      outRaw=v; return true;
    }
    state=0;
  }
  return false;
}


// ===== 高レベル操作 =====
void setTorqueAll(bool on) {
  for (int id=0; id<8; id++) { scs_setTorque(id,on); delayMicroseconds(1500); }
  torqueOn = on;
}

// 無ジャークON：①全軸READ ②現在位置をゴール書込 ③torque ENABLE（フェーズ分離）。
// READ失敗軸はトルクを入れない（旧ゴールへ跳ねるのを防ぐ）。
void torqueOnNoJerk() {
  int raw[8]; bool ok[8];
  for (int id=0; id<8; id++) ok[id]=scs_readPosition(id, raw[id]);
  for (int id=0; id<8; id++) { if(!ok[id])continue; scs_moveToPos(id, constrain(raw[id],POS_MIN[id],POS_MAX[id])); delayMicroseconds(1500); }
  for (int id=0; id<8; id++) { if(!ok[id])continue; scs_setTorque(id,true); delayMicroseconds(1500); }
  torqueOn = true;
}

// 現在位置キャッシュ更新（読めた軸だけ更新、失敗は前回値を保持）
void refreshCache() {
  for (int id=0; id<8; id++) {
    int r;
    curOk[id] = scs_readPosition(id, r);
    if (curOk[id]) curRaw[id] = r;
  }
}


// ===== キーフレーム =====
void recordFrame() {
  if (frameCount >= MAX_FRAMES) return;
  for (int id=0; id<8; id++) {
    int r;
    frames[frameCount][id] = scs_readPosition(id, r) ? (uint16_t)r
                                                     : (uint16_t)curRaw[id];  // 失敗は直近キャッシュ
  }
  frameTransMs[frameCount] = defTransMs;   // 新規コマは既定の移行時間で初期化（後で個別調整可）
  frameCount++;
  Serial.printf("record: frame %d\n", frameCount);
}

void undoFrame()  { if (frameCount>0) frameCount--; }
void clearFrames(){ frameCount = 0; }

// 指定コマを削除して後ろを詰める
void deleteFrame(int idx) {
  if (idx<0 || idx>=frameCount) return;
  for (int i=idx; i<frameCount-1; i++) {
    memcpy(frames[i], frames[i+1], sizeof(frames[i]));
    frameTransMs[i] = frameTransMs[i+1];
  }
  frameCount--;
}

// 再生開始（ノンブロッキング）。現在位置→frame0→frame1→… をLinearで。開始は無ジャーク。
void startPlay() {
  if (frameCount < 1 || playing) return;
  for (int id=0; id<8; id++) { int r; playFrom[id] = scs_readPosition(id,r) ? (uint16_t)r : (uint16_t)(511+offset[id]); }
  // 現在位置をゴールに書いてからトルクON（無ジャーク）
  for (int id=0; id<8; id++) { scs_moveToPos(id, constrain(playFrom[id],POS_MIN[id],POS_MAX[id])); delayMicroseconds(1500); }
  setTorqueAll(true);
  playSeg = 0; playStep = 0;
  playSteps = frameTransMs[0] / PLAY_STEP_MS; if (playSteps < 1) playSteps = 1;
  playing = true; playLast = millis();
}

// 再生を1ステップ進める（loopから毎回呼ぶ。PLAY_STEP_MS間隔で実行）。
void tickPlay() {
  if (!playing) return;
  if (millis() - playLast < PLAY_STEP_MS) return;
  playLast = millis();
  playStep++;
  for (int id=0; id<8; id++) {
    int p = (int)playFrom[id] + ((int)frames[playSeg][id]-(int)playFrom[id]) * playStep / playSteps;
    p = constrain(p, POS_MIN[id], POS_MAX[id]);
    scs_moveToPos(id, p); delayMicroseconds(1500);
  }
  if (playStep >= playSteps) {            // この区間完了 → 次コマへ
    for (int id=0; id<8; id++) playFrom[id] = frames[playSeg][id];
    playSeg++;
    if (playSeg >= frameCount) { playing = false; return; }
    playStep = 0;
    playSteps = frameTransMs[playSeg] / PLAY_STEP_MS; if (playSteps < 1) playSteps = 1;
  }
}

void stopPlay() { playing = false; }


// ===== LCD（確認用。詳細はiPad） =====
void drawLcd() {
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(torqueOn ? TFT_GREEN : TFT_ORANGE, TFT_BLACK);
  M5.Display.setCursor(2, 2);
  M5.Display.print(torqueOn ? "TRQ ON" : "TRQ OFF");
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Display.setCursor(2, 24);
  M5.Display.print(apIp.toString());
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setCursor(2, 38);
  M5.Display.printf("frames:%d", frameCount);
  for (int id=0; id<8; id++) {
    int xx = 2 + (id/4)*66, yy = 54 + (id%4)*16;
    M5.Display.setCursor(xx, yy);
    if (curOk[id]) M5.Display.printf("%d:%4d", id, curRaw[id]);
    else           M5.Display.printf("%d:----", id);
  }
}


// ===== Web =====
void handleStatus() {
  String j = "{\"torque\":"; j += (torqueOn ? "true" : "false");
  j += ",\"frames\":"; j += frameCount;
  j += ",\"defMs\":"; j += defTransMs;
  j += ",\"playing\":"; j += (playing ? "true" : "false");
  j += ",\"playIdx\":"; j += (playing ? playSeg : -1);
  j += ",\"raw\":[";  for(int i=0;i<8;i++){ if(i)j+=","; j+=curRaw[i]; }
  j += "],\"ok\":[";  for(int i=0;i<8;i++){ if(i)j+=","; j+=(curOk[i]?"1":"0"); }
  j += "],\"fr\":[";
  for (int i=0;i<frameCount;i++) {
    if(i) j+=",";
    j += "{\"t\":"; j += frameTransMs[i]; j += ",\"r\":[";
    for(int k=0;k<8;k++){ if(k)j+=","; j+=frames[i][k]; }
    j += "]}";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

void handleTorqueOff() { setTorqueAll(false); server.send(200,"text/plain","ok"); }
void handleTorqueOn()  { torqueOnNoJerk();    server.send(200,"text/plain","ok"); }
void handleRecord()    { recordFrame();       server.send(200,"text/plain","ok"); }
void handleUndo()      { undoFrame();          server.send(200,"text/plain","ok"); }
void handleClear()     { clearFrames();        server.send(200,"text/plain","ok"); }
void handlePlay()      { startPlay();           server.send(200,"text/plain","ok"); }
void handleStop()      { stopPlay();            server.send(200,"text/plain","ok"); }

// 現在のシーケンスを自己完結JSONでダウンロード（Python解析・参照ライブラリ用）。
// 較正offset・L1/L2・符号規則を同梱するので、これ単体で raw→角度→脚先 まで再現できる。
void handleExport() {
  String j = "{\"name\":\"current\",\"defMs\":"; j += defTransMs;
  j += ",\"calib\":{\"offset\":[";
  for(int i=0;i<8;i++){ if(i)j+=","; j+=offset[i]; }
  j += "],\"L1\":50.0,\"L2\":65.0,\"sign\":\"RL/FR:+ , RR/FL:-\"}";
  j += ",\"frames\":[";
  for(int i=0;i<frameCount;i++){
    if(i)j+=",";
    j += "{\"t\":"; j += frameTransMs[i]; j += ",\"raw\":[";
    for(int k=0;k<8;k++){ if(k)j+=","; j+=frames[i][k]; }
    j += "]}";
  }
  j += "]}";
  server.sendHeader("Content-Disposition", "attachment; filename=robot0009_seq.json");
  server.send(200, "application/json", j);
}
void handleTransM()    { if(defTransMs>100) defTransMs-=100; server.send(200,"text/plain","ok"); }
void handleTransP()    { if(defTransMs<5000) defTransMs+=100; server.send(200,"text/plain","ok"); }
// 各コマの移行時間 ±（クエリ i= でコマ番号指定）
void handleFrameTransM() { int i=server.arg("i").toInt(); if(i>=0&&i<frameCount&&frameTransMs[i]>100)  frameTransMs[i]-=100; server.send(200,"text/plain","ok"); }
void handleFrameTransP() { int i=server.arg("i").toInt(); if(i>=0&&i<frameCount&&frameTransMs[i]<5000) frameTransMs[i]+=100; server.send(200,"text/plain","ok"); }
void handleDeleteFrame() { deleteFrame(server.arg("i").toInt()); server.send(200,"text/plain","ok"); }

void handleRoot() {
  String h = "<!DOCTYPE html><html lang='ja'><head><meta charset='utf-8'>";
  h += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<title>robot0009 Teaching</title><style>";
  h += "body{font-family:sans-serif;text-align:center;margin:0;padding:10px;font-size:18px;}";
  h += "h2{margin:6px;} .trq{font-size:24px;font-weight:bold;padding:6px;border-radius:8px;}";
  h += ".on{background:#19d219;color:#fff;} .off{background:#ff9800;color:#fff;}";
  h += "table{margin:8px auto;border-collapse:collapse;} td{border:1px solid #ccc;padding:6px 10px;font-variant-numeric:tabular-nums;}";
  h += "button{font-size:18px;font-weight:bold;margin:5px;padding:12px 16px;border-radius:10px;border:1px solid #888;}";
  h += ".rec{background:#3a7bff;color:#fff;} .play{background:#19d219;color:#fff;} .danger{background:#ffd2d2;} .calib{background:#e0e0e0;}";
  h += ".sm{font-size:14px;padding:6px 10px;margin:2px;}";
  h += ".fr{border:1px solid #ccc;border-radius:8px;margin:6px auto;padding:6px;max-width:460px;}";
  h += ".fr small{color:#666;font-variant-numeric:tabular-nums;}";
  h += ".now{border:3px solid #19d219;background:#eaffea;}";       // 再生中コマの強調枠
  h += ".nowtag{color:#19d219;font-weight:bold;margin-left:6px;}"; // ▶再生中タグ
  h += ".exp{background:#fff3c2;}";
  h += ".sel{outline:4px solid #1565c0;box-shadow:0 0 0 4px #90caf9;}"; // 有効中ボタンの強調
  h += "button:active{filter:brightness(.82);transform:scale(.96);}";   // 押下フィードバック
  h += "</style></head><body>";
  h += "<h2>robot0009 Teaching</h2>";
  h += "<div id='trq' class='trq'>...</div>";
  h += "<p>新規コマの既定移行 <button class='sm' onclick=\"act('/transM')\">-</button><b id='tm'>0</b>ms<button class='sm' onclick=\"act('/transP')\">+</button></p>";
  h += "<table id='tbl'></table>";
  h += "<p><button id='bOff' class='calib' onclick=\"act('/torqueOff')\">脱力(OFF)</button>";
  h += "<button id='bOn' class='calib' onclick=\"act('/torqueOn')\">保持(無ジャークON)</button></p>";
  h += "<p><button class='rec' onclick=\"act('/record')\">このコマを記録</button>";
  h += "<button class='danger' onclick=\"act('/undo')\">最後を削除</button>";
  h += "<button class='danger' onclick=\"if(confirm('全消去?'))act('/clear')\">全消去</button></p>";
  h += "<p><button id='bPlay' class='play' onclick=\"if(confirm('再生します。手で押さえて下さい'))act('/play')\">▶ 再生</button>";
  h += "<button class='danger' onclick=\"act('/stop')\">■ 停止</button>";
  h += "<button class='exp' onclick=\"location.href='/export'\">⬇ エクスポート(JSON)</button></p>";
  h += "<h3>コマ一覧</h3><div id='list'></div>";
  h += "<script>";
  h += "const LBL=['RL肩','RR肩','RL膝','RR膝','FR肩','FL肩','FR膝','FL膝'];";
  h += "async function act(p){await fetch(p,{method:'POST'});poll();}";
  h += "async function poll(){try{let j=await(await fetch('/status')).json();";
  h += "let t=document.getElementById('trq');t.textContent=j.torque?'TORQUE ON':'TORQUE OFF';t.className='trq '+(j.torque?'on':'off');";
  h += "document.getElementById('bOff').className='calib'+(!j.torque?' sel':'');";   // 脱力中=強調
  h += "document.getElementById('bOn').className='calib'+(j.torque?' sel':'');";      // 保持中=強調
  h += "document.getElementById('bPlay').className='play'+(j.playing?' sel':'');";    // 再生中=強調
  h += "document.getElementById('fc')&&(document.getElementById('fc').textContent=j.frames);document.getElementById('tm').textContent=j.defMs;";
  h += "let s='<tr><td>ID</td><td>部位</td><td>現在raw</td></tr>';";
  h += "for(let i=0;i<8;i++){s+='<tr><td>'+i+'</td><td>'+LBL[i]+'</td><td>'+(j.ok[i]?j.raw[i]:'--')+'</td></tr>';}";
  h += "document.getElementById('tbl').innerHTML=s;";
  h += "let f='';for(let i=0;i<j.fr.length;i++){let fr=j.fr[i];";
  h += "let now=(j.playing&&i==j.playIdx);";
  h += "f+=\"<div class='fr\"+(now?' now':'')+\"'><b>コマ\"+(i+1)+\"</b>\"+(now?\"<span class='nowtag'>▶再生中</span>\":'')+\" 移行 <button class='sm' onclick=\\\"act('/frameTransM?i=\"+i+\"')\\\">-</button>\"+fr.t+\"ms<button class='sm' onclick=\\\"act('/frameTransP?i=\"+i+\"')\\\">+</button> \";";
  h += "f+=\"<button class='danger sm' onclick=\\\"act('/deleteFrame?i=\"+i+\"')\\\">削除</button><br><small>\"+fr.r.join(' ')+\"</small></div>\";}";
  h += "document.getElementById('list').innerHTML=f;}catch(e){}}";
  h += "setInterval(poll,300);poll();";
  h += "</script></body></html>";
  server.send(200, "text/html", h);
}
