#include <Arduino.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>

#define TX_PIN   38
#define CENTER   511
#define MS_MOVE  400
#define MS_WALK  280
#define MS_STR   600

// ============================================================
// サーボ配置（実機確認済み）
//   ID 4: 右前・肩   ID 6: 右前・膝
//   ID 5: 左前・肩   ID 7: 左前・膝
//   ID 1: 右後・肩   ID 3: 右後・膝
//   ID 0: 左後・肩   ID 2: 左後・膝
//
// 対角グループ（歩行用）
//   GA = 右前 + 左後  (4,6,0,2)
//   GB = 左前 + 右後  (5,7,1,3)
//
// 肩だけ / 膝だけ グループ
//   SHOULDER_A = GA の肩 (4,0)
//   SHOULDER_B = GB の肩 (5,1)
//   KNEE_A     = GA の膝 (6,2)
//   KNEE_B     = GB の膝 (7,3)
// ============================================================

// ── 対角グループ ──────────────────────────────────────────────
const uint8_t GA[]         = {4,6,0,2};   // 右前+左後（肩+膝）
const uint8_t GB[]         = {5,7,1,3};   // 左前+右後（肩+膝）
const uint8_t SHOULDER_A[] = {4,0};       // GAの肩のみ
const uint8_t SHOULDER_B[] = {5,1};       // GBの肩のみ
const uint8_t KNEE_A[]     = {6,2};       // GAの膝のみ
const uint8_t KNEE_B[]     = {7,3};       // GBの膝のみ

// ── 左右グループ（旋回用）────────────────────────────────────
const uint8_t GL_SH[] = {5,0};   // 左側・肩 (左前5, 左後0)
const uint8_t GR_SH[] = {4,1};   // 右側・肩 (右前4, 右後1)
const uint8_t GL_KN[] = {7,2};   // 左側・膝 (左前7, 左後2)
const uint8_t GR_KN[] = {6,3};   // 右側・膝 (右前6, 右後3)

// ============================================================
// 物理限界（実機で確認して調整してください）
// STANDBY値を基準に ±100 を仮設定しています
// ============================================================
const int POS_MIN[8] = {
  276,  // ID0: 左後・肩  (standby 376 - 100)
  536,  // ID1: 右後・肩  (standby 636 - 100)
  791,  // ID2: 左後・膝  (standby 891 - 100)
   36,  // ID3: 右後・膝  (standby 136 - 100)
  326,  // ID4: 右前・肩  (standby 426 - 100)
  471,  // ID5: 左前・肩  (standby 571 - 100)
  821,  // ID6: 右前・膝  (standby 921 - 100)
   66   // ID7: 左前・膝  (standby 166 - 100)
};
const int POS_MAX[8] = {
  476,  // ID0: 左後・肩  (standby 376 + 100)
  736,  // ID1: 右後・肩  (standby 636 + 100)
  991,  // ID2: 左後・膝  (standby 891 + 100)
  236,  // ID3: 右後・膝  (standby 136 + 100)
  526,  // ID4: 右前・肩  (standby 426 + 100)
  671,  // ID5: 左前・肩  (standby 571 + 100)
 1023,  // ID6: 右前・膝  (standby 921 + 100、上限1023)
  266   // ID7: 左前・膝  (standby 166 + 100)
};

// ── 歩行ストライド量（パルス値）────────────────────────────
// STANDBY肩値から±この値だけ動かす
#define STRIDE   60   // 肩の前後ストライド
#define LIFT     50   // 膝の持ち上げ量

AsyncWebServer server(80);
AsyncWebSocket  ws("/ws");
const char* ssid = "robot0009";
const char* pass = "password";
const IPAddress ip(192,168,55,27);
const IPAddress subnet(255,255,255,0);

#define M_IDLE    0
#define M_SWEEP   1
#define M_STEP    2
#define M_FWD     3
#define M_BACK    4
#define M_LEFT    5
#define M_RIGHT   6
#define M_STRETCH 7
#define M_FOLD    8

volatile uint8_t mode = M_IDLE;
unsigned long txCount = 0;

// ── NVS: スタンバイ姿勢(p0-p7) + 収納姿勢(s0-s7) ────────────
uint16_t standbyPos[8];
uint16_t storagePos[8];
Preferences prefs;

void loadPrefs() {
  prefs.begin("robot", true);
  for (int i=0; i<8; i++) {
    char key[4];
    snprintf(key,sizeof(key),"p%d",i);
    standbyPos[i] = prefs.getUShort(key, CENTER);
    snprintf(key,sizeof(key),"s%d",i);
    storagePos[i] = prefs.getUShort(key, CENTER);
  }
  prefs.end();
}
void saveStandby() {
  prefs.begin("robot", false);
  for (int i=0; i<8; i++) {
    char key[4]; snprintf(key,sizeof(key),"p%d",i);
    prefs.putUShort(key, standbyPos[i]);
  }
  prefs.end();
}
void saveStorage() {
  prefs.begin("robot", false);
  for (int i=0; i<8; i++) {
    char key[4]; snprintf(key,sizeof(key),"s%d",i);
    prefs.putUShort(key, storagePos[i]);
  }
  prefs.end();
}

// ── 送信（物理限界クランプ付き）──────────────────────────────
void sendPos(uint8_t id, int pos, int ms) {
  if (id < 8) {
    pos = constrain(pos, POS_MIN[id], POS_MAX[id]); // 物理限界
  }
  pos = constrain(pos, 0, 1023); // 電気的限界（念のため）
  byte msg[13];
  msg[0]=0xFF; msg[1]=0xFF;
  msg[2]=id;   msg[3]=9;
  msg[4]=3;    msg[5]=42;
  msg[6]=(pos>>8)&0xFF; msg[7]=pos&0xFF;
  msg[8]=(ms>>8)&0xFF;  msg[9]=ms&0xFF;
  msg[10]=0; msg[11]=0;
  byte sum=0; for(int i=2;i<12;i++) sum+=msg[i]; msg[12]=~sum;
  Serial1.write(msg,13); Serial1.flush(); txCount++;
}

void sendTorque(uint8_t id, bool enable) {
  byte msg[8];
  msg[0]=0xFF; msg[1]=0xFF;
  msg[2]=id;   msg[3]=4;
  msg[4]=3;    msg[5]=40;
  msg[6]= enable ? 1 : 0;
  byte sum=0; for(int i=2;i<7;i++) sum+=msg[i]; msg[7]=~sum;
  Serial1.write(msg,8); Serial1.flush();
}

void sendGrp(const uint8_t* ids, int n, int pos, int ms) {
  for (int i=0; i<n; i++) sendPos(ids[i], pos, ms);
}

// ── スタンバイから相対値で送る ───────────────────────────────
void sendRelPos(uint8_t id, int offset, int ms) {
  sendPos(id, (int)standbyPos[id] + offset, ms);
}
void sendRelGrp(const uint8_t* ids, int n, int offset, int ms) {
  for (int i=0; i<n; i++) sendRelPos(ids[i], offset, ms);
}

// ── 膝の「内側」「外側」方向（サーボ取付向きが左右で逆）────
// 内側（足を持ち上げる方向）
//   ID2(左後膝), ID6(右前膝) → +増やすと内側
//   ID3(右後膝), ID7(左前膝) → -減らすと内側
void kneeIn(uint8_t id, int amount, int ms) {
  if (id == 2 || id == 6) sendRelPos(id, -amount, ms);
  else                     sendRelPos(id, -amount, ms);
}
void kneeOut(uint8_t id, int amount, int ms) {
  if (id == 2 || id == 6) sendRelPos(id, +amount, ms);
  else                     sendRelPos(id, +amount, ms);
}
void kneeStandby(uint8_t id, int ms) { sendRelPos(id, 0, ms); }

// GA膝(ID6,ID2) / GB膝(ID7,ID3) まとめて操作
void kneeInGA(int v, int ms)  { kneeIn(6,v,ms);  kneeIn(2,v,ms);  }
void kneeInGB(int v, int ms)  { kneeIn(7,v,ms);  kneeIn(3,v,ms);  }
void kneeOutGA(int v, int ms) { kneeOut(6,v,ms); kneeOut(2,v,ms); }
void kneeOutGB(int v, int ms) { kneeOut(7,v,ms); kneeOut(3,v,ms); }
void kneeStdGA(int ms) { kneeStandby(6,ms); kneeStandby(2,ms); }
void kneeStdGB(int ms) { kneeStandby(7,ms); kneeStandby(3,ms); }

// 左側膝(ID7,ID2) / 右側膝(ID6,ID3)
void kneeInGL(int v, int ms)  { kneeIn(7,v,ms);  kneeIn(2,v,ms);  }
void kneeInGR(int v, int ms)  { kneeIn(6,v,ms);  kneeIn(3,v,ms);  }
void kneeStdGL(int ms) { kneeStandby(7,ms); kneeStandby(2,ms); }
void kneeStdGR(int ms) { kneeStandby(6,ms); kneeStandby(3,ms); }

void goStandby() { for(int i=0;i<8;i++) sendPos(i, standbyPos[i], 600); }
void goStorage()  { for(int i=0;i<8;i++) sendPos(i, storagePos[i], 600); }

// ── 待機（モード変更で即抜ける）──────────────────────────────
static bool waitM(uint8_t m, int ms) {
  unsigned long end = millis() + ms;
  while ((long)(millis() - end) < 0) {
    if (mode != m) return false;
    delay(10);
  }
  return true;
}

void pushState() {
  char buf[48];
  snprintf(buf,sizeof(buf),"{\"mode\":%d,\"tx\":%lu}",mode,txCount);
  ws.textAll(buf);
}

// ── WebSocket ─────────────────────────────────────────────────
void onWsEvent(AsyncWebSocket* w, AsyncWebSocketClient* cl,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  w->cleanupClients();
  if (type == WS_EVT_CONNECT) {
    char buf[48];
    snprintf(buf,sizeof(buf),"{\"mode\":%d,\"tx\":%lu}",mode,txCount);
    cl->text(buf); return;
  }
  if (type != WS_EVT_DATA) return;
  AwsFrameInfo* info = (AwsFrameInfo*)arg;
  if (!info->final||info->index!=0||info->len!=len||info->opcode!=WS_TEXT) return;

  char buf[20];
  size_t n = min(len,(size_t)19);
  memcpy(buf,data,n); buf[n]='\0';

  int cfgId, cfgVal;

  if (sscanf(buf,"cfg:%d:%d",&cfgId,&cfgVal)==2) {
    if (cfgId>=0&&cfgId<=7&&cfgVal>=0&&cfgVal<=1023) {
      standbyPos[cfgId]=(uint16_t)cfgVal;
      sendTorque(cfgId,true); sendPos(cfgId,cfgVal,300);
    }
    return;
  }
  if (sscanf(buf,"scfg:%d:%d",&cfgId,&cfgVal)==2) {
    if (cfgId>=0&&cfgId<=7&&cfgVal>=0&&cfgVal<=1023) {
      storagePos[cfgId]=(uint16_t)cfgVal;
      sendTorque(cfgId,true); sendPos(cfgId,cfgVal,300);
    }
    return;
  }

  if (!strcmp(buf,"save"))    { saveStandby(); ws.textAll("{\"saved\":true}"); return; }
  if (!strcmp(buf,"ssave"))   { saveStorage(); ws.textAll("{\"saved\":true}"); return; }

  if (!strcmp(buf,"getcfg")) {
    char resp[80];
    snprintf(resp,sizeof(resp),"{\"cfg\":[%d,%d,%d,%d,%d,%d,%d,%d]}",
      standbyPos[0],standbyPos[1],standbyPos[2],standbyPos[3],
      standbyPos[4],standbyPos[5],standbyPos[6],standbyPos[7]);
    cl->text(resp); return;
  }
  if (!strcmp(buf,"getscfg")) {
    char resp[80];
    snprintf(resp,sizeof(resp),"{\"scfg\":[%d,%d,%d,%d,%d,%d,%d,%d]}",
      storagePos[0],storagePos[1],storagePos[2],storagePos[3],
      storagePos[4],storagePos[5],storagePos[6],storagePos[7]);
    cl->text(resp); return;
  }

  if (!strcmp(buf,"stopall")) { mode=M_IDLE; pushState(); return; }

  if (!strcmp(buf,"center")) {
    if (mode==M_FOLD) { sendTorque(0xFE,true); delay(100); }
    mode=M_IDLE; goStandby(); pushState(); return;
  }

  if (!strcmp(buf,"fold")) {
    if (mode==M_FOLD) {
      sendTorque(0xFE,true); delay(100);
      mode=M_IDLE; goStandby();
    } else {
      mode=M_FOLD;
      goStorage();
      delay(800);
      sendTorque(0xFE,false);
    }
    pushState(); return;
  }

  if (mode==M_FOLD) { sendTorque(0xFE,true); delay(100); }
  uint8_t req=M_IDLE;
  if      (!strcmp(buf,"sweep"))   req=M_SWEEP;
  else if (!strcmp(buf,"step"))    req=M_STEP;
  else if (!strcmp(buf,"fwd"))     req=M_FWD;
  else if (!strcmp(buf,"back"))    req=M_BACK;
  else if (!strcmp(buf,"left"))    req=M_LEFT;
  else if (!strcmp(buf,"right"))   req=M_RIGHT;
  else if (!strcmp(buf,"stretch")) req=M_STRETCH;
  mode=(mode==req) ? M_IDLE : req;
  pushState();
}

static const char* modeStr[]={"Standby","Sweep","Step","Fwd","Back","Left","Right","Stretch","FOLD"};
void drawDisp() {
  M5.Display.fillScreen(0x0000);
  M5.Display.setTextColor(0xFFFF,0x0000);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(0,0);
  M5.Display.println("robot0009");
  M5.Display.println(modeStr[mode]);
  M5.Display.print("IP:"); M5.Display.println(WiFi.softAPIP());
  M5.Display.print("TX:"); M5.Display.println(txCount);
}

void setup() {
  auto cfg = M5.config(); cfg.serial_baudrate=115200;
  M5.begin(cfg); Serial.setTxTimeoutMs(0);
  delay(1000);
  M5.Display.setRotation(0);
  M5.Display.fillScreen(0); M5.Display.setTextColor(0xFFFF,0);
  M5.Display.setCursor(0,0); M5.Display.println("Booting...");

  loadPrefs();

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ip,ip,subnet);
  WiFi.softAP(ssid,pass);
  delay(200);

  pinMode(TX_PIN,OUTPUT); digitalWrite(TX_PIN,HIGH); delay(10);
  Serial1.begin(1000000,SERIAL_8N1,-1,TX_PIN); delay(100);

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    String h;
    h += "<!DOCTYPE html><html><head>";
    h += "<meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>";
    h += "<title>robot0009</title><style>";
    h += "body{font-family:-apple-system,sans-serif;margin:0;padding:14px;background:#f0f0f0;display:flex;justify-content:center;}";
    h += ".c{width:300px;}h2,h3{text-align:center;margin:0 0 12px;}";
    h += ".row{display:flex;gap:8px;margin-bottom:8px;}";
    h += ".btn{flex:1;height:64px;font-size:1.25rem;font-weight:bold;border:none;border-radius:12px;color:#fff;cursor:pointer;transition:filter .1s;}";
    h += ".btn:active{filter:brightness(.8);}";
    h += ".sw{background:#22c55e;}.sw.on{background:#ef4444;}";
    h += ".st{background:#f59e0b;}.st.on{background:#b45309;}";
    h += ".fw{background:#3b82f6;}.fw.on{background:#1d4ed8;}";
    h += ".bk{background:#6366f1;}.bk.on{background:#3730a3;}";
    h += ".lr{background:#14b8a6;}.lr.on{background:#0f766e;}";
    h += ".sx{background:#a855f7;}.sx.on{background:#7e22ce;}";
    h += ".sb{background:#6b7280;}.sb.on{background:#374151;}";
    h += ".fo{background:#f97316;}.fo.on{background:#7c2d12;}";
    h += ".co{background:#475569;height:52px;font-size:1.1rem;}";
    h += "#st{text-align:center;margin-top:10px;font-size:.9rem;color:#555;}";
    h += ".tabs{display:flex;gap:6px;margin-bottom:12px;}";
    h += ".tabBtn{flex:1;height:44px;font-size:1rem;font-weight:bold;border:none;border-radius:10px;background:#cbd5e1;cursor:pointer;}";
    h += ".tabBtn.active{background:#3b82f6;color:#fff;}";
    h += ".srow{display:flex;align-items:center;gap:4px;margin-bottom:8px;}";
    h += ".sid{width:38px;font-weight:bold;font-size:.9rem;flex-shrink:0;}";
    h += ".val{width:46px;text-align:center;font-size:1rem;font-weight:bold;background:#fff;border-radius:6px;padding:5px 0;flex-shrink:0;}";
    h += ".ab{flex:1;height:42px;font-size:1rem;font-weight:bold;border:none;border-radius:8px;background:#e2e8f0;cursor:pointer;}";
    h += ".ab:active{background:#cbd5e1;}";
    h += ".savebtn{width:100%;height:54px;font-size:1.15rem;font-weight:bold;border:none;border-radius:12px;background:#22c55e;color:#fff;cursor:pointer;margin-bottom:8px;}";
    h += ".backbtn{width:100%;height:48px;font-size:1.1rem;font-weight:bold;border:none;border-radius:12px;background:#6b7280;color:#fff;cursor:pointer;}";
    h += "#saveMsg{display:block;text-align:center;color:#16a34a;font-weight:bold;min-height:1.4em;margin-bottom:6px;}";
    h += "</style></head><body><div class='c'>";
    h += "<div id='main'><h2>robot0009</h2>";
    h += "<div class='row'><button id='b1' class='btn sw' onclick='tap(1,\"sweep\")'>START</button></div>";
    h += "<div class='row'><button id='b2' class='btn st' onclick='tap(2,\"step\")'>STEP</button><button id='b7' class='btn sx' onclick='tap(7,\"stretch\")'>STRETCH</button></div>";
    h += "<div class='row'><button id='b3' class='btn fw' onclick='tap(3,\"fwd\")'>FWD</button></div>";
    h += "<div class='row'><button id='b5' class='btn lr' onclick='tap(5,\"left\")'>LEFT</button><button id='b6' class='btn lr' onclick='tap(6,\"right\")'>RIGHT</button></div>";
    h += "<div class='row'><button id='b4' class='btn bk' onclick='tap(4,\"back\")'>BACK</button></div>";
    h += "<div class='row'><button id='b0' class='btn sb' onclick='tap(0,\"center\")'>STANDBY</button><button id='b8' class='btn fo' onclick='ws.send(\"fold\")'>FOLD</button></div>";
    h += "<div class='row'><button class='btn co' onclick='openCfg()'>CONFIG</button></div>";
    h += "<div id='st'>Standby | TX: 0</div></div>";
    h += "<div id='cfg' style='display:none'><h3>姿勢設定</h3>";
    h += "<div class='tabs'>";
    h +=   "<button id='tab0' class='tabBtn active' onclick='showTab(0)'>STANDBY</button>";
    h +=   "<button id='tab1' class='tabBtn'        onclick='showTab(1)'>収納</button>";
    h += "</div>";
    h += "<div id='panel0'><div id='rows0'></div></div>";
    h += "<div id='panel1' style='display:none'><div id='rows1'></div></div>";
    h += "<span id='saveMsg'></span>";
    h += "<button class='savebtn' onclick='saveActive()'>NVSに保存</button>";
    h += "<button class='backbtn' onclick='closeCfg()'>戻る</button>";
    h += "</div>";
    h += "<script>";
    h += "var ws,cur=0,activeTab=0;";
    h += "var vals=[511,511,511,511,511,511,511,511];";
    h += "var svals=[511,511,511,511,511,511,511,511];";
    h += "var BID =[null,'b1','b2','b3','b4','b5','b6','b7'];";
    h += "var BCLS=[null,'sw','st','fw','bk','lr','lr','sx'];";
    h += "var BLB =[null,'START','STEP','FWD','BACK','LEFT','RIGHT','STRETCH'];";
    h += "function makeRows(pfx,arr,fn,el){";
    h +=   "var s='';";
    h +=   "for(var i=0;i<8;i++){";
    h +=     "s+=\"<div class='srow'><span class='sid'>ID \"+i+\"</span>\";";
    h +=     "s+=\"<button class='ab' onclick='\"+fn+\"(\"+i+\",-20)'>-20</button>\";";
    h +=     "s+=\"<button class='ab' onclick='\"+fn+\"(\"+i+\",-5)'>-5</button>\";";
    h +=     "s+=\"<span class='val' id='\"+pfx+i+\"'>\"+arr[i]+\"</span>\";";
    h +=     "s+=\"<button class='ab' onclick='\"+fn+\"(\"+i+\",5)'>+5</button>\";";
    h +=     "s+=\"<button class='ab' onclick='\"+fn+\"(\"+i+\",20)'>+20</button></div>\";";
    h +=   "}";
    h +=   "document.getElementById(el).innerHTML=s;";
    h += "}";
    h += "function buildCfg(){makeRows('sv',vals,'adj','rows0');makeRows('ss',svals,'adjs','rows1');}";
    h += "function adj(id,d){";
    h +=   "vals[id]=Math.max(0,Math.min(1023,vals[id]+d));";
    h +=   "document.getElementById('sv'+id).textContent=vals[id];";
    h +=   "if(ws&&ws.readyState===1)ws.send('cfg:'+id+':'+vals[id]);";
    h += "}";
    h += "function adjs(id,d){";
    h +=   "svals[id]=Math.max(0,Math.min(1023,svals[id]+d));";
    h +=   "document.getElementById('ss'+id).textContent=svals[id];";
    h +=   "if(ws&&ws.readyState===1)ws.send('scfg:'+id+':'+svals[id]);";
    h += "}";
    h += "function showTab(t){";
    h +=   "activeTab=t;";
    h +=   "document.getElementById('panel0').style.display=t===0?'block':'none';";
    h +=   "document.getElementById('panel1').style.display=t===1?'block':'none';";
    h +=   "document.getElementById('tab0').className='tabBtn'+(t===0?' active':'');";
    h +=   "document.getElementById('tab1').className='tabBtn'+(t===1?' active':'');";
    h += "}";
    h += "function saveActive(){if(ws&&ws.readyState===1)ws.send(activeTab===0?'save':'ssave');}";
    h += "function openCfg(){";
    h +=   "if(ws&&ws.readyState===1){ws.send('stopall');ws.send('getcfg');ws.send('getscfg');}";
    h +=   "showTab(0);";
    h +=   "document.getElementById('main').style.display='none';";
    h +=   "document.getElementById('cfg').style.display='block';";
    h += "}";
    h += "function closeCfg(){";
    h +=   "document.getElementById('cfg').style.display='none';";
    h +=   "document.getElementById('main').style.display='block';";
    h += "}";
    h += "function upd(m,tx){";
    h +=   "if(cur>0&&cur<=7)document.getElementById(BID[cur]).className='btn '+BCLS[cur];";
    h +=   "document.getElementById('b0').className='btn sb';";
    h +=   "document.getElementById('b8').className='btn fo'+(m===8?' on':'');";
    h +=   "cur=m;";
    h +=   "if(cur>0&&cur<=7)document.getElementById(BID[cur]).className='btn '+BCLS[cur]+' on';";
    h +=   "if(cur===0)document.getElementById('b0').className='btn sb on';";
    h +=   "document.getElementById('st').textContent=(m===0?'Standby':m===8?'FOLD':BLB[m])+' | TX:'+tx;";
    h += "}";
    h += "function tap(m,cmd){if(ws&&ws.readyState===1)ws.send(cmd);}";
    h += "function connect(){";
    h +=   "ws=new WebSocket('ws://'+location.hostname+'/ws');";
    h +=   "ws.onclose=function(){setTimeout(connect,1000);};";
    h +=   "ws.onmessage=function(e){";
    h +=     "var d=JSON.parse(e.data);";
    h +=     "if(d.mode!==undefined){upd(d.mode,d.tx);}";
    h +=     "else if(d.cfg){";
    h +=       "vals=d.cfg;";
    h +=       "for(var i=0;i<8;i++){var el=document.getElementById('sv'+i);if(el)el.textContent=vals[i];}";
    h +=     "}else if(d.scfg){";
    h +=       "svals=d.scfg;";
    h +=       "for(var i=0;i<8;i++){var el=document.getElementById('ss'+i);if(el)el.textContent=svals[i];}";
    h +=     "}else if(d.saved){";
    h +=       "var el=document.getElementById('saveMsg');";
    h +=       "el.textContent='保存済み！';";
    h +=       "setTimeout(function(){el.textContent='';},2000);";
    h +=     "}";
    h +=   "};";
    h += "}";
    h += "buildCfg();connect();";
    h += "</script></div></body></html>";
    req->send(200,"text/html",h);
  });

  server.begin();
  goStandby(); delay(800);
  drawDisp();
}

// ── loop ──────────────────────────────────────────────────────
void loop() {
  M5.update();
  uint8_t m = mode;

  switch (m) {

    // ── SWEEP: 動作確認用スイープ ───────────────────────────────
    case M_SWEEP:
      kneeInGA(LIFT, MS_MOVE);
      sendRelGrp(SHOULDER_A, 2, +STRIDE, MS_MOVE);
      kneeStdGB(MS_MOVE);
      sendRelGrp(SHOULDER_B, 2, -STRIDE, MS_MOVE);
      if(!waitM(m, MS_MOVE+200)) break;
      goStandby();
      if(!waitM(m, MS_MOVE+200)) break;
      kneeInGB(LIFT, MS_MOVE);
      sendRelGrp(SHOULDER_B, 2, +STRIDE, MS_MOVE);
      kneeStdGA(MS_MOVE);
      sendRelGrp(SHOULDER_A, 2, -STRIDE, MS_MOVE);
      if(!waitM(m, MS_MOVE+200)) break;
      goStandby();
      waitM(m, MS_MOVE+200);
      break;

    // ── STEP: 対角2脚交互足踏み ─────────────────────────────────
    // フェーズ1: GA(右前ID4,6 + 左後ID0,2)
    //   肩1,5上げ → 膝3,7内側（垂直保持）
    //   肩1,5下げ → 膝3,7外側
    //   スタンバイへ
    // フェーズ2: GB(左前ID5,7 + 右後ID1,3) 同様
    case M_STEP:
      // フェーズ1: GA 持ち上げ（肩上げ＋膝内側）
      sendRelGrp(SHOULDER_A, 2, +STRIDE, MS_WALK);
      kneeInGA(LIFT, MS_WALK);
      if(!waitM(m, MS_WALK)) break;
      // GA 踏み込み（肩下げ＋膝外側）
      sendRelGrp(SHOULDER_A, 2, -STRIDE, MS_WALK);
      kneeOutGA(LIFT, MS_WALK);
      if(!waitM(m, MS_WALK)) break;
      // GA スタンバイへ
      sendRelGrp(SHOULDER_A, 2, 0, MS_WALK);
      kneeStdGA(MS_WALK);
      if(!waitM(m, MS_WALK)) break;
      // フェーズ2: GB 持ち上げ（肩上げ＋膝内側）
      sendRelGrp(SHOULDER_B, 2, +STRIDE, MS_WALK);
      kneeInGB(LIFT, MS_WALK);
      if(!waitM(m, MS_WALK)) break;
      // GB 踏み込み（肩下げ＋膝外側）
      sendRelGrp(SHOULDER_B, 2, -STRIDE, MS_WALK);
      kneeOutGB(LIFT, MS_WALK);
      if(!waitM(m, MS_WALK)) break;
      // GB スタンバイへ
      sendRelGrp(SHOULDER_B, 2, 0, MS_WALK);
      kneeStdGB(MS_WALK);
      waitM(m, MS_WALK);
      break;

    // ── FWD: 前進（対角歩行）────────────────────────────────────
    case M_FWD:
      // フェーズ1: GA持ち上げ前へ＋GB蹴り
      kneeInGA(LIFT, MS_WALK);
      sendRelGrp(SHOULDER_A, 2, +STRIDE, MS_WALK);
      kneeStdGB(MS_WALK);
      sendRelGrp(SHOULDER_B, 2, -STRIDE, MS_WALK);
      if(!waitM(m, MS_WALK)) break;
      // GA着地
      kneeStdGA(MS_WALK);
      if(!waitM(m, MS_WALK)) break;
      // フェーズ2: GB持ち上げ前へ＋GA蹴り
      kneeInGB(LIFT, MS_WALK);
      sendRelGrp(SHOULDER_B, 2, +STRIDE, MS_WALK);
      kneeStdGA(MS_WALK);
      sendRelGrp(SHOULDER_A, 2, -STRIDE, MS_WALK);
      if(!waitM(m, MS_WALK)) break;
      // GB着地
      kneeStdGB(MS_WALK);
      waitM(m, MS_WALK);
      break;

    // ── BACK: 後退（肩方向をFWDの逆に）────────────────────────
    case M_BACK:
      kneeInGA(LIFT, MS_WALK);
      sendRelGrp(SHOULDER_A, 2, -STRIDE, MS_WALK);
      kneeStdGB(MS_WALK);
      sendRelGrp(SHOULDER_B, 2, +STRIDE, MS_WALK);
      if(!waitM(m, MS_WALK)) break;
      kneeStdGA(MS_WALK);
      if(!waitM(m, MS_WALK)) break;
      kneeInGB(LIFT, MS_WALK);
      sendRelGrp(SHOULDER_B, 2, -STRIDE, MS_WALK);
      kneeStdGA(MS_WALK);
      sendRelGrp(SHOULDER_A, 2, +STRIDE, MS_WALK);
      if(!waitM(m, MS_WALK)) break;
      kneeStdGB(MS_WALK);
      waitM(m, MS_WALK);
      break;

    // ── LEFT: 左旋回 ────────────────────────────────────────────
    case M_LEFT:
      // 左側持ち上げ＋後ろへ、右側は前へ
      kneeInGL(LIFT, MS_WALK);
      sendRelGrp(GL_SH, 2, -STRIDE, MS_WALK);
      kneeStdGR(MS_WALK);
      sendRelGrp(GR_SH, 2, +STRIDE, MS_WALK);
      if(!waitM(m, MS_WALK)) break;
      kneeStdGL(MS_WALK);
      if(!waitM(m, MS_WALK)) break;
      // 右側持ち上げ＋後ろへ、左側は前へ
      kneeInGR(LIFT, MS_WALK);
      sendRelGrp(GR_SH, 2, -STRIDE, MS_WALK);
      kneeStdGL(MS_WALK);
      sendRelGrp(GL_SH, 2, +STRIDE, MS_WALK);
      if(!waitM(m, MS_WALK)) break;
      kneeStdGR(MS_WALK);
      waitM(m, MS_WALK);
      break;

    // ── RIGHT: 右旋回（LEFTの肩方向を逆に）────────────────────
    case M_RIGHT:
      kneeInGR(LIFT, MS_WALK);
      sendRelGrp(GR_SH, 2, -STRIDE, MS_WALK);
      kneeStdGL(MS_WALK);
      sendRelGrp(GL_SH, 2, +STRIDE, MS_WALK);
      if(!waitM(m, MS_WALK)) break;
      kneeStdGR(MS_WALK);
      if(!waitM(m, MS_WALK)) break;
      kneeInGL(LIFT, MS_WALK);
      sendRelGrp(GL_SH, 2, -STRIDE, MS_WALK);
      kneeStdGR(MS_WALK);
      sendRelGrp(GR_SH, 2, +STRIDE, MS_WALK);
      if(!waitM(m, MS_WALK)) break;
      kneeStdGL(MS_WALK);
      waitM(m, MS_WALK);
      break;

    // ── STRETCH: 全脚同時に屈伸 ─────────────────────────────────
    case M_STRETCH:
      // 全脚持ち上げ（膝内側＋肩上げ）
      kneeInGA(LIFT, MS_STR);
      kneeInGB(LIFT, MS_STR);
      sendRelGrp(SHOULDER_A, 2, +STRIDE, MS_STR);
      sendRelGrp(SHOULDER_B, 2, +STRIDE, MS_STR);
      if(!waitM(m, MS_STR+200)) break;
      goStandby();
      if(!waitM(m, MS_STR+200)) break;
      // 全脚沈み込み（膝外側＋肩下げ）
      kneeOutGA(LIFT, MS_STR);
      kneeOutGB(LIFT, MS_STR);
      sendRelGrp(SHOULDER_A, 2, -STRIDE, MS_STR);
      sendRelGrp(SHOULDER_B, 2, -STRIDE, MS_STR);
      if(!waitM(m, MS_STR+200)) break;
      goStandby();
      waitM(m, MS_STR+200);
      break;

    case M_FOLD:
    default:
      delay(20);
      break;
  }

  static unsigned long lastD = 0;
  if (millis()-lastD > 500) { lastD=millis(); drawDisp(); }
}