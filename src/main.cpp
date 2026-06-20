// robot0009 - SCS0009 quadruped (original: ATOM Matrix / M5Atom)
// ATOMS3 / M5Unified port. Servo TX is GPIO5 (moved from G38 by HW mod),
// so the internal IMU (MPU6886, SDA=G38/SCL=G39) works together with servos.
#include <M5Unified.h>
#include <Preferences.h>
#include <Kalman.h>
#include <WiFi.h>
#include <WebServer.h>

#define TX_PIN 5
// サーボバスのRX。HW改造でGPIO6を追加（TX側に1kΩ直列、RXはバス直結）。
// これにより現在位置の READ 応答を受信できる（半二重シングルワイヤ）。
#define RX_PIN 6

WebServer server(80);
const char ssid[] = "robot0009";  // SSID
const char pass[] = "password";   // password
const IPAddress ip(192, 168, 55, 27);      //　IP address
const IPAddress subnet(255, 255, 255, 0);

Preferences preferences;

//           ID     0    1    2    3    4    5    6    7
// 実機計測（2026-06-13）：各サーボが水平になる raw が 511+offset。
// 水平RAW = 476,511,524,504,564,467,526,545 → offset = 水平RAW-511
int offset[] = { -35,   0,  13,  -7,  53, -44,  15,  34 };

// 物理限界（仮値。実機で各サーボの可動域を測って絞り込むこと → TODO）
const int POS_MIN[8] = {60, 60, 60, 60, 60, 60, 60, 60};
const int POS_MAX[8] = {960, 960, 960, 960, 960, 960, 960, 960};

//leg length
float L1 = 50.0, L2 = 65.0;//unit:mm


Kalman kalmanX, kalmanY;
float kalAngleX, kalAngleDotX, kalAngleY, kalAngleDotY;
unsigned long oldTime = 0, loopTime, nowTime;
float dt;
float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float theta_X = 0.0, theta_Y = 0.0;
float theta_Xdot = 0.0, theta_Ydot = 0.0;

float hX, hY, hXpre, hYpre;

int pos = 0;


int Mode;
// 原作は #define だが、Imu が M5.Imu とマクロ衝突するため定数に変更（値は原作のまま）
constexpr int L = 1;
constexpr int R = 2;
constexpr int Imu = 3;
constexpr int Jump = 4;
constexpr int Jump2 = 42;
constexpr int Step = 5;
constexpr int Stretch = 6;
constexpr int Advance = 7;
constexpr int Back = 8;
constexpr int Roll = 9;
constexpr int Calib = 10;   // キャリブレーション（全サーボを角度0=511+offsetで保持）

// キャリブレーション対象サーボ（0..7）
int calSel = 0;

// 画面切替：false=操作ビュー / true=準備設定ビュー（速度・個別サーボ調整）
bool setupView = false;

// IDごとの想定マッピング（実機で検証する）。表示用ラベル。
const char* servoLabel(int id){
  switch(id){
    case 0: return "ID0 RL shoulder";
    case 1: return "ID1 RR shoulder";
    case 2: return "ID2 RL knee";
    case 3: return "ID3 RR knee";
    case 4: return "ID4 FR shoulder";
    case 5: return "ID5 FL shoulder";
    case 6: return "ID6 FR knee";
    case 7: return "ID7 FL knee";
    default: return "?";
  }
}

String modeBtJump = "off";
String modeBtStep = "off";
String modeBtImu = "off";
String modeBtStretch = "off";
String modeBtAd = "off";
String modeBtBack = "off";
String modeBtL = "off";
String modeBtR = "off";
String modeBtRoll = "off";


//パラメータ
//-------------------------------------------------
float wakeUpAngle = -1.0;
int period = 80, x = 60, height = 40, upHeight = 10, stride = 12;
float Kp = 0.3, Kd = 0.3;
// サーボの移動速度制限。0=最高速（危険）。小さいほどゆっくり。安全のため低めで起動。
int servoSpeed = 200;
//-------------------------------------------------

void scs_moveToPos(byte id, int position);
bool scs_readPosition(byte id, int &outRaw);
void softStartToHorizontal();
void servo_write(int ch, float ang);
void fRIK(float x, float z);
void fLIK(float x, float z);
void rRIK(float x, float z);
void rLIK(float x, float z);


//加速度センサから傾きデータ取得 [deg]
void get_theta() {
  M5.Imu.getAccel(&accX, &accY, &accZ);
  //傾斜角導出 単位はdeg
  // TODO: ATOM Matrix と ATOMS3 では MPU6886 の実装向きが異なる可能性が高い。
  // 実機テスト（シリアルの acc デバッグ出力）で符号・軸を確認してから IMU/Jump を使うこと。
  theta_X =  atan(accY / -accZ) * 57.29578f;
  theta_Y =  atan(-accX / -accZ) * 57.29578f;  // 実機計測：ロール軸が原作と逆のため符号反転（2026-06-11）

  // 表裏判定。ATOMS3/MPU6886 はZ軸の向きが原作(ATOM Matrix)と逆で、
  // 画面上向き=立ち姿勢で accZ≈+1g、裏返しで accZ≈-1g となる。
  // このため原作の accZ 比較を反転させている（2026-06-13 実機確認）。
  if(Mode != Jump2){
    if(pos == 1 && accZ > 0.96) pos = 0;   // 画面上向き=立ち姿勢
    if(pos == 0 && accZ < -0.96) pos = 1;  // 裏返し
  }
}

//Y軸 角速度取得
void get_gyro_data() {
  M5.Imu.getGyro(&gyroX, &gyroY, &gyroZ);
  theta_Xdot = -gyroX;
  theta_Ydot = -gyroY;  // theta_Y の符号反転と整合させるため反転（2026-06-11）
}


//browser
// 動作系はすべてPOSTフォームで送信する（GETリンクだとSafari等のリンク先読みで
// 勝手に動作が発火し暴れるため）。各エンドポイントもPOST専用で登録する。
static String formBtn(const char* action, const String& label, const String& cls){
  return "<form method='post' action='" + String(action) + "' style='display:inline;margin:0'>"
         "<button class='" + cls + "'>" + label + "</button></form>";
}
static String pmBtn(const char* action, const String& label){
  return "<form method='post' action='" + String(action) + "' style='display:inline;margin:0'>"
         "<button class='pm'>" + label + "</button></form>";
}

void handleRoot() {
  String temp ="<!DOCTYPE html>\n<html lang=\"ja\"><head>";
  temp +="<meta charset=\"utf-8\">";
  temp +="<title>robot0009</title>";
  temp +="<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  temp +="<style>";
  temp +=".container{margin:auto;text-align:center;font-size:1.2rem;}";
  temp +="span{display:inline-block;border:1px solid #ccc;width:90px;height:36px;line-height:36px;font-size:1.15rem;vertical-align:middle;margin:2px;}";
  temp +=".pm{width:58px;height:44px;font-size:1.15rem;font-weight:bold;margin:2px;vertical-align:middle;border-radius:6px;border:1px solid #999;}";
  temp +="button{width:104px;height:50px;font-weight:bold;font-size:1.05rem;margin:4px;border-radius:8px;border:1px solid #888;}";
  temp +="h4{margin:6px;font-size:1.2rem;}";
  temp +="b{font-size:1.25rem;}";
  temp +=".toggle{width:270px;height:56px;font-size:1.3rem;}"; // 画面切替（大きめ）
  temp +=".wide{width:auto;padding:0 16px;font-size:1.1rem;}"; // 初期姿勢 真っ直ぐ
  // カテゴリ配色（有効中の緑 button.on が最優先で上書き）
  temp +=".stand{background:#3a7bff;color:#fff;}";   // 立ち
  temp +=".calib{background:#cfcfcf;}";               // 真っ直ぐ・設定系
  temp +=".walk{background:#cfe8ff;}";                // 前後・足踏み・屈伸
  temp +=".turn{background:#fff0c2;}";                // 旋回
  temp +=".bal{background:#e3d4ff;}";                 // 水平維持(IMU)
  temp +=".danger{background:#ffd2d2;}";              // ジャンプ・反転
  temp +="button.on{background:#19d219;color:#fff;}"; // ← 有効中は常に緑
  // 方向パッド（3x3、中央=立ち）
  temp +=".pad{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;max-width:340px;margin:10px auto;}";
  temp +=".pad form{margin:0;}";
  temp +=".pad button{width:100%;height:64px;font-size:1.15rem;}";
  temp +="</style></head><body><div class=\"container\">";
  temp +="<h3>robot0009</h3>";

  if(setupView){
    // ===================== 準備設定ビュー =====================
    temp += formBtn("/viewMain", "← 操作にもどる", "calib toggle");
    temp +="<hr><b>サーボ速度</b>（小さいほどゆっくり＝安全）<br>";
    temp += pmBtn("/speedM","-") + "<span>"+String(servoSpeed)+"</span>" + pmBtn("/speedP","+") + "<br>";
    temp += formBtn("/resetParams","設定を初期値へ","calib");

    temp +="<hr><b>個別サーボ 角度調整</b><br>";
    temp += formBtn("/calMode","真っ直ぐ（水平保持）", String("calib ")+((Mode==Calib)?"on":"off"));
    temp +="<h4>" + String(servoLabel(calSel)) + "</h4>";
    temp +="サーボ選択<br>" + pmBtn("/calPrev","&#9664;ID") + "<span>ID "+String(calSel)+"</span>" + pmBtn("/calNext","ID&#9654;") + "<br>";
    temp +="オフセット<br>" + pmBtn("/calM10","-10") + pmBtn("/calM1","-1")
          + "<span>"+String(offset[calSel])+"</span>"
          + pmBtn("/calP1","+1") + pmBtn("/calP10","+10") + "<br>";
    temp +="生値 raw<br><span>" + String(511 + offset[calSel]) + "</span><br>";
    temp += formBtn("/calReset","このIDをリセット","calib");

  }else{
    // ===================== 操作ビュー =====================
    // 画面最上部に切替ボタン（大きめ）
    temp += formBtn("/viewSetup", "準備設定へ →", "calib toggle");

    // 姿勢調節（動作群の上）
    temp +="<hr><b>姿勢調節</b><br>";
    temp +="前後位置（x）<br>" + pmBtn("/xM","-") + "<span>"+String(x)+"</span>" + pmBtn("/xP","+") + "<br>";
    temp +="高さ（height）<br>" + pmBtn("/heightM","-") + "<span>"+String(height)+"</span>" + pmBtn("/heightP","+") + "<br>";

    // 動作の見出しの横に「初期姿勢 真っ直ぐ」
    temp +="<hr><b>動作</b> ";
    temp += formBtn("/calMode","初期姿勢 真っ直ぐ", String("calib wide ")+((Mode==Calib)?"on":"off"));

    // 方向パッド（3x3）。中央=立ち
    //  足踏み  前進   屈伸
    //  左旋回  立ち   右旋回
    //  反転    後進   ジャンプ
    temp +="<div class=\"pad\">";
    temp += formBtn("/step",     "足踏み",   String("walk ")+modeBtStep);
    temp += formBtn("/ad",       "前進",     String("walk ")+modeBtAd);
    temp += formBtn("/stretch",  "屈伸",     String("walk ")+modeBtStretch);
    temp += formBtn("/L",        "左旋回",   String("turn ")+modeBtL);
    temp += formBtn("/standMode","立ち",     String("stand ")+((Mode==0)?"on":"off"));
    temp += formBtn("/R",        "右旋回",   String("turn ")+modeBtR);
    temp += formBtn("/Roll",     "反転",     String("danger ")+modeBtRoll);
    temp += formBtn("/back",     "後進",     String("walk ")+modeBtBack);
    temp += formBtn("/Jump",     "ジャンプ", String("danger ")+modeBtJump);
    temp +="</div>";

    // 水平維持（IMU）はパッド外
    temp += formBtn("/imu","水平維持（IMU）", String("bal ")+modeBtImu);

    // 歩行調整（動作群の下）
    temp +="<hr><b>歩行調整</b><br>";
    temp +="歩く周期（1歩の時間）<br>" + pmBtn("/periodM","-") + "<span>"+String(period)+"</span>" + pmBtn("/periodP","+") + "<br>";
    temp +="足の上げ高さ<br>" + pmBtn("/upHeightM","-")+"<span>"+String(upHeight)+"</span>"+pmBtn("/upHeightP","+")+ "<br>";
    temp +="歩幅<br>" + pmBtn("/strideM","-") + "<span>"+String(stride)+"</span>" + pmBtn("/strideP","+") + "<br>";
  }

  temp +="</div></body></html>";
  server.send(200, "text/html", temp);
}

void handleL() {
  if(modeBtL == "off"){
    modeBtL = "on";
    modeBtR = "off";
    modeBtJump = "off";
    modeBtStep = "off";
    modeBtStretch = "off";
    modeBtAd = "off";
    modeBtBack = "off";
    modeBtImu = "off";
    modeBtRoll = "off";

    Mode = L;
  }else{
    modeBtL = "off";

    Mode = 0;
  }
  handleRoot();
}

void handleR() {
  if(modeBtR == "off"){
    modeBtL = "off";
    modeBtR = "on";
    modeBtJump = "off";
    modeBtStep = "off";
    modeBtStretch = "off";
    modeBtAd = "off";
    modeBtBack = "off";
    modeBtImu = "off";
    modeBtRoll = "off";

    Mode = R;
  }else{
    modeBtR = "off";

    Mode = 0;
  }
  handleRoot();
}

void handleJump() {
  if(modeBtJump == "off" && pos == 0){
    modeBtL = "off";
    modeBtR = "off";
    modeBtJump = "on";
    modeBtStep = "off";
    modeBtStretch = "off";
    modeBtAd = "off";
    modeBtBack = "off";
    modeBtImu = "off";
    modeBtRoll = "off";

    Mode = Jump;
  }else{
    modeBtJump = "off";

    Mode = 0;
  }
  handleRoot();
}

void handleStep() {
  if(modeBtStep == "off"){
    modeBtL = "off";
    modeBtR = "off";
    modeBtJump = "off";
    modeBtStep = "on";
    modeBtStretch = "off";
    modeBtAd = "off";
    modeBtBack = "off";
    modeBtImu = "off";
    modeBtRoll = "off";

    Mode = Step;
  }else{
    modeBtStep = "off";

    Mode = 0;
  }
  handleRoot();
}


void handleImu() {
  if(modeBtImu == "off"){
    modeBtL = "off";
    modeBtR = "off";
    modeBtJump = "off";
    modeBtStep = "off";
    modeBtStretch = "off";
    modeBtAd = "off";
    modeBtBack = "off";
    modeBtImu = "on";
    modeBtRoll = "off";

    // 積分項リセット（前回の残留値で脚が跳ねるのを防止）
    hXpre = 0; hYpre = 0; hX = 0; hY = 0;

    Mode = Imu;
  }else{
    modeBtImu = "off";

    hXpre = 0; hYpre = 0; hX = 0; hY = 0;

    Mode = 0;
  }
  handleRoot();
}

void handleStretch() {
  if(modeBtStretch == "off"){
    modeBtL = "off";
    modeBtR = "off";
    modeBtJump = "off";
    modeBtStep = "off";
    modeBtStretch = "on";
    modeBtAd = "off";
    modeBtBack = "off";
    modeBtImu = "off";
    modeBtRoll = "off";

    Mode = Stretch;
  }else{
    modeBtStretch = "off";

    Mode = 0;
  }
  handleRoot();
}

void handleAd() {
  if(modeBtAd == "off"){
    modeBtL = "off";
    modeBtR = "off";
    modeBtJump = "off";
    modeBtStep = "off";
    modeBtStretch = "off";
    modeBtAd = "on";
    modeBtBack = "off";
    modeBtImu = "off";
    modeBtRoll = "off";

    Mode = Advance;
  }else{
    modeBtAd = "off";

    Mode = 0;
  }
  handleRoot();
}

void handleBack() {
  if(modeBtBack == "off"){
    modeBtL = "off";
    modeBtR = "off";
    modeBtJump = "off";
    modeBtStep = "off";
    modeBtStretch = "off";
    modeBtAd = "off";
    modeBtBack = "on";
    modeBtImu = "off";
    modeBtRoll = "off";

    Mode = Back;
  }else{
    modeBtBack = "off";

    Mode = 0;
  }
  handleRoot();
}

void handleRoll() {
  if(modeBtRoll == "off"){
    modeBtL = "off";
    modeBtR = "off";
    modeBtJump = "off";
    modeBtStep = "off";
    modeBtStretch = "off";
    modeBtAd = "off";
    modeBtBack = "off";
    modeBtImu = "off";
    modeBtRoll = "on";

    Mode = Roll;
  }else{
    modeBtRoll = "off";

    Mode = 0;
  }
  handleRoot();
}


// ===== キャリブレーション操作 =====
void saveOffset(int id){
  char key[6];
  snprintf(key, sizeof(key), "off%d", id);
  preferences.putInt(key, offset[id]);
}

void handleCalNext() { calSel = (calSel + 1) % 8; handleRoot(); }
void handleCalPrev() { calSel = (calSel + 7) % 8; handleRoot(); }

void handleCalP10() { offset[calSel] = constrain(offset[calSel] + 10, -300, 300); saveOffset(calSel); handleRoot(); }
void handleCalP1()  { offset[calSel] = constrain(offset[calSel] + 1,  -300, 300); saveOffset(calSel); handleRoot(); }
void handleCalM1()  { offset[calSel] = constrain(offset[calSel] - 1,  -300, 300); saveOffset(calSel); handleRoot(); }
void handleCalM10() { offset[calSel] = constrain(offset[calSel] - 10, -300, 300); saveOffset(calSel); handleRoot(); }
void handleCalReset() { offset[calSel] = 0; saveOffset(calSel); handleRoot(); }

void handleCalibMode() { Mode = Calib; handleRoot(); }  // 全サーボ水平保持
void handleStandMode() { Mode = 0;     handleRoot(); }  // IKによる立ち姿勢

void handleSpeedM() {  // 遅く（より安全）
  if(servoSpeed > 20){ servoSpeed -= 20; preferences.putInt("servoSpeed", servoSpeed); }
  handleRoot();
}
void handleSpeedP() {  // 速く
  if(servoSpeed < 1000){ servoSpeed += 20; preferences.putInt("servoSpeed", servoSpeed); }
  handleRoot();
}

// 画面切替。準備設定へ入るときは水平保持(CALIB)にして、offset調整が即見えるようにする。
void handleViewSetup() { setupView = true;  Mode = Calib; handleRoot(); }
void handleViewMain()  { setupView = false; handleRoot(); }

// 制御パラメータを初期値へ戻す（offset=キャリブ値は消さない）
void handleResetParams() {
  period = 80; x = 60; height = 40; upHeight = 10; stride = 12;
  servoSpeed = 200; Kp = 0.3; Kd = 0.3; wakeUpAngle = -1.0;
  preferences.putInt("period", period);
  preferences.putInt("x", x);
  preferences.putInt("height", height);
  preferences.putInt("upHeight", upHeight);
  preferences.putInt("stride", stride);
  preferences.putInt("servoSpeed", servoSpeed);
  preferences.putFloat("Kp", Kp);
  preferences.putFloat("Kd", Kd);
  preferences.putFloat("wakeUpAngle", wakeUpAngle);
  handleRoot();
}

void handleWakeUpAngleM() {
  if(wakeUpAngle > -90.0){
    wakeUpAngle -= 0.5;
    preferences.putFloat("wakeUpAngle", wakeUpAngle);
  }
  handleRoot();
}
void handleWakeUpAngleP() {
  if(wakeUpAngle <= 90.0){
    wakeUpAngle += 0.5;
    preferences.putFloat("wakeUpAngle", wakeUpAngle);
  }
  handleRoot();
}



void handleKpM() {
  if(Kp > 0){
    Kp -= 0.1;
    preferences.putFloat("Kp", Kp);
  }
  handleRoot();
}
void handleKpP() {
  if(Kp <= 10){
    Kp += 0.1;
    preferences.putFloat("Kp", Kp);
  }
  handleRoot();
}

void handleKdM() {
  if(Kd > 0){
    Kd -= 0.1;
    preferences.putFloat("Kd", Kd);
  }
  handleRoot();
}
void handleKdP() {
  if(Kd <= 10){
    Kd += 0.1;
    preferences.putFloat("Kd", Kd);
  }
  handleRoot();
}




void handlePeriodM() {
  if(period > 40){   // 40未満は歩行ループが高速回転して危険なので下限
    period -= 5;
    preferences.putInt("period", period);
  }
  handleRoot();
}
void handlePeriodP() {
  if(period <= 3000){
    period += 5;
    preferences.putInt("period", period);
  }
  handleRoot();
}


void handlexM() {
  if(x > -615){
    x -= 5;
    preferences.putInt("x", x);
  }
  handleRoot();
}
void handlexP() {
  if(x < 515){
    x += 5;
    preferences.putInt("x", x);
  }
  handleRoot();
}


void handleHeightM() {
  if(height > -120){
    height -= 5;
    preferences.putInt("height", height);
  }
  handleRoot();
}
void handleHeightP() {
  if(height <= 120){
    height += 5;
    preferences.putInt("height", height);
  }
  handleRoot();
}

void handleUpHeightM() {
  if(upHeight > 0){
    upHeight -= 2;
    preferences.putInt("upHeight", upHeight);
  }
  handleRoot();
}
void handleUpHeightP() {
  if(upHeight <= 500){
    upHeight += 2;
    preferences.putInt("upHeight", upHeight);
  }
  handleRoot();
}

void handleStrideM() {
  if(stride > 0){
    stride -= 2;
    preferences.putInt("stride", stride);
  }
  handleRoot();
}
void handleStrideP() {
  if(stride <= 40){
    stride += 2;
    preferences.putInt("stride", stride);
  }
  handleRoot();
}


const char* modeName() {
  switch(Mode){
    case L:       return "L";
    case R:       return "R";
    case Imu:     return "IMU";
    case Jump:    return "Jump";
    case Jump2:   return "Jump2";
    case Step:    return "Step";
    case Stretch: return "Stretch";
    case Advance: return "Advance";
    case Back:    return "Back";
    case Roll:    return "Roll";
    case Calib:   return "CALIB";
    default:      return "Stand";
  }
}

// 原作の5x5 LEDマトリクス表示の置き換え：128x128 LCDへ
// 上段：モード名 / 中段：kalAngleX,Y / 下段：水準器風バブル
void updateDisplay() {
  M5.Display.startWrite();

  M5.Display.setTextSize(2);
  M5.Display.fillRect(0, 0, 128, 18, TFT_BLACK);
  M5.Display.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Display.setCursor(2, 1);
  M5.Display.print(modeName());

  M5.Display.fillRect(0, 20, 128, 36, TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setCursor(2, 21);
  M5.Display.printf("X:%6.1f", kalAngleX);
  M5.Display.setCursor(2, 39);
  M5.Display.printf("Y:%6.1f", kalAngleY);

  M5.Display.fillRect(0, 60, 128, 64, TFT_BLACK);
  if(Mode == Calib){
    // キャリブレーション：選択ID・offset・raw を表示
    M5.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5.Display.setCursor(2, 62);
    M5.Display.printf("ID:%d", calSel);
    M5.Display.setCursor(2, 80);
    M5.Display.printf("off:%4d", offset[calSel]);
    M5.Display.setCursor(2, 98);
    M5.Display.printf("raw:%4d", 511 + offset[calSel]);
  }else{
    M5.Display.drawRect(32, 60, 64, 64, TFT_DARKGREY);
    M5.Display.drawFastHLine(32, 92, 64, TFT_DARKGREY);
    M5.Display.drawFastVLine(64, 60, 64, TFT_DARKGREY);
    int bx = 64 + (int)constrain(kalAngleY, -28.0f, 28.0f);
    int by = 92 + (int)constrain(kalAngleX, -28.0f, 28.0f);
    uint16_t c = (fabsf(kalAngleX) <= 1.0f && fabsf(kalAngleY) <= 1.0f) ? TFT_GREEN : TFT_CYAN;
    M5.Display.fillCircle(bx, by, 3, c);
  }

  M5.Display.endWrite();
}


//Core0
void browser(void *pvParameters) {
  M5.Display.setBrightness(60);
  M5.Display.fillScreen(TFT_BLACK);

  disableCore0WDT();

  for (;;){
    server.handleClient();

    updateDisplay();

    // IMU軸キャリブレーション用デバッグ出力（§7：符号・軸の実機確認に使う）
    static unsigned long lastDbg = 0;
    if(millis() - lastDbg >= 500){
      lastDbg = millis();
      Serial.printf("acc: %6.3f %6.3f %6.3f  theta: %6.1f %6.1f  kal: %6.1f %6.1f  pos:%d\n",
                    accX, accY, accZ, theta_X, theta_Y, kalAngleX, kalAngleY, pos);
    }

    delay(50);
  }
}


void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  Serial1.begin(1000000, SERIAL_8N1, RX_PIN, TX_PIN);

  if(!M5.Imu.isEnabled()){
    Serial.println("IMU not found!");
  }

  get_theta();
  kalmanX.setAngle(theta_X);
  kalmanY.setAngle(theta_Y);

  delay(200);


  //parameter memory
  preferences.begin("parameter", false);

  wakeUpAngle = preferences.getFloat("wakeUpAngle", wakeUpAngle);

  Kp = preferences.getFloat("Kp", Kp);
  Kd = preferences.getFloat("Kd", Kd);

  period = preferences.getInt("period", period);
  x = preferences.getInt("x", x);
  height = preferences.getInt("height", height);
  upHeight = preferences.getInt("upHeight", upHeight);
  stride = preferences.getInt("stride", stride);
  servoSpeed = preferences.getInt("servoSpeed", servoSpeed);

  // 残留データの安全補正：過去のテストで保存された極端値で暴れるのを防ぐ。
  // 特に period が小さすぎると歩行ループが高速回転して危険。
  period     = constrain(period,     40, 3000);
  upHeight   = constrain(upHeight,    0,   60);
  stride     = constrain(stride,      0,   40);
  servoSpeed = constrain(servoSpeed, 20, 1000);

  // サーボoffset（キャリブレーション値）の読み込み
  for(int i = 0; i < 8; i++){
    char key[6];
    snprintf(key, sizeof(key), "off%d", i);
    offset[i] = preferences.getInt(key, offset[i]);
  }

  // 起動直後はキャリブレーションモードで全サーボをセンター保持
  Mode = Calib;


  WiFi.softAP(ssid, pass);
  delay(100);
  WiFi.softAPConfig(ip, ip, subnet);

  IPAddress myIP = WiFi.softAPIP();


  // ページ表示のみGET。動作系はすべてPOST専用（GET先読みでの誤発火防止）。
  server.on("/", HTTP_GET, handleRoot);

  server.on("/Jump", HTTP_POST, handleJump);
  server.on("/step", HTTP_POST, handleStep);
  server.on("/imu", HTTP_POST, handleImu);
  server.on("/stretch", HTTP_POST, handleStretch);
  server.on("/ad", HTTP_POST, handleAd);
  server.on("/back", HTTP_POST, handleBack);
  server.on("/L", HTTP_POST, handleL);
  server.on("/R", HTTP_POST, handleR);
  server.on("/Roll", HTTP_POST, handleRoll);

  // キャリブレーション
  server.on("/calMode", HTTP_POST, handleCalibMode);
  server.on("/standMode", HTTP_POST, handleStandMode);
  server.on("/speedM", HTTP_POST, handleSpeedM);
  server.on("/speedP", HTTP_POST, handleSpeedP);
  server.on("/resetParams", HTTP_POST, handleResetParams);
  server.on("/viewSetup", HTTP_POST, handleViewSetup);
  server.on("/viewMain", HTTP_POST, handleViewMain);
  server.on("/calNext", HTTP_POST, handleCalNext);
  server.on("/calPrev", HTTP_POST, handleCalPrev);
  server.on("/calP10", HTTP_POST, handleCalP10);
  server.on("/calP1", HTTP_POST, handleCalP1);
  server.on("/calM1", HTTP_POST, handleCalM1);
  server.on("/calM10", HTTP_POST, handleCalM10);
  server.on("/calReset", HTTP_POST, handleCalReset);


  server.on("/wakeUpAngleM", HTTP_POST, handleWakeUpAngleM);
  server.on("/wakeUpAngleP", HTTP_POST, handleWakeUpAngleP);

  server.on("/KpM", HTTP_POST, handleKpM);
  server.on("/KpP", HTTP_POST, handleKpP);
  server.on("/KdM", HTTP_POST, handleKdM);
  server.on("/KdP", HTTP_POST, handleKdP);

  server.on("/periodM", HTTP_POST, handlePeriodM);
  server.on("/periodP", HTTP_POST, handlePeriodP);
  server.on("/xM", HTTP_POST, handlexM);
  server.on("/xP", HTTP_POST, handlexP);
  server.on("/heightM", HTTP_POST, handleHeightM);
  server.on("/heightP", HTTP_POST, handleHeightP);
  server.on("/upHeightM", HTTP_POST, handleUpHeightM);
  server.on("/upHeightP", HTTP_POST, handleUpHeightP);
  server.on("/strideM", HTTP_POST, handleStrideM);
  server.on("/strideP", HTTP_POST, handleStrideP);

  server.begin();

  // 起動時ソフトスタート：各サーボの実際の現在位置を読み取り、そこから
  // ゆっくり水平姿勢(511+offset)へ移行する。loop()のモード処理に入る前に一度だけ。
  // （loop()はsetup()完了後に走り始めるため、ここで完了させておく）
  softStartToHorizontal();

  //browser task
  xTaskCreatePinnedToCore(
    browser
    ,  "browser"   // A name just for humans
    ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL
    ,  0);
}


void loop() {
  nowTime = micros();
  loopTime = nowTime - oldTime;
  oldTime = nowTime;

  dt = (float)loopTime / 1000000.0; //sec

  get_theta();
  get_gyro_data();

  //カルマンフィルタ 姿勢 傾き
  kalAngleX = kalmanX.getAngle(theta_X, theta_Xdot, dt);
  kalAngleY = kalmanY.getAngle(theta_Y, theta_Ydot, dt);

  //カルマンフィルタ 姿勢 角速度
  kalAngleDotX = kalmanX.getRate();
  kalAngleDotY = kalmanY.getRate();

  //Serial.println(kalAngleX);


  if(Mode == Calib){ //キャリブレーション：全サーボをセンター(511+offset)で保持
    for(int i = 0; i < 8; i++){
      servo_write(i, 0);
    }
    delay(20);
  }else if(Mode == Jump){ //ジャンプ
    fRIK(20, 35);
    fLIK(20, 35);
    rLIK(-25, 95);
    rRIK(-25, 95);
    delay(500);

    fRIK(-35, 100);
    fLIK(-35, 100);
    delay(160);


    rRIK(107, -42);
    rLIK(107, -42);
    fRIK(-70, 90);
    fLIK(-70, 90);
    delay(100);

    rRIK(80, 30);
    rLIK(80, 30);

    Mode = Jump2;

    fRIK(-85, 40);
    fLIK(-85, 40);

  }else if(Mode == Jump2){
    if(kalAngleX < 0.0 && kalAngleX > wakeUpAngle){
      fRIK(60, 40);
      fLIK(60, 40);
      rRIK(60, 40);
      rLIK(60, 40);

      Mode = 0;
    }
  }else if(Mode == Stretch){ //屈伸
    float tim,tt;
    unsigned long time_mSt= millis();

    tim=0;
    while(tim<period * 8){
      tim=millis()-time_mSt;
      tt=float(tim * 2 * PI / (period * 8));
      fRIK(x, height + upHeight * sin(tt));
      fLIK(x, height + upHeight * sin(tt));
      rLIK(x, height + upHeight * sin(tt));
      rRIK(x, height + upHeight * sin(tt));
    }
  }else if(Mode == Step){ //足踏み
    float tim,tt;
    unsigned long time_mSt= millis();

    tim=0;
    while(tim<period){
      tim=millis()-time_mSt;
      tt=float(tim * PI / 2 / period);
      fRIK(x, height + upHeight * (0.5 * cos(2*tt) - 0.5));
      fLIK(x, height);
      rLIK(x, height + upHeight * (0.5 * cos(2*tt) - 0.5));
      rRIK(x, height);
    }

    time_mSt= millis();
    tim=0;
    while(tim<period){
      tim=millis()-time_mSt;
      tt=float(tim * PI / 2 / period);
      fRIK(x, height + upHeight * (-0.5 * cos(2*tt) - 0.5));
      fLIK(x, height);
      rLIK(x, height + upHeight * (-0.5 * cos(2*tt) - 0.5));
      rRIK(x, height);
    }

    time_mSt= millis();
    tim=0;
    while(tim<period){
      tim=millis()-time_mSt;
      tt=float(tim * PI / 2 / period);
      fRIK(x, height);
      fLIK(x, height + upHeight * (0.5 * cos(2*tt) - 0.5));
      rLIK(x, height);
      rRIK(x, height + upHeight * (0.5 * cos(2*tt) - 0.5));
    }

    time_mSt= millis();
    tim=0;
    while(tim<period){
      tim=millis()-time_mSt;
      tt=float(tim * PI / 2 / period);
      fRIK(x, height);
      fLIK(x, height + upHeight * (-0.5 * cos(2*tt) - 0.5));
      rLIK(x, height);
      rRIK(x, height + upHeight * (-0.5 * cos(2*tt) - 0.5));
    }
  }else if(Mode == Advance){ //前進
    float tim,tt;
    unsigned long time_mSt= millis();

    tim=0;
    while(tim<period){
      tim=millis()-time_mSt;
      tt=float(tim * PI / 2 / period);
      fRIK(x - stride * cos(tt), height + upHeight * (0.5 * cos(2*tt) - 0.5));
      fLIK(x + stride * cos(tt), height);
      rLIK(x + stride * cos(tt), height + upHeight * (0.5 * cos(2*tt) - 0.5));
      rRIK(x - stride * cos(tt), height);
    }

    time_mSt= millis();
    tim=0;
    while(tim<period){
      tim=millis()-time_mSt;
      tt=float(tim * PI / 2 / period);
      fRIK(x + stride * sin(tt), height + upHeight * (-0.5 * cos(2*tt) - 0.5));
      fLIK(x - stride * sin(tt), height);
      rLIK(x - stride * sin(tt), height + upHeight * (-0.5 * cos(2*tt) - 0.5));
      rRIK(x + stride * sin(tt), height);
    }

    time_mSt= millis();
    tim=0;
    while(tim<period){
      tim=millis()-time_mSt;
      tt=float(tim * PI / 2 / period);
      fRIK(x + stride * cos(tt), height);
      fLIK(x - stride * cos(tt), height + upHeight * (0.5 * cos(2*tt) - 0.5));
      rLIK(x - stride * cos(tt), height);
      rRIK(x + stride * cos(tt), height + upHeight * (0.5 * cos(2*tt) - 0.5));
    }

    time_mSt= millis();
    tim=0;
    while(tim<period){
      tim=millis()-time_mSt;
      tt=float(tim * PI / 2 / period);
      fRIK(x - stride * sin(tt), height);
      fLIK(x + stride * sin(tt), height + upHeight * (-0.5 * cos(2*tt) - 0.5));
      rLIK(x + stride * sin(tt), height);
      rRIK(x - stride * sin(tt), height + upHeight * (-0.5 * cos(2*tt) - 0.5));
    }
  }else if(Mode == Back){ //後進
    float tim,tt;
    unsigned long time_mSt= millis();

    tim=0;
    while(tim<period){
      tim=millis()-time_mSt;
      tt=float(tim * PI / 2 / period);
      fRIK(x + stride * cos(tt), height + upHeight * (0.5 * cos(2*tt) - 0.5));
      fLIK(x - stride * cos(tt), height);
      rLIK(x - stride * cos(tt), height + upHeight * (0.5 * cos(2*tt) - 0.5));
      rRIK(x + stride * cos(tt), height);
    }

    time_mSt= millis();
    tim=0;
    while(tim<period){
      tim=millis()-time_mSt;
      tt=float(tim * PI / 2 / period);
      fRIK(x - stride * sin(tt), height + upHeight * (-0.5 * cos(2*tt) - 0.5));
      fLIK(x + stride * sin(tt), height);
      rLIK(x + stride * sin(tt), height + upHeight * (-0.5 * cos(2*tt) - 0.5));
      rRIK(x - stride * sin(tt), height);
    }

    time_mSt= millis();
    tim=0;
    while(tim<period){
      tim=millis()-time_mSt;
      tt=float(tim * PI / 2 / period);
      fRIK(x - stride * cos(tt), height);
      fLIK(x + stride * cos(tt), height + upHeight * (0.5 * cos(2*tt) - 0.5));
      rLIK(x + stride * cos(tt), height);
      rRIK(x - stride * cos(tt), height + upHeight * (0.5 * cos(2*tt) - 0.5));
    }

    time_mSt= millis();
    tim=0;
    while(tim<period){
      tim=millis()-time_mSt;
      tt=float(tim * PI / 2 / period);
      fRIK(x + stride * sin(tt), height);
      fLIK(x - stride * sin(tt), height + upHeight * (-0.5 * cos(2*tt) - 0.5));
      rLIK(x - stride * sin(tt), height);
      rRIK(x + stride * sin(tt), height + upHeight * (-0.5 * cos(2*tt) - 0.5));
    }
  }else if(Mode == R){ //R
    float tim,tt;
    unsigned long time_mSt= millis();

    tim=0;
    while(tim<period){
      tim=millis()-time_mSt;
      tt=float(tim * PI / 2 / period);
      fRIK(10, 40 + 10 * (0.5 * cos(2*tt) - 0.5));
      fLIK(10 + 10 * cos(tt), 40);
      rLIK(10 + 10 * cos(tt), 40 + 10 * (0.5 * cos(2*tt) - 0.5));
      rRIK(10, 40);
    }

    time_mSt= millis();
    tim=0;
    while(tim<period){
      tim=millis()-time_mSt;
      tt=float(tim * PI / 2 / period);
      fRIK(10, 40 + 10 * (-0.5 * cos(2*tt) - 0.5));
      fLIK(10 - 10 * sin(tt), 40);
      rLIK(10 - 10 * sin(tt), 40 + 10 *(-0.5 * cos(2*tt) - 0.5));
      rRIK(10, 40);
    }

    time_mSt= millis();
    tim=0;
    while(tim<period){
      tim=millis()-time_mSt;
      tt=float(tim * PI / 2 / period);
      fRIK(10, 40);
      fLIK(10 - 10 * cos(tt), 40 + 10 * (0.5 * cos(2*tt) - 0.5));
      rLIK(10 - 10 * cos(tt), 40);
      rRIK(10, 40 + 10 * (0.5 * cos(2*tt) - 0.5));
    }

    time_mSt= millis();
    tim=0;
    while(tim<period){
      tim=millis()-time_mSt;
      tt=float(tim * PI / 2 / period);
      fRIK(10, 40);
      fLIK(10 + 10 * sin(tt), 40 + 10 * (-0.5 * cos(2*tt) - 0.5));
      rLIK(10 + 10 * sin(tt), 40);
      rRIK(10, 40 + 10 * (-0.5 * cos(2*tt) - 0.5));
    }
  }else if(Mode == L){ //L
    float tim,tt;
    unsigned long time_mSt= millis();

    tim=0;
    while(tim<period){
      tim=millis()-time_mSt;
      tt=float(tim * PI / 2 / period);
      fRIK(10 - 10 * cos(tt), 40 + 10 * (0.5 * cos(2*tt) - 0.5));
      fLIK(10, 40);
      rLIK(10, 40 + 10 * (0.5 * cos(2*tt) - 0.5));
      rRIK(10 - 10 * cos(tt), 40);
    }

    time_mSt= millis();
    tim=0;
    while(tim<period){
      tim=millis()-time_mSt;
      tt=float(tim * PI / 2 / period);
      fRIK(10 + 10 * sin(tt), 40 + 10 * (-0.5 * cos(2*tt) - 0.5));
      fLIK(10, 40);
      rLIK(10, 40 + 10 *(-0.5 * cos(2*tt) - 0.5));
      rRIK(10 + 10 * sin(tt), 40);
    }

    time_mSt= millis();
    tim=0;
    while(tim<period){
      tim=millis()-time_mSt;
      tt=float(tim * PI / 2 / period);
      fRIK(10 + 10 * cos(tt), 40);
      fLIK(10, 40 + 10 * (0.5 * cos(2*tt) - 0.5));
      rLIK(10, 40);
      rRIK(10 + 10 * cos(tt), 40 + 10 * (0.5 * cos(2*tt) - 0.5));
    }

    time_mSt= millis();
    tim=0;
    while(tim<period){
      tim=millis()-time_mSt;
      tt=float(tim * PI / 2 / period);
      fRIK(10 - 10 * sin(tt), 40);
      fLIK(10, 40 + 10 * (-0.5 * cos(2*tt) - 0.5));
      rLIK(10, 40);
      rRIK(10 - 10 * sin(tt), 40 + 10 * (-0.5 * cos(2*tt) - 0.5));
    }
  }else if(Mode == Roll){ //反転
    fRIK(50, 30);
    fLIK(50, 30);
    rRIK(50, 30);
    rLIK(50, 30);
    delay(500);

    fRIK(10, 45);
    fLIK(10, 45);
    rRIK(0, 80);
    rLIK(0, 80);
    delay(500);

    fRIK(10, 90);
    fLIK(10, 90);
    delay(150);

    fRIK(70, -50);
    fLIK(70, -50);
    delay(200);

    rRIK(70, -50);
    rLIK(70, -50);
    delay(600);

    Mode = 0;
  }else if(Mode == Imu){
    hX = hXpre + Kp / 100.0 * kalAngleX + Kd / 100.0 * kalAngleDotX;
    hY = hYpre + Kp / 100.0 * kalAngleY + Kd / 100.0 * kalAngleDotY;

    // 積分の発散防止（クランプ後の値を次回の積分に引き継ぐ）
    hX = constrain(hX, -25.0, 25.0);
    hY = constrain(hY, -25.0, 25.0);

    hXpre = hX;
    hYpre = hY;

    if(pos == 1){ //反転時
      hX = -hX;
      hY = -hY;
    }

    fRIK(x, height - hX + hY);
    fLIK(x, height - hX - hY);
    rRIK(x, height + hX + hY);
    rLIK(x, height + hX - hY);
  }else{ //初期姿勢
    fRIK(x, height);
    fLIK(x, height);
    rRIK(x, height);
    rLIK(x, height);
  }
}


//参考 https://qiita.com/Ninagawa123/items/7b79c5f5117dd1470ac9
void scs_moveToPos(byte id, int position) {
  // コマンドパケットを作成
  byte message[13];
  message[0] = 0xFF;  // ヘッダ
  message[1] = 0xFF;  // ヘッダ
  message[2] = id;    // サーボID
  message[3] = 9;     // パケットデータ長
  message[4] = 3;     // コマンド（3は書き込み命令）
  message[5] = 42;    // レジスタ先頭番号
  message[6] = (position >> 8) & 0xFF; // 位置情報バイト上位
  message[7] = position & 0xFF; // 位置情報バイト下位
  message[8] = 0x00;  // 時間（上位）：0=速度制御
  message[9] = 0x00;  // 時間（下位）
  message[10] = (servoSpeed >> 8) & 0xFF; // 速度（上位）。0=最高速、小さいほどゆっくり
  message[11] = servoSpeed & 0xFF;        // 速度（下位）

  // チェックサムの計算
  byte checksum = 0;
  for (int i = 2; i < 12; i++) {
    checksum += message[i];
  }
  message[12] = ~checksum; // チェックサム

  // コマンドパケットを送信
  for (int i = 0; i < 13; i++) {
    Serial1.write(message[i]);
  }
}


// サーボの現在位置(Present Position, レジスタ0x38=56, 2byte)を読む。
// READ DATA命令=0x02。要求パケット(8byte):
//   FF FF [id] 0x04 0x02 0x38 0x02 [checksum]
//   checksum = (~(id + 0x04 + 0x02 + 0x38 + 0x02)) & 0xFF
// 応答パケット(8byte):
//   FF FF [id] 0x04 [error] [dataL] [dataH] [checksum]
//   ※現在位置データは low byte first（WRITEのpositionとは逆順）。
// 半二重シングルワイヤのため、送信した8byteが自分のRXへ自己エコーで返る。
// そこで送信前後にRXをflushし、受信側ではヘッダ(FF FF)を探索して
// 自己エコーと本当の応答を区別する。タイムアウト20ms、応答無しはfalse。
bool scs_readPosition(byte id, int &outRaw) {
  byte req[8];
  req[0] = 0xFF;
  req[1] = 0xFF;
  req[2] = id;
  req[3] = 0x04;  // 以降のバイト数（命令+アドレス+長さ+checksum）
  req[4] = 0x02;  // READ DATA
  req[5] = 0x38;  // Present Position レジスタ先頭(56)
  req[6] = 0x02;  // 読み出し長(2byte)
  byte cs = 0;
  for (int i = 2; i < 7; i++) cs += req[i];
  req[7] = ~cs;

  // 送信前にRXバッファを空にして、古いデータと自己エコーの混入を減らす
  while (Serial1.available()) Serial1.read();

  for (int i = 0; i < 8; i++) Serial1.write(req[i]);
  Serial1.flush();  // 送信完了まで待つ

  // ヘッダ(FF FF)を探索しながら応答を受信する。
  // 自己エコー（送信した8byte）も FF FF で始まるため、ヘッダ直後の
  // [id][len] が応答フォーマット（len=0x04）かつchecksum一致するもののみ採用。
  // 単純化のため、ヘッダ探索→8byte読み→検証 を取りこぼしなく行う。
  unsigned long start = millis();
  int state = 0;            // FF FF 検出用ステート
  while (millis() - start < 20) {
    if (!Serial1.available()) continue;
    byte b = Serial1.read();

    if (state < 2) {
      // ヘッダ FF FF を待つ
      if (b == 0xFF) state++;
      else state = 0;
      continue;
    }

    // ここで b はヘッダ直後の1byte目（= id のはず）
    byte rid = b;
    // 残り5byte（len, error, dataL, dataH, checksum）を集める
    byte rest[5];
    int got = 0;
    while (got < 5 && millis() - start < 20) {
      if (!Serial1.available()) continue;
      rest[got++] = Serial1.read();
    }
    if (got < 5) return false;  // タイムアウト

    byte len   = rest[0];
    byte err   = rest[1];
    byte dataL = rest[2];
    byte dataH = rest[3];
    byte rcs   = rest[4];

    // 自己エコー（要求パケット FF FF id 04 02 38 02 cs）は
    // len=0x04 だが続きが 02 38 ... のため checksum が合わない。
    // READ応答の checksum = (~(id + len + error + dataL + dataH)) & 0xFF
    byte calc = (~(rid + len + err + dataL + dataH)) & 0xFF;
    if (rid == id && len == 0x04 && calc == rcs) {
      outRaw = dataL | (dataH << 8);  // low byte first
      return true;
    }

    // 不一致（自己エコー等）：ヘッダ探索からやり直す
    state = 0;
  }
  return false;  // タイムアウト
}


// 起動時のソフトスタート：各サーボの現在位置を読み取り、そこから水平姿勢
// (511+offset[id]) へ約20ステップ・合計800〜1000msかけてゆっくり補間移動する。
// 読み取りに失敗したサーボは目標rawをそのまま使う（=ランプせずスキップ）。
// ステップのループを外側、サーボのループを内側にして全軸を同期して動かす。
void softStartToHorizontal() {
  const int STEPS = 20;
  const int STEP_DELAY = 45;  // ms。20step×45ms ≒ 900ms
  int startRaw[8];
  int targetRaw[8];

  Serial.println("softStart: read current positions");
  for (int id = 0; id < 8; id++) {
    targetRaw[id] = constrain(511 + offset[id], POS_MIN[id], POS_MAX[id]);
    int raw = 0;
    if (scs_readPosition(id, raw)) {
      startRaw[id] = raw;
      Serial.printf("  ID%d read OK  raw=%d  target=%d\n", id, raw, targetRaw[id]);
    } else {
      // 失敗：目標値を始点にして、このサーボはランプしない
      startRaw[id] = targetRaw[id];
      Serial.printf("  ID%d read FAIL -> skip ramp (use target=%d)\n", id, targetRaw[id]);
    }
  }

  Serial.println("softStart: ramp to horizontal");
  for (int s = 1; s <= STEPS; s++) {
    for (int id = 0; id < 8; id++) {
      // 現在raw → 目標raw を線形補間
      int posRaw = startRaw[id] + (targetRaw[id] - startRaw[id]) * s / STEPS;
      posRaw = constrain(posRaw, POS_MIN[id], POS_MAX[id]);
      scs_moveToPos(id, posRaw);
      delayMicroseconds(1500);  // 既存の連射防止ペーシングに合わせる
    }
    delay(STEP_DELAY);  // ステップ間の待ち（既存ペーシングとは別）
  }
  Serial.println("softStart: done");
}


void servo_write(int ch, float ang){
  int sig = 511 + offset[ch] + int((512.0 / 150.0) * ang);
  sig = constrain(sig, POS_MIN[ch], POS_MAX[ch]);  // 物理限界クランプ
  scs_moveToPos(ch, sig);
  // ペーシング：歩行ループがコマンドを連射してサーボが暴走するのを防ぐ。
  // 8サーボ×1.5ms≒12ms（約80Hz）でSTANDも歩行も同じ滑らかさに揃える。
  delayMicroseconds(1500);
}



//-------------------------------------------
//IK
//-------------------------------------------
// 2リンクIK。foot(x,z) → th1[deg]=腿の角度（0で水平、+で脚が下がる）,
//                        th2[deg]=膝の曲げ角（0で脚まっすぐ、+で足が下がる）。
// 角度0は実機キャリブレーションの「水平」姿勢に一致（offsetで校正済み）。
static void legIK(float x, float z, float &th1, float &th2){
  float ld = sqrt(x*x + z*z);
  float c1 = (L1*L1 - L2*L2 + ld*ld) / (2*L1*ld);
  float c2 = (ld*ld - L1*L1 - L2*L2) / (2*L1*L2);
  c1 = constrain(c1, -1.0f, 1.0f);   // 到達不能時の acos NaN を防止
  c2 = constrain(c2, -1.0f, 1.0f);
  float phi = atan2(x, z);
  th1 = 90.0 - (phi + acos(c1)) * 180.0 / PI;
  th2 = acos(c2) * 180.0 / PI;
}

// 実機計測（2026-06-13）に基づく取付符号：
//   +方向=下がる … RL肩(0) RL膝(2) FR肩(4) FR膝(6) → +th
//   +方向=上がる … RR肩(1) RR膝(3) FL肩(5) FL膝(7) → -th
// 対角 RL+FR が (+,+)、RR+FL が (-,-)。
// pos==1（裏返し）は本体が上下逆になるため符号反転（self-right用、要実機検証）。
void fRIK(float x, float z){          // 前右 FR：肩ID4 膝ID6  (+th1,+th2)
  float th1, th2; legIK(x, z, th1, th2);
  float s = (pos == 0) ? 1.0f : -1.0f;
  servo_write(4,  s * th1);
  servo_write(6,  s * th2);
}

void fLIK(float x, float z){          // 前左 FL：肩ID5 膝ID7  (-th1,-th2)
  float th1, th2; legIK(x, z, th1, th2);
  float s = (pos == 0) ? 1.0f : -1.0f;
  servo_write(5, -s * th1);
  servo_write(7, -s * th2);
}

void rRIK(float x, float z){          // 後右 RR：肩ID1 膝ID3  (-th1,-th2)
  float th1, th2; legIK(x, z, th1, th2);
  float s = (pos == 0) ? 1.0f : -1.0f;
  servo_write(1, -s * th1);
  servo_write(3, -s * th2);
}

void rLIK(float x, float z){          // 後左 RL：肩ID0 膝ID2  (+th1,+th2)
  float th1, th2; legIK(x, z, th1, th2);
  float s = (pos == 0) ? 1.0f : -1.0f;
  servo_write(0,  s * th1);
  servo_write(2,  s * th2);
}
