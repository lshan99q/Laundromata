#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h> // 用于 3D 计算

// -------------------- 硬件引脚 --------------------
#define OLED_SCL 10
#define OLED_SDA 9
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);

#define K_UP    4
#define K_DOWN  5
#define K_OK    7   // '#'
#define K_BACK  6   // '*'
#define K_BOOT  0   // BOOT 键，用于进入开发者简介

#define UART_RX 44
#define UART_TX 43
HardwareSerial MySerial(1);

// 板载 LED
#define LED_PIN 48

// -------------------- 状态管理 --------------------
const char* menuList[4] = {"Mode A (Pa)", "Mode B (Pb)", "Mode C (Pc)", "Mode D (Pd)"};
int menuIndex = 0;
int menuOffsetY = 0;     // 当前偏移量（动画）
int targetOffsetY = 0;   // 目标偏移量
int slideY = 0;
int slideTarget = 0;
bool isSliding = false;



// UI 状态机
enum SystemState {
  STATE_HOME,
  STATE_MENU,
  STATE_VIEW_LOG,
  STATE_DEV_INTRO, // 新增：开发者简介 3D 模式
  STATE_DEV_RETURN
};
SystemState currentState = STATE_HOME;

int scrollLine = 0; // 日志查看滚动行

// -------------------- 3D 引擎变量 --------------------
double angleX = 0;
double angleY = 0;

// -------------------- DEV INTRO 动画控制 --------------------
int devAnimType = -1;            // -1 未初始化, 0 = cube, 1 = pyramid
unsigned long devAnimTicker = 0; // 用于简单节拍
float devAngleX = 0.0;           // DEV 动画独立角度（不影响其它）
float devAngleY = 0.0;


// -------------------- 数据定义 --------------------
// 每个模式发送的命令
uint8_t cmdPa[] = {0x13, 0x01, 0xA1, 0x61, 0xC3};
uint8_t cmdPb[] = {0x13, 0x09, 0x4A, 0x3F, 0x91, 0x2C, 0x23, 0x74, 0x27, 0x00, 0x01, 0x15, 0xEA};
uint8_t cmdPc[] = {0x13, 0x09, 0xB0, 0x17, 0xBC, 0x43, 0x23, 0x74, 0x27, 0x00, 0x91, 0x15, 0xEA};
uint8_t cmdPd[] = {0x13, 0x09, 0xB0, 0x17, 0xBC, 0x43, 0x23, 0x74, 0x27, 0x00, 0x01, 0x15, 0xEA};
uint8_t universalSuccess[] = {0x13, 0x09, 0xB0, 0x17, 0xBC, 0x43, 0x23, 0x74, 0x27, 0x00, 0x01, 0x15, 0xEA};

// 接收缓冲
#define MAX_REPLY_LEN 1024
uint8_t replyBuf[MAX_REPLY_LEN];
int replyLen = 0;

// 串口临时缓冲 String
String rxLineBuf = "";
unsigned long lastRecvTime = 0;

// 扣费检测缓冲
uint8_t tempBuf[20];
int tempIndex = 0;

// OLED 日志缓冲
#define OLED_MAX_LINES 6
#define CHARS_PER_LINE 20
String oledLines[OLED_MAX_LINES];

// -------------------- 核心工具函数 --------------------
bool isPaymentRequest(const uint8_t* data, int len) {
  return (len >= 10 && data[0] == 0x13 && data[1] == 0x06);
}

String bytesToHexString(const uint8_t *buf, int len) {
  if (len <= 0) return "";
  String s;
  s.reserve(len * 3);
  for (int i = 0; i < len; i++) {
    char tmp[4];
    sprintf(tmp, "%02X", buf[i]);
    if (i > 0) s += ' ';
    s += tmp;
  }
  return s;
}

void oledAppendLineSegments(const String &s) {
  String tmp = s;
  int pos = 0;
  while (pos < (int)tmp.length()) {
    String seg = tmp.substring(pos, min(pos + CHARS_PER_LINE, (int)tmp.length()));
    for (int i = 0; i < OLED_MAX_LINES - 1; i++) oledLines[i] = oledLines[i + 1];
    oledLines[OLED_MAX_LINES - 1] = seg;
    pos += CHARS_PER_LINE;
  }
}

// -------------------- 3D Cube --------------------
struct Point3D { double x, y, z; };
struct Point2D { int x, y; };

Point3D cubeNodes[8] = {
  {-10, -10, -10}, {10, -10, -10}, {10, 10, -10}, {-10, 10, -10},
  {-10, -10, 10}, {10, -10, 10}, {10, 10, 10}, {-10, 10, 10}
};
int cubeEdges[12][2] = {
  {0,1}, {1,2}, {2,3}, {3,0}, {4,5}, {5,6}, {6,7}, {7,4}, {0,4}, {1,5}, {2,6}, {3,7}
};

void drawRotatingCube(int offsetX, int offsetY) {
  double sinX = sin(angleX), cosX = cos(angleX);
  double sinY = sin(angleY), cosY = cos(angleY);
  
  Point2D projNodes[8];
  for(int i=0;i<8;i++){
    Point3D p = cubeNodes[i];
    double x1 = p.x * cosY - p.z * sinY;
    double z1 = p.z * cosY + p.x * sinY;
    double y2 = p.y * cosX - z1 * sinX;
    double z2 = z1 * cosX + p.y * sinX;
    double scale = 120 / (z2 + 60);
    projNodes[i].x = (int)(x1 * scale) + offsetX;
    projNodes[i].y = (int)(y2 * scale) + offsetY;
  }
  for(int i=0;i<12;i++){
    u8g2.drawLine(
      projNodes[cubeEdges[i][0]].x, projNodes[cubeEdges[i][0]].y,
      projNodes[cubeEdges[i][1]].x, projNodes[cubeEdges[i][1]].y
    );
  }
  angleX += 0.04;
  angleY += 0.07;
}

// -------------------- 三角锥 (Pyramid) --------------------
void drawRotatingPyramid(int offsetX, int offsetY) {
  // 三角锥 4 个顶点（底面为等边三角形）
  struct P3 { double x,y,z; };
  P3 nodes[4] = {
    { -12,  -8, -8 },   // 底面顶点 A
    {  12,  -8, -8 },   // 底面顶点 B
    {   0,   12, -8 },   // 底面顶点 C
    {   0,    0, 16 }    // 顶点 Apex（尖顶）
  };

  double sx = sin(devAngleX), cx = cos(devAngleX);
  double sy = sin(devAngleY), cy = cos(devAngleY);

  // 投影并存储 2D 点
  Point2D p2[4];
  for (int i = 0; i < 4; ++i) {
    double x = nodes[i].x, y = nodes[i].y, z = nodes[i].z;
    // 先绕 Y 轴旋转（水平旋转）
    double x1 = x * cy - z * sy;
    double z1 = z * cy + x * sy;
    // 再绕 X 轴旋转（垂直旋转）
    double y1 = y * cx - z1 * sx;
    double z2 = z1 * cx + y * sx;
    double scale = 120.0 / (z2 + 80.0); // 透视缩放
    p2[i].x = (int)(x1 * scale) + offsetX;
    p2[i].y = (int)(y1 * scale) + offsetY;
  }

  // 绘制底面三角形边
  u8g2.drawLine(p2[0].x, p2[0].y, p2[1].x, p2[1].y);
  u8g2.drawLine(p2[1].x, p2[1].y, p2[2].x, p2[2].y);
  u8g2.drawLine(p2[2].x, p2[2].y, p2[0].x, p2[0].y);
  // 绘制三条侧边到顶点
  u8g2.drawLine(p2[0].x, p2[0].y, p2[3].x, p2[3].y);
  u8g2.drawLine(p2[1].x, p2[1].y, p2[3].x, p2[3].y);
  u8g2.drawLine(p2[2].x, p2[2].y, p2[3].x, p2[3].y);

  // 更新角度（让它转动）
  devAngleX += 0.035;
  devAngleY += 0.055;
}

// -------------------- UI --------------------
void drawModernHome() {
  u8g2.clearBuffer();
  u8g2.sendBuffer();
  for (int y = 64; y >= 30; y-=2) {
    u8g2.clearBuffer();
    u8g2.drawRFrame(10, y-15, 108, 30, 5); 
    u8g2.setFont(u8g2_font_ncenB10_tr);
    int w = u8g2.getStrWidth("Laundromata");
    u8g2.drawStr((128-w)/2, y+5, "Laundromata");
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(35, 60, "System Init...");
    u8g2.sendBuffer();
    delay(10);
  }
  delay(500);
}

void drawModernMenu() {

  // --- 滑动动画 ---
  if (isSliding) {
    slideY += (slideTarget - slideY) * 0.25;

    // 到达目标
    if (abs(slideTarget - slideY) < 1) {
      slideY = slideTarget;
      isSliding = false;
    }
  }

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_helvB08_tr);
  u8g2.drawStr(4, 10, "Select Mode");
  u8g2.drawLine(0, 12, 128, 12);

  int itemHeight = 12;

  // --- 当前菜单 ---
  for (int i = 0; i < 4; i++) {
    int y = 16 + (i * itemHeight) + slideY;
    if (i == menuIndex) {
      u8g2.setDrawColor(1);
      u8g2.drawRBox(2, y, 124, itemHeight + 1, 3);
      u8g2.setDrawColor(0);
    } else {
      u8g2.setDrawColor(1);
    }
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(6, y + 9, menuList[i]);
  }

  u8g2.setDrawColor(1);
  u8g2.sendBuffer();
}


void drawModernLog() {
  u8g2.clearBuffer();
  u8g2.drawBox(0, 0, 128, 10);
  u8g2.setDrawColor(0);
  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.drawStr(2, 7, "LOG MONITOR");
  char buf[20];
  sprintf(buf, "%d B", replyLen);
  u8g2.drawStr(90, 7, buf);
  u8g2.setDrawColor(1);

  u8g2.setFont(u8g2_font_5x7_tr);
  int bytesPerLine = 8;
  int lineHeight = 8;
  int contentY = 12;
  int maxLinesVis = 6;
  
  int totalLines = (replyLen + bytesPerLine - 1) / bytesPerLine;
  int maxScroll = max(0, totalLines - maxLinesVis);
  if (scrollLine < 0) scrollLine = 0;
  if (scrollLine > maxScroll) scrollLine = maxScroll;

  for (int r = 0; r < maxLinesVis; r++) {
    int lineIdx = scrollLine + r;
    int startByte = lineIdx * bytesPerLine;
    if (startByte >= replyLen) break;
    int y = contentY + r * lineHeight + 7;
    for (int b = 0; b < bytesPerLine; b++) {
      if (startByte + b >= replyLen) break;
      char hex[4];
      sprintf(hex, "%02X", replyBuf[startByte + b]);
      u8g2.drawStr(4 + b*15, y, hex);
    }
  }

  if (totalLines > maxLinesVis) {
    int barH = 54;
    int knobH = max(4, barH * maxLinesVis / totalLines);
    int knobY = 10 + (scrollLine * (barH - knobH) / maxScroll);
    u8g2.drawVLine(126, 10, barH);
    u8g2.drawBox(125, knobY, 3, knobH);
  }

  u8g2.sendBuffer();
}

void drawDevIntro() {
  // 首次进入时随机决定要展示的模型
  if (devAnimType == -1) {
    devAnimType = random(0, 2); // 0 -> cube, 1 -> pyramid
    devAngleX = 0.0;
    devAngleY = 0.0;
    devAnimTicker = millis();
  }

  // 每帧清屏并绘制左侧 3D 区域（保持在 0..63 内）
  u8g2.clearBuffer();

  // 背景装饰（反色的竖线分割）
  u8g2.drawVLine(64, 0, 64);

  // 绘制中心 3D 区域的小边框（反色）
  u8g2.setDrawColor(1);
 

  // 根据 devAnimType 绘制对应模型（位置：区块中心）
  if (devAnimType == 0) {
    // 画正方体（使用现有函数 drawRotatingCube，偏移到左区中心）
    // 注意：drawRotatingCube 使用全局 angleX/angleY，暂时用 devAngle* 替代实现：
    // 临时保存全局角度，用 devAngle 绘制，再恢复（避免改太多原函数）
    double bakX = angleX, bakY = angleY;
    angleX = devAngleX;
    angleY = devAngleY;
    drawRotatingCube(32, 32);
    devAngleX = angleX;
    devAngleY = angleY;
    angleX = bakX;
    angleY = bakY;
  } else {
    // 画三角锥
    drawRotatingPyramid(32, 32);
  }

  // 右侧文字信息区（DEV INFO）
  u8g2.setFont(u8g2_font_helvB08_tr);
  u8g2.drawStr(70, 16, "DEV INFO");
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(70, 30, "developer:");
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(70, 42, "Circe");
  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.drawStr(70, 58, "Ver 2.0 Pro");

  // 小提示：按任意键返回菜单（闪烁）
  if ((millis() / 300) % 2 == 0) {
    u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.drawStr(70, 50, "Press * or #");
  }

  u8g2.sendBuffer();
}


void showPaymentSuccessModern() {
  SystemState prevState = currentState;
  for(int i=0; i<5; i++){
    u8g2.clearBuffer();
    u8g2.setDrawColor(0);
    u8g2.drawRBox(14, 12, 100, 40, 4);
    u8g2.setDrawColor(1);
    u8g2.drawRFrame(14, 12, 100, 40, 4);
    u8g2.setFont(u8g2_font_helvB08_tr);
    u8g2.drawStr(28, 30, "Payment OK");
    u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.drawStr(32, 44, "Auto Reply Sent");
    u8g2.sendBuffer();
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }
  delay(1000);
  currentState = prevState;
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(K_UP, INPUT_PULLUP);
  pinMode(K_DOWN, INPUT_PULLUP);
  pinMode(K_OK, INPUT_PULLUP);
  pinMode(K_BACK, INPUT_PULLUP);
  pinMode(K_BOOT, INPUT_PULLUP);

  Wire.begin(OLED_SDA, OLED_SCL);
  u8g2.begin();
  MySerial.begin(9600, SERIAL_8N1, UART_RX, UART_TX);
  drawModernHome();
  currentState = STATE_MENU;
}

// -------------------- Loop --------------------
void loop() {
  if (digitalRead(K_BOOT) == LOW) {
  delay(50);
  if (digitalRead(K_BOOT) == LOW) {
    if (currentState != STATE_DEV_INTRO) {
      devAnimType = -1; // 重置，下一帧会随机选择
      currentState = STATE_DEV_INTRO;
    } else {
      currentState = STATE_MENU;
    }
    while(digitalRead(K_BOOT) == LOW);
  }
}

  while (MySerial.available()) {
    int c = MySerial.read();
    if (c < 0) break;
    tempBuf[tempIndex++] = (uint8_t)c;
    if (tempIndex >= 20) tempIndex = 0;
    if (tempIndex >= 10 && isPaymentRequest(tempBuf, tempIndex)) {
        MySerial.write(universalSuccess, sizeof(universalSuccess));
        showPaymentSuccessModern();
        tempIndex = 0;
    }
    if (replyLen < MAX_REPLY_LEN) replyBuf[replyLen++] = (uint8_t)c;
    else { memmove(replyBuf, replyBuf+1, MAX_REPLY_LEN-1); replyBuf[MAX_REPLY_LEN-1]=(uint8_t)c; }
    if (rxLineBuf.length() < 512) rxLineBuf += (char)c;
    lastRecvTime = millis();
    digitalWrite(LED_PIN, HIGH);
  }

  if (rxLineBuf.length() > 0 && (millis() - lastRecvTime > 100)) {
    String hx = bytesToHexString((uint8_t*)rxLineBuf.c_str(), rxLineBuf.length());
    oledAppendLineSegments(hx);
    rxLineBuf = "";
    digitalWrite(LED_PIN, LOW);
  }

  switch (currentState) {
    case STATE_DEV_INTRO:
  drawDevIntro();
  if (!digitalRead(K_BACK) || !digitalRead(K_OK)) {
    currentState = STATE_DEV_RETURN;      // 切换到返回动画状态
    devAnimTicker = millis();             // 记录动画起始时间
  }
  break;


    case STATE_MENU:
      drawModernMenu();
      if (digitalRead(K_UP) == LOW && !isSliding) {

  // 1) 目标滑动位移（整页 64px）
  slideTarget = 64;
  isSliding = true;

  // 等动画走到一半再切换选项
  delay(120);
  menuIndex = (menuIndex + 3) % 4;

  slideY = -64;     // 新菜单从上方进场
  slideTarget = 0;  // 回到正常位置
  isSliding = true;

  delay(150);
}



      if (digitalRead(K_DOWN) == LOW && !isSliding) {

  slideTarget = -64;
  isSliding = true;

  delay(120);
  menuIndex = (menuIndex + 1) % 4;

  slideY = 64;      // 新菜单从下方进场
  slideTarget = 0;
  isSliding = true;

  delay(150);
}



      if (digitalRead(K_OK) == LOW) {
        const uint8_t* d; int l;
        if(menuIndex==0) {d=cmdPa; l=sizeof(cmdPa);}
        else if(menuIndex==1) {d=cmdPb; l=sizeof(cmdPb);}
        else if(menuIndex==2) {d=cmdPc; l=sizeof(cmdPc);}
        else {d=cmdPd; l=sizeof(cmdPd);}
        MySerial.write(d, l);
        replyLen = 0; scrollLine = 0;
        currentState = STATE_VIEW_LOG;
        delay(200);
      }
      break;

    case STATE_VIEW_LOG:
      drawModernLog();
      if (digitalRead(K_UP) == LOW) { scrollLine--; delay(50); }
      if (digitalRead(K_DOWN) == LOW) { scrollLine++; delay(50); }
      if (digitalRead(K_BACK) == LOW) { currentState = STATE_MENU; delay(200); }
      break;

    default:
      currentState = STATE_MENU;
      break;
  }
}
