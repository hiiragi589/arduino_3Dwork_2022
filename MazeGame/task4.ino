#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <U8glib.h>
#include <DFRobotDFPlayerMini.h>
#include <SoftwareSerial.h>
#include "MyLibrary.h"

#define AUDIO_ON  // 効果音付き

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define MazeWidth 7       // 迷路の横幅
#define MazeHeight 7      // 迷路の縦幅

const int screenMidW = SCREEN_WIDTH/2;   // 画面横の中央
const int screenMidH = SCREEN_HEIGHT/2;  // 画面縦の中央

// ジョイコンのピン情報
const int X_PIN = A0;  // X軸方向の入力ピン
const int Y_PIN = A1;  // Y軸方向の入力ピン
const int JSW_PIN = 6; // ジョイコンスイッチの入力ピン
const int CSW_PIN = 7; // クラッシュスイッチの入力ピン

// I2C通信のオブジェクトインスタンス
MPU6050 mpu;
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);

#ifdef AUDIO_ON
// オーディオ関連
SoftwareSerial mySoftwareSerial(2, 3); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

bool ClearSE = false;
bool StartSE = false;
bool WalkSE = false;
#endif

// MPU6050からの情報
Quaternion preq, curq;     // クォータニオン [w, x, y, z]
float euler[3];         // オイラー角 [psi, theta, phi]
uint8_t fifoBuffer[64]; // DMPにおけるFIFOストレージバッファ

// 迷路の情報
bool  maze[MazeWidth][MazeHeight];    // 迷路のマップ
const int cellsize = 10;              // マップビューでのマス目サイズ
const int ItemAllNum = MazeHeight-2;  // アイテムの総数 10以上
Item itemList[ItemAllNum];            // アイテムの情報

// 視野の情報
const int fov = 90;                       // 視野角
const int deltaAngle = 3;                 // レイキャスティングの光線幅
const int layNum = fov / deltaAngle + 1;  // 光線の数
const int layAllNum = 360 / deltaAngle;   // 光線ベクトルの総数
const int half = (layAllNum >> 1);
const int quarter = (layAllNum >> 2);
const int eighth = (layAllNum >> 3);
Vector2D unitvec[eighth + 1];             // deltaAngle 刻みの単位ベクトル (0°から45°まで)
const int wallwidth = 3;                  // 光線に対応する描画長方形の幅
const int viewmargin = (SCREEN_WIDTH - (wallwidth + 1) * layNum) / 2;
                                          // 左端の余白
int wallheight[layNum];                   // 光線に対応する描画長方形の高さ

// プレイヤーなどの情報
Player player = Player(1.5,1.5);
int tmpcenterAngleIndex = player.centerAngleIndex;

// その他の情報
int preJoyConSWState = 1;
int preCrashSWState = 1;
unsigned long previousMs = 0;
bool GameStart = false;
bool GameClear = false;

void buildMaze(){
  for(int i=0; i<MazeWidth; i++) for(int j=0; j<MazeHeight; j++){
    if( i==0 || i==MazeWidth-1 || j==0 || j==MazeHeight-1 ) maze[i][j] = true;
    else if( i%2==0 && j%2==0 ) maze[i][j] = true;
    else  maze[i][j] = false;
  }
  for(int i=2; i<MazeWidth-2; i+=2) for(int j=2; j<MazeHeight-2; j+=2){
    int r = ((j==2) ? random(4) : random(3));
    while(1){
      if(r==0 && !maze[i+1][j]){
        maze[i+1][j] = true;  break;
      }else if(r==1 && !maze[i][j+1]){
        maze[i][j+1] = true;  break;
      }else if(r==2 && !maze[i-1][j]){
        maze[i-1][j] = true;  break;
      }else if(r==3 && !maze[i][j-1]){
        maze[i][j-1] = true;  break;
      }
      r++;
      if(j==2 && r==4) r=0;
      if(j!=2 && r==3) r=0;
    }
  }
  int k=0;
  for(int j=1; j<MazeHeight-1; j++){
    int i = random(1,MazeWidth-1);
    while(maze[i][j]) if(++i>=MazeWidth-1) i=1;
    itemList[k].pos.x = i + random(1,10) / (float)10.0;
    itemList[k].pos.y = j + random(1,10) / (float)10.0;
    itemList[k++].taken = false;
  }
}

void setupMPU() {
  Wire.begin();
  Wire.setClock(400000); // I2Cクロックを400kHz
  mpu.initialize();
  uint8_t devStatus = mpu.dmpInitialize();
  
  // 調べたオフセットを設定する (!!!!要変更!!!!)
  mpu.setXAccelOffset(-344);
  mpu.setYAccelOffset(365);
  mpu.setZAccelOffset(1195);
  mpu.setXGyroOffset(11);
  mpu.setYGyroOffset(-3);
  mpu.setZGyroOffset(21);

  if (devStatus == 0) {
    // キャリブレーション (較正) 回数の設定
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
  } else {
    Serial.print("DMP Initialization failed.");
  }
}

void getUnitVec(int angleIndex, Vector2D *vec){
  bool xRev = false;
  bool yRev = false;
  bool swap = false;
  if(angleIndex < 0)   angleIndex+= layAllNum;
  if(angleIndex >= layAllNum) angleIndex -= layAllNum;
  if(angleIndex >= half){
    angleIndex = layAllNum - angleIndex;
    yRev = true;
  }
  if(angleIndex >= quarter){
    angleIndex = half - angleIndex;
    xRev = true;
  }
  if(angleIndex >= eighth){
    angleIndex = quarter - angleIndex;
    swap = true;
  }
  vec->x = unitvec[angleIndex].x;
  vec->y = unitvec[angleIndex].y;
  if(swap){
    float tmp = vec->x;
    vec->x = vec->y;  vec->y = tmp;
  }
  if(xRev)  vec->x = -vec->x;
  if(yRev)  vec->y = -vec->y;
  return;
}

void correctDistort(int angleIndex, float *len){
  Vector2D vec;
  getUnitVec(angleIndex-player.centerAngleIndex, &vec);
  *len = *len * ((abs(vec.x) < 0.001) ? 0.001 : vec.x);
}

bool hitWall(float px, float py){
  int x = floor(px);
  int y = floor(py);
  if(x<0 || y<0 || x>=MazeWidth || y>=MazeHeight) return false;
  if(px-x < 0.0001){
    if(maze[x][y]||maze[x-1][y]) return true;
  }
  if(py-y < 0.0001){
    if(maze[x][y]||maze[x][y-1]) return true;
  }
  return false;
}

float getLengthToWall(Vector2D *vec){
  float wallx = (vec->x >= 0) ? ceil(player.pos.x) : floor(player.pos.x);
  float wally = (vec->y >= 0) ? ceil(player.pos.y) : floor(player.pos.y);
  float dist, len, minlength;
  minlength = 3.0;
  bool found = false;
  for(; abs(wallx-player.pos.x)<=3.0; wallx += ((vec->x >= 0) ? 1.0 : -1.0)){
    dist = wallx-player.pos.x;
    len = dist / ((abs(vec->x) < 0.01) ? 0.01 : vec->x);
    if(hitWall(wallx, player.pos.y+len*vec->y)){
      if(minlength > len){
        found = true;
        minlength = len;  break;
      }
    }
  }
  for(; abs(wally-player.pos.y)<=3.0; wally += ((vec->y >= 0) ? 1.0 : -1.0)){
    dist = wally-player.pos.y;
    len = dist / ((abs(vec->y) < 0.01) ? 0.01 : vec->y);
    if(hitWall(player.pos.x+len*vec->x, wally)){
      if(minlength > len){
        found = true;
        minlength = len;  break;
      }
    }
  }
  if(found) return minlength;
  else  return -1;
}

int getWallHeight(int angleIndex){  
  Vector2D vec;
  getUnitVec(angleIndex, &vec);
  float len = getLengthToWall(&vec);
  if(len < 0) return 0;
  correctDistort(angleIndex, &len);
  if(len < 0.001) len = 0.001;
  return constrain((SCREEN_HEIGHT / (len * 2)), 0, SCREEN_HEIGHT);
}

void setWallHeight(){
  uint8_t j=0;
  int rightlayAngleIndex = player.centerAngleIndex - (layNum>>1);
  int leftlayAngleIndex = rightlayAngleIndex + layNum - 1;
  for(int i=leftlayAngleIndex; i >= rightlayAngleIndex; i--){
    wallheight[j++] = getWallHeight(i);
  }
}

void repaintView(){
  setWallHeight();
  u8g.firstPage();
  do {
    int leftpos = viewmargin;
    for(int i=0; i<layNum; i++){
      if(wallheight[i] > 0){
        u8g.drawBox(leftpos, screenMidH - (wallheight[i] >> 1), wallwidth, wallheight[i]);
      }
      leftpos += wallwidth + 1;
    }
    for(int i=0; i<ItemAllNum; i++) if(!itemList[i].taken){
      float dist = sqrt(getDistanceSq(&player.pos,&itemList[i].pos));
      if(dist < 3.0){
        Vector2D vec;
        vec.x = (itemList[i].pos.x - player.pos.x) / dist;
        vec.y = (itemList[i].pos.y - player.pos.y) / dist;
        int itemAngle = atan2(vec.y,vec.x) * 180 / M_PI - player.centerAngleIndex * deltaAngle;
        if(itemAngle < -180) itemAngle += 360;
        if(itemAngle > 180) itemAngle -= 360;
        if(abs(itemAngle) <= (fov>>1)){
          float len = getLengthToWall(&vec);
          if(dist < len || len < 0){
            int Itemx = SCREEN_WIDTH/2 - (itemAngle) * (wallwidth+1) / deltaAngle;
            int Itemy = SCREEN_HEIGHT - dist * 10;
            int diameter = (int)(6.0/dist);
            u8g.setColorIndex(0);
            u8g.drawDisc(Itemx,Itemy,diameter);
            u8g.setColorIndex(1); 
            u8g.drawCircle(Itemx,Itemy,diameter);
          }
        }
      }
    }
  } while ( u8g.nextPage() );
}

void repaintMap(){
  char ScoreStr[3], AllNumStr[3];
  if(player.ItemNum < 10) ScoreStr[0] = ' ';
  else  ScoreStr[0] = '0' + player.ItemNum / 10;
  ScoreStr[1] = '0' + player.ItemNum % 10;
  if(ItemAllNum < 10) AllNumStr[0] = ' ';
  else  AllNumStr[0] = '0' + ItemAllNum / 10;
  AllNumStr[1] = '0' + ItemAllNum % 10;
  ScoreStr[2] = AllNumStr[2] = '\0';
  Vector2D vec;
  
  u8g.firstPage();
  do {
    u8g.drawLine(60,0,60,60);
    u8g.drawLine(0,32,60,32);
    
    u8g.drawStr(4,4,"Score");
    u8g.drawStr(4,40,"All Items");
    u8g.drawStr(8,18,ScoreStr);
    u8g.drawStr(8,52,AllNumStr);

    u8g.drawFrame(66,2,60,60);
    u8g.drawDisc(96,32,2);
    getUnitVec(player.centerAngleIndex, &vec);
    int dx = floor(cellsize * vec.x * 0.7);
    int dy = - floor(cellsize * vec.y * 0.7);
    u8g.drawLine(96,32,96+dx,32+dy);

    int uppos = 2, downpos = 2 + (int)(cellsize * (player.pos.y - floor(player.pos.y)));
    int i, j = floor(player.pos.y) + 3;
    int leftpos, rightpos;
    while(uppos != SCREEN_HEIGHT-2 || downpos != SCREEN_HEIGHT-2){
      if(uppos != downpos){
        leftpos = 66; rightpos = 66 + (int)(cellsize * (ceil(player.pos.x) - player.pos.x));
        i = floor(player.pos.x) - 3;
        while(leftpos != SCREEN_WIDTH-2 || rightpos != SCREEN_WIDTH-2){
          if(leftpos != rightpos){
            if(0<=i && i<MazeWidth && 0<=j && j<MazeHeight && maze[i][j])
              u8g.drawBox(leftpos,uppos,rightpos-leftpos,downpos-uppos);
          }
          leftpos = rightpos; rightpos = min(rightpos+cellsize,SCREEN_WIDTH-2);
          i++;
        }
      }
      uppos = downpos; downpos = min(downpos+cellsize,SCREEN_HEIGHT-2);
      j--;
    }
    for(i=0; i<ItemAllNum; i++) if(!itemList[i].taken){
      int Itemdx = cellsize * (itemList[i].pos.x - player.pos.x);
      int Itemdy = - cellsize * (itemList[i].pos.y - player.pos.y);
      if(abs(Itemdx)<=30 && abs(Itemdy)<=30){
        u8g.drawCircle(96+Itemdx,32+Itemdy,2);
      }
    }
    u8g.setColorIndex(0);
    u8g.drawBox(66,0,60,2);
    u8g.drawBox(66,62,60,2);
    u8g.drawBox(61,2,4,60);
    u8g.drawBox(126,2,2,60);
    u8g.setColorIndex(1);
  } while ( u8g.nextPage() );
  
}

void repaintGameMessage(const char s[]){
  u8g.firstPage();
  do {
    u8g.setScale2x2();
    u8g.drawStr(2,10,s);
    u8g.undoScale();
    u8g.drawStr(12,44, "Push any button");
  } while ( u8g.nextPage() );
}

void movePlayer(int x, int y){
  float dx, dy;
  Vector2D vec1, vec2;
  getUnitVec(player.centerAngleIndex, &vec1);
  getUnitVec(player.centerAngleIndex - quarter, &vec2);
  x = ( x - 512 ) / 100;
  y = ( y - 512 ) / 100;
  dx = 0.05 * x * vec2.x + 0.05 * y * vec1.x;
  dy = 0.05 * x * vec2.y + 0.05 * y * vec1.y;
  x = floor(player.pos.x+dx);
  y = floor(player.pos.y+dy);
  if(!maze[x][y] && !hitWall(player.pos.x+dx,player.pos.y+dy)){
    player.pos.x += dx; player.pos.y += dy;
#ifdef AUDIO_ON
    if(abs(dx) > 0.2 || abs(dy) > 0.2){
      if(!WalkSE){
        myDFPlayer.volume(45);
        myDFPlayer.enableLoop();
        myDFPlayer.loop(3); // 移動音
        WalkSE = true;
      }
    }else{
      WalkSE = false;
      myDFPlayer.disableLoop();
      myDFPlayer.volume(15);
    }
#endif
  }
}

void getQuaternion(Quaternion *q){
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(q, fifoBuffer);
  }
}

void setup() {
  
  Serial.begin(115200);
#ifdef AUDIO_ON
  mySoftwareSerial.begin(9600);
  if (!myDFPlayer.begin(mySoftwareSerial)) {
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true);
  }
  myDFPlayer.volume(15);
#endif

  u8g.setColorIndex(1);
  u8g.setFont(u8g_font_6x10);
  u8g.setFontRefHeightExtendedText();
  u8g.setDefaultForegroundColor();
  u8g.setFontPosTop();
  
  setupMPU(); // MPU6050の設定

  pinMode(X_PIN, INPUT);
  pinMode(Y_PIN, INPUT);
  pinMode(JSW_PIN, INPUT_PULLUP);
  pinMode(CSW_PIN, INPUT_PULLUP);

  // 単位ベクトルの取得
  for(uint8_t i=0; i<=eighth; i++){
    unitvec[i].x = cos( i*deltaAngle/(180/PI) );
    unitvec[i].y = sin( i*deltaAngle/(180/PI) );
  }

  // 迷路ゲームの設定
  Gamereset();
}

float getDistanceSq(Vector2D *vec1, Vector2D *vec2){
  float dx = vec1->x - vec2->x;
  float dy = vec1->y - vec2->y;
  return dx*dx + dy*dy;
}

void Gamereset(){
  GameStart = false;
  GameClear = false;
  buildMaze();
  player.pos.x = 1.5; player.pos.y = 1.5;
  player.ItemNum = 0;
  player.centerAngleIndex = 0;
#ifdef AUDIO_ON
  ClearSE = false;
  StartSE = false;
#endif
}

void loop() {
  unsigned long currentMs = millis();
  int curJoyConSWState = (PIND & (1<<JSW_PIN));
  int curCrashSWState = (PIND & (1<<CSW_PIN));
  int X_POS = analogRead(X_PIN);
  int Y_POS = analogRead(Y_PIN);

  if(GameClear){
#ifdef AUDIO_ON
    if(!ClearSE){
      myDFPlayer.play(5); // クリア音
      ClearSE = true;
    }
#endif
    if(preJoyConSWState != curJoyConSWState || preCrashSWState != curCrashSWState){
      if( curJoyConSWState || !curCrashSWState){
        Gamereset();
      }
    }else{
      repaintGameMessage("Game Clear");
    }
  }else if(GameStart){
  
    if(curCrashSWState){
      getQuaternion(&preq);
      tmpcenterAngleIndex = player.centerAngleIndex;
    }else{
      getQuaternion(&curq);
      Quaternion offq = curq.getProduct(preq.getConjugate());
      mpu.dmpGetEuler(euler, &offq);
      player.centerAngleIndex = tmpcenterAngleIndex - (euler[0] * 180/M_PI) / deltaAngle;
    }
    
    if(currentMs - previousMs >= 10){
      movePlayer(X_POS,Y_POS);
      for(int i=0; i<ItemAllNum; i++) if(!itemList[i].taken){
        if(getDistanceSq(&player.pos,&itemList[i].pos)<0.16){
#ifdef AUDIO_ON
          myDFPlayer.disableLoop();
          myDFPlayer.volume(15);
          myDFPlayer.play(2); // 獲得音
          myDFPlayer.enableLoop();
          delay(1500);
          WalkSE = false;
#endif
          itemList[i].taken = true;
          if(++player.ItemNum == ItemAllNum) GameClear = true;
        }
      }
      if(curJoyConSWState){
#ifdef AUDIO_ON
        if(preJoyConSWState != curJoyConSWState)  myDFPlayer.play(4); // 場面転換
#endif
        repaintMap();
      }else  repaintView();
      previousMs = currentMs;
    }
  }else{
#ifdef AUDIO_ON
    if(!StartSE){
      myDFPlayer.enableLoop();
      myDFPlayer.loop(1); // スタート画面
      StartSE = true;
    }
#endif
    if(preJoyConSWState != curJoyConSWState || preCrashSWState != curCrashSWState){
      if( curJoyConSWState || !curCrashSWState){
        GameStart = true;
#ifdef AUDIO_ON
        myDFPlayer.disableLoop();
#endif
      }
    }else{
      repaintGameMessage("Maze Game");
    }
  }
  preJoyConSWState = curJoyConSWState;
  preCrashSWState = curCrashSWState;
}
