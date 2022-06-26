#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <EEPROM.h>

#include "RC.h"

#define WIFI_CHANNEL 4//定義WIFI腳位為4
#define PWMOUT  // normal esc, uncomment for serial esc
#define LED 2//定義LED燈腳位為2
#define CALSTEPS 256 // gyro and acc calibration steps

extern int16_t accZero[3];//設置accZero[3]靜態保證16位元整數之變數

///引入PID.ino的參數值////
extern float yawRate;//設置yawRate浮點數變數//
extern float rollPitchRate;//設置rollPitchRate浮點數變數//
extern float P_PID;//設置P_PID浮點數變數//
extern float I_PID;//設置I_PID浮點數變數//
extern float D_PID;//設置D_PID浮點數變數//
extern float P_Level_PID;//設置P_Level_PID浮點數變數//
extern float I_Level_PID;//設置I_Level_PID浮點數變數//
extern float D_Level_PID;//設置D_Level_PID浮點數變數//



volatile boolean recv;//設置volaite布林數 recv
//volatile int peernum = 0;
//esp_now_peer_info_t slave;

void recv_cb(const uint8_t *macaddr, const uint8_t *data, int len)//宣告函式 recv_cb
{
  recv = true;//將recv賦予true值
  //Serial.print("recv_cb ");
  //Serial.println(len); 
  if (len == RCdataSize) //假設(此式左右兩邊LEN與RCdataSizeg相等時)
  {
    for (int i=0;i<RCdataSize;i++) RCdata.data[i] = data[i];//判斷i是否小於RCdateSize，其中i初始值等於0，下一次循環為i+1，回傳值date[i]由RCdate.data[i] 變成 
  }
  /*
  if (!esp_now_is_peer_exist(macaddr))
  {
    Serial.println("adding peer ");
    esp_now_add_peer(macaddr, ESP_NOW_ROLE_COMBO, WIFI_CHANNEL, NULL, 0);
    peernum++;
  }
  */
};

#define ACCRESO 4096 //定義 ACCRESO 4096
#define CYCLETIME 3 //定義 CYCLETIME 3
#define MINTHROTTLE 1090 //定義 MINTHROTTLE 1090
#define MIDRUD 1495 //定義 MIDRUD 1495
#define THRCORR 19 //定義 THRCORR 19

enum ang { ROLL,PITCH,YAW };//列舉 ang{ROLL,PITCH,YAW}

static int16_t gyroADC[3]; //設置gyroADC[3]靜態保證16位元整數之變數 
static int16_t accADC[3]; //設置accADC[3]靜態保證16位元整數之變數  
static int16_t gyroData[3]; //設置gyroData[3]靜態保證16位元整數之變數  
static float angle[2]    = {0,0}; //設置angle[2]靜態浮點數變數，且其值皆為0
extern int calibratingA; //設置cailbratingA整數變數

#ifdef flysky //假如定義為flysky 就定義 ROL 0 PIT 1 THR 2 RUD 3
  #define ROL 0
  #define PIT 1
  #define THR 2
  #define RUD 3
#else //orangerx //反之 定義ROL 1 PIT 2 THR 0 RUD 3
  #define ROL 1
  #define PIT 2
  #define THR 0
  #define RUD 3
#endif //假設結束

#define AU1 4 //定義 AU1 4
#define AU2 5 //定義 AU2 5
static int16_t rcCommand[] = {0,0,0};//設置靜態保證16位元整數之變數rcCommand[]，且其值都為0

#define GYRO     0 //定義 GYRO     0
#define STABI    1 //定義 STABI    1
static int8_t flightmode;//設置靜態保證8位元整數之變數flightmode
static int8_t oldflightmode;//設置靜態保證8位元整數之變數oldflightmode

boolean armed = false; //設置布林值 armed = flase
uint8_t armct = 0; //設置unsigned8位元 armct =0
int debugvalue = 0;//設置整數 debugvalue = 0  


void setup() 
{
  Serial.begin(115200); Serial.println();//序列埠鲍率115200時印出空白並換行

  delay(3000); // give it some time to stop shaking after battery plugin(延遲3秒)
  MPU6050_init();//引入MPU6050.ino 的MPU6050_init()函式
  MPU6050_readId(); // must be 0x68, 104dec 引入MPU6050.ino 的MPU6050_readId()函式
  
  EEPROM.begin(64); //EEPROM.begin(size)
  if (EEPROM.read(63) != 0x55) Serial.println("Need to do ACC calib");//若EEPROM.read(size=63)不等於0x55這個位子 ，序列埠即印出("Need to do ACC calib")
  else ACC_Read(); // eeprom is initialized 
  if (EEPROM.read(62) != 0xAA) Serial.println("Need to check and write PID");
  //若EEPROM.read(size=62)不等於0xAA這個位子 ，序列埠即印出("Need to check and write PID")
  else PID_Read(); // eeprom is initialized

  
  WiFi.mode(WIFI_STA); // Station mode for esp-now 
  #if defined webServer //假設定義 webServer
    setupwebserver(); //執行setupwebserver()
    delay(500); //延遲(500毫秒)
  #endif //假設結束


  #if defined externRC //假設定義 externRC
    init_RC(); //執行init_RC()
  #else//反之
    Serial.printf("This mac: %s, ", WiFi.macAddress().c_str()); //序列埠印出"This mac:其字串將獲取 WiFi shield 的 MAC 地址"
    Serial.printf(", channel: %i\n", WIFI_CHANNEL); //序列埠印出", channel:其字串將獲取 WIFI_CHANNEL"
    if (esp_now_init() != 0) Serial.println("*** ESP_Now init failed"); //假設 (esp_now_init() 不等於 0) 序列埠印出"*** ESP_Now init failed"
    esp_now_register_recv_cb(recv_cb); //執行esp_now_register_recv_cb(recv_cb) 函式
  #endif//假設結束

  delay(500);//延遲0.5S
  pinMode(LED,OUTPUT);//設置LED腳位為OUTPUT
  digitalWrite(LED,LOW);//將LED腳位狀態設置為低電位
  initServo();//執行initServo()函式
}

uint32_t rxt; // receive time, used for falisave
//非負32位元數 rxt 

void loop() 
{
  uint32_t now,mnow,diff; 
  //非負32位元數 now,mnow,diff
  now = millis(); // actual time
  //now = 毫秒
  if (debugvalue == 5) mnow = micros();
  //假如 (此式5與debugvalue左右兩值相等)  mnow = 微秒()
  #if defined webServer //假設定義 webServer
    loopwebserver(); //執行 loopwebserver()
  #endif //假設結束

  if (recv) //假設(recv變數)
  {
    recv = false; //將recv賦予false值
    #if !defined externRC //假設不定義 externRC
      buf_to_rc(); //執行 buf_to_rc()
    #endif //假設結束

    if (debugvalue == 4) Serial.printf("%4d %4d %4d %4d \n", rcValue[0], rcValue[1], rcValue[2], rcValue[3]); 
    //假如debugvalue == 4 序列埠印出(四個長度為四的十進位整數 換行 陣列rcValue[0], rcValue[1], rcValue[2], rcValue[3]的值 )
    if      (rcValue[AU1] < 1300) flightmode = GYRO;
    //假如(rcValue[AU1] 小於 1300) 將flightmode賦予GYRO值
    else                          flightmode = STABI;  
    //否則 將flightmode賦予STABI值
    if (oldflightmode != flightmode)//假如oldflightmode 不等於 flightmode
    {
      zeroGyroAccI();//執行zeroGyroAccI()函式
      oldflightmode = flightmode;//將oldflightmode賦予flightmode之值
    }

    if (armed) //假設(armed變數)
    {
      rcValue[THR]    -= THRCORR;//將rcValue[THR]賦予rcValue[THR]-THRCORR值
      rcCommand[ROLL]  = rcValue[ROL] - MIDRUD;//將rcCommand[ROLL]賦予rcValue[ROL] - MIDRUD值
      rcCommand[PITCH] = rcValue[PIT] - MIDRUD;//將rcCommand[PITCH]賦予rcValue[PIT] - MIDRUD值
      rcCommand[YAW]   = rcValue[RUD] - MIDRUD;//將rcCommand[YAW]賦予rcValue[RUD] - MIDRUD值
    else //反之
    {  
     if (rcValue[THR] < MINTHROTTLE) armct++;//若 (rcValue[THR] 大於 MINTHROTTLE)arcmt值每次加1
      if (armct >= 25)//假設(armct 大於等於 25) 
      { 
        digitalWrite(LED,HIGH); //改變LED腳位電壓為高電壓
        armed = true;//將armed賦予正布林值
      }
    }

    if (debugvalue == 5) Serial.printf("RC input ms: %d\n",now - rxt);//假設(此式debugvalue與5等號兩邊相等)序列埠印出字串RC input ms
    rxt = millis();//將rxt賦予毫秒值
  }

  Gyro_getADC();//執行函式 Gyro_getADC()
  
  ACC_getADC();//執行函式 ACC_getADC()

  getEstimatedAttitude();//執行函式 getEstimatedAttitude()

  pid();//執行函式 pid()

  mix();//執行函式 mix()

  writeServo();//執行函式 writeServo()
  
  // Failsave part
 if (now > rxt+90)///假設(now 大於 rxt+90)
  {
    rcValue[THR] = MINTHROTTLE;///將rcValue[THR]賦予MINTHROTTLE
    if (debugvalue == 5) Serial.printf("RC Failsafe after %d \n",now-rxt);//假設(此式debugvaluem與5左右兩邊相等)序列埠印出字串RC Failsafe after %d \n
    rxt = now;//將rxt賦予now值
  }

  // parser part
  if (Serial.available()) //判斷串列埠緩衝區有無資料
  {
    char ch = Serial.read();//將字元 ch 賦予序列埠讀到的值
    // Perform ACC calibration
    if (ch == 10) Serial.println();//假設(此式ch與10左右兩邊相等)序列埠印出值
   else if (ch == 'A');//反之則假設(此式ch與A左右兩邊相等)
    { 
     Serial.println("Doing ACC calib");//序列埠印出字串Doing ACC calib
      calibratingA = CALSTEPS;//將calibratingA 賦予 CALSTEPS值
      while (calibratingA != 0)//當calibratingA 不等於 0
      {
        delay(CYCLETIME);//延遲(CYCLETIME)
        ACC_getADC(); //執行ACC_getADC()函式
      }
      ACC_Store();//執行 ACC_Store()函式
      Serial.println("ACC calib Done");//序列埠印出ACC calib Done
    }
    else if (ch == 'R')//假使字元等於R
    {

      /*序列埠印出(Act Rate :yawRate的值   rollPitchRate的值
                  Act PID :
                  P_PID的值  I_PID的值  D_PID的值
                  P_Level_PID的值  I_Level_PID的值  D_Level_PID的值
                  )*/
      Serial.print("Act Rate :  ");
      Serial.print(yawRate); Serial.print("  ");
      Serial.print(rollPitchRate); Serial.println();
      Serial.println("Act PID :");
      Serial.print(P_PID); Serial.print("  ");
      Serial.print(I_PID); Serial.print("  ");
      Serial.print(D_PID); Serial.println();
      Serial.print(P_Level_PID); Serial.print("  ");
      Serial.print(I_Level_PID); Serial.print("  ");
      Serial.print(D_Level_PID); Serial.println();
    }
    else if (ch == 'D')/假使字元等於D
    {
      Serial.println("Loading default PID");//序列埠印出Loading default PID
      yawRate = 6.0; //將yawRate 賦予 值6
      rollPitchRate = 5.0; //將 rollPitchRate 賦予 值5
      P_PID = 0.15;    // P8 //將P_PID 賦予 值0.15
      I_PID = 0.00;    // I8 //將I_PID 賦予 值0.00
      D_PID = 0.08; //將D_PID 賦予 值0.08
      P_Level_PID = 0.35;   // P8 //將P_Level_PID 賦予 值0.35
      I_Level_PID = 0.00;   // I8 //將I_Level_PID 賦予 值0.00
      D_Level_PID = 0.10; //將D_Level_PID 賦予 值0.10
      PID_Store(); //執行PID_Store()
    }
    else if (ch == 'W') //反之則 (此式左右兩邊ch與W相等)
    {
      char ch = Serial.read();//序列埠讀到的值
      int n = Serial.available();//序列埠緩衝區的值
      if (n == 3)//假設 (此式左右兩邊n與3相等)
      {
        //根據序列埠讀到的值去改變值並印初期值為多少
        n = readsernum(); //將n  賦予 readsernum()出來之值
        if       (ch == 'p') { P_PID       = float(n) * 0.01 + 0.004; Serial.print("pid P ");       Serial.print(P_PID); }
        
        //假設 (此式左右兩邊ch與字元p相等){將P_PID賦予float(n) * 0.01 + 0.004值;序列埠印出"pid P "與P_PID讀取值}
        
        else if (ch == 'i') { I_PID       = float(n) * 0.01 + 0.004; Serial.print("pid I ");       Serial.print(I_PID); }
        
        //假設上述皆不成立則 (此式左右兩邊ch與字元i相等){將I_PID賦予float(n) * 0.01 + 0.004值;序列埠印出"pid I "與I_PID讀取值}
        
        else if (ch == 'd') { D_PID       = float(n) * 0.01 + 0.004; Serial.print("pid D ");       Serial.print(D_PID); }
        
        //假設上述皆不成立則 (此式左右兩邊ch與字元d相等){將D_PID賦予float(n) * 0.01 + 0.004值;序列埠印出"pid D "與D_PID讀取值}
        
        else if (ch == 'P') { P_Level_PID = float(n) * 0.01 + 0.004; Serial.print("pid Level P "); Serial.print(P_Level_PID); }
        
        //假設上述皆不成立則 (此式左右兩邊ch與字元P相等){將P_Level_PID賦予float(n) * 0.01 + 0.004值;序列埠印出"pid  Level P "與P_Level_PID讀取值}
        
        else if (ch == 'I') { I_Level_PID = float(n) * 0.01 + 0.004; Serial.print("pid Level I "); Serial.print(I_Level_PID); }
        
        //假設上述皆不成立則 (此式左右兩邊ch與字元I相等){將I_Level_PID賦予float(n) * 0.01 + 0.004值;序列埠印出"pid  Level I "與I_Level_PID讀取值}
        
        else if (ch == 'D') { D_Level_PID = float(n) * 0.01 + 0.004; Serial.print("pid Level D "); Serial.print(D_Level_PID); }
        
        //假設上述皆不成立則 (此式左右兩邊ch與字元D相等){將D_Level_PID賦予float(n) * 0.01 + 0.004值;序列埠印出"pid  Level D "與D_Level_PID讀取值}
        
        else Serial.println("unknown command");if不成立

        //假設上述皆不成立則序列埠印出"unknown command"
      }
      else if (ch == 'S') { PID_Store(); Serial.print("stored in EEPROM"); }
      
      //假設if不成立則 (此式左右兩邊ch與字元S相等){執行PID_Store()函式;序列埠印出"stored in EEPROM"
      
//    else //反之
     
      {
        Serial.println("Input format wrong");//序列埠印出"Input format wrong"
        Serial.println("Wpxx, Wixx, Wdxx - write gyro PID, example: Wd13");
        //序列埠印出"Wpxx, Wixx, Wdxx - write gyro PID, example: Wd13"
        Serial.println("WPxx, WIxx, WDxx - write level PID, example: WD21");
        //序列埠印出"WPxx, WIxx, WDxx - write level PID, example: WD21"
      }
    }
    else if (ch >= '0' && ch <='9') debugvalue = ch -'0';
    else
    {
      /*序列埠印出(A - acc calib
                 D - write default PID
                 R - read actual PID
                 Wpxx, Wixx, Wdxx - write gyro PID
                 WPxx, WIxx, WDxx - write level PID
                 WS - Store PID in EEPROM
                 Display data:
                 0 - off
                 1 - Gyro values
                 2 - Acc values
                 3 - Angle values
                 4 - RC values
                 5 - Cycletime*/
      Serial.println("A - acc calib");
      Serial.println("D - write default PID");
      Serial.println("R - read actual PID");
      Serial.println("Wpxx, Wixx, Wdxx - write gyro PID");
      Serial.println("WPxx, WIxx, WDxx - write level PID");
      Serial.println("WS - Store PID in EEPROM");
      Serial.println("Display data:");
      Serial.println("0 - off");
      Serial.println("1 - Gyro values");
      Serial.println("2 - Acc values");
      Serial.println("3 - Angle values");
      Serial.println("4 - RC values");
      Serial.println("5 - Cycletime");
    }
  }

  if      (debugvalue == 1) Serial.printf("%4d %4d %4d \n", gyroADC[0], gyroADC[1], gyroADC[2]);  
  else if (debugvalue == 2) Serial.printf("%5d %5d %5d \n", accADC[0], accADC[1], accADC[2]);
  else if (debugvalue == 3) Serial.printf("%3f %3f \n", angle[0], angle[1]); 
  
  delay(CYCLETIME-1);  

  if (debugvalue == 5) 
  {
    diff = micros() - mnow;
    Serial.println(diff); 
  }
}


//宣告一個函式readsernum
int readsernum()
{
  int num;
  char numStr[3];  
  numStr[0] = Serial.read();
  numStr[1] = Serial.read();
  return atol(numStr);
}
