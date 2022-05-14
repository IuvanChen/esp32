#include<WiFi.h>

const char ssid[]="艾凡陳的iPhone"; //修改為你家的WiFi網路名稱
const char pwd[]="05160819"; //修改為你家的WiFi密碼

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA); //設置WiFi模式
  WiFi.begin(ssid,pwd); 

  Serial.print("WiFi connecting");

  //當WiFi連線時會回傳WL_CONNECTED，因此跳出迴圈時代表已成功連線
  while(WiFi.status()!=WL_CONNECTED){
    Serial.print(".");
    delay(500);   
  }

  Serial.println("");
  Serial.print("IP位址:");
  Serial.println(WiFi.localIP()); //讀取IP位址
  Serial.print("WiFi RSSI:");
  Serial.println(WiFi.RSSI()); //讀取WiFi強度
  

}

void loop() {
}
