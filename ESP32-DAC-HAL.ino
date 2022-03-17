#include <Arduino.h>
#include <esp32-hal-dac.h>//DAC功能引用該庫，可以不進行#include<>引用

//查看源碼後得到，DAC引腳只能是這兩個引腳
#define LED1 25
#define LED2 26
 
void setup(){
  //長時候發現，DAC的IO口也可以不進行初始化
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
}
 
void loop()
{
  // 逐漸變亮
  for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle = dutyCycle + 1)
  {
    dacWrite(LED1, dutyCycle);  // 輸出DAC
    dacWrite(LED2, 255 - dutyCycle);  // 輸出DAC
    delay(5);
  }
 
  // 逐漸變暗
  for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle = dutyCycle - 1)
  {
    dacWrite(LED1, dutyCycle);  // 輸出DAC
    dacWrite(LED2, 255 - dutyCycle);  // 輸出DAC
    delay(5);
  }
}
