int16_t ADCvalue;

void setup(){
  Serial.begin(9600);
}

void loop() {
  ADCvalue = analogRead(A0);
  Thermistor(ADCvalue);
}
void Thermistor(int16_t ADCvalue)
{
  double T, Temp;
  double T0 = 301.15;  // 273.15 + 28 (room temperature) 室溫換成絕對溫度
  double lnR;
  int16_t R;          // Thermistor resistence 
  int16_t R0 = 8805;  // calibrated by reading R at room temperature (=28 degree Celsius )
  int16_t B  = 3950;
  int16_t Pullup = 9930; // 10K ohm
  
  // R / (Pullup + R)   = adc / 4096
  R = (Pullup * ADCvalue) / (4096 - ADCvalue);
  
  // B = (log(R) - log(R0)) / (1/T - 1/T0) 
  T = 1 / (1/T0 + (log(R)-log(R0)) / B );
  Temp = T - 273.15;  
    
  Serial.println(Temp);
}
