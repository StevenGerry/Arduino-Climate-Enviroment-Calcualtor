//大气质量检测仪　空気研究計量器
//プログラマー　１８１０１６３ 黄文浩
//中二信息技术研究院  中二情報技術研究室
//2018.10.10 DS18B20测试+屏幕调试
//2018.10.18 DHT11温湿度检测组建测试（气压件调试失败待解决）
//2018.10.21 代码结构优化
//2018.11.1 PM2.5+气体检测调试
//2018.11.8 整体测试优化调试
//2018.12.22 结题优化

#include <OneWire.h>
#include <Wire.h>
#include <BH1750.h>
#include <U8glib.h>
#include <DHT.h>
#include <Adafruit_BMP085.h>

//气压---------------------------------
Adafruit_BMP085 bmp;

//湿度原件-----------------------------
//定义针脚
#define DHTPIN 24
//定义类型，DHT11或者其它
#define DHTTYPE DHT11
//进行初始设置 
DHT dht(DHTPIN, DHTTYPE);

//MQ-5---------------------------------
#define MQD 32
#define MQA A2

//Screen-------------------------------
U8GLIB_NHD_C12864 u8g(36, 38, 40, 42, 44);  // SPI Com: SCK = 36, MOSI = 38, CS = 40, RS = 42, RST = 44

//Light--------------------------------
BH1750 lightMeter; 

//Temp---------------------------------
OneWire  ds(22);  //

//PM2.5--------------------------------
int measurePin = 0;  // 模0
int ledPower = 46;    // 数46
  
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;
  
float voMeasured = 0;
float calcVoltage = 0;
float dustDensity = 0;


void setup(void) {
  Serial.begin(9600);
  u8g.setRot180();
  u8g.setFont(u8g_font_6x12);
  u8g.setFontRefHeightText();
  u8g.setFontPosTop();
  lightMeter.begin();//启动感光元件
  pinMode(26, OUTPUT);//光度提示LED针脚
  pinMode(25, OUTPUT);//湿度提示LED针脚
  pinMode(MQD,INPUT);
  pinMode(MQA,INPUT);
  pinMode(ledPower,OUTPUT);//PM2.5
  dht.begin(); //DHT开始工作
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}
  }
}
 
void loop(void) {

  analogWrite(26, 255);//蜂鸣器信号控制
  
//空气质量检测---------------------------------------------------------
  digitalWrite(ledPower,LOW);       //开启内部LED
  delayMicroseconds(samplingTime);  // 开启LED后的280us的等待时间
  
  voMeasured = analogRead(measurePin);   // 读取模拟值
  
  delayMicroseconds(deltaTime);        //  40us等待时间
  digitalWrite(ledPower,HIGH);         // 关闭LED
  delayMicroseconds(sleepTime);
  
  // 0 - 5V mapped to 0 - 1023 integer values
  // recover voltage
  calcVoltage = voMeasured * (5.0 / 1024.0);   //将模拟值转换为电压值
  calcVoltage = calcVoltage+0.6;
  dustDensity = (0.17 * (calcVoltage)-0.1)*1000;
  
  Serial.print("Raw Signal Value (0-1023): ");
  Serial.println(voMeasured);
  
  Serial.print("Voltage: ");
  Serial.println(calcVoltage);
  
  Serial.print("Dust Density: ");
  Serial.println(dustDensity); // unit: mg/m3


//Gas Dust------------------------------------
  float valgasinput;
  float valgas;
  valgasinput=analogRead(MQA);   //传感器接于模拟口0

if(digitalRead(MQD)==HIGH)
    {
      Serial.println("There are something smell not good...!");
      Serial.print("value:");
      Serial.println(analogRead(MQA));
      if (valgasinput>100) {
        analogWrite(26, 0);
        delay(50);
        analogWrite(26, 500);
        delay(50);
        analogWrite(26, 0);
        delay(50);
        analogWrite(26, 500);
        delay(50);
        }//非正常情况下的报警蜂鸣延时
    }
    else
    {
      Serial.println("Nothing");
    }
  delay(10);
//----------------------------------------


//Humidity--------------------------------------
  // 两次检测之间，要等几秒钟，这个传感器有点慢。
  delay(250);
  // 读温度或湿度要用250毫秒
  float h = dht.readHumidity();//读湿度
  float t = dht.readTemperature();//读温度，默认为摄氏度
  Serial.print("Humidity: ");//湿度
  Serial.println(h);


if (h==0) {
        analogWrite(26, 0);
        delay(50);
        analogWrite(26, 500);
        delay(50);
        analogWrite(26, 0);
        delay(50);
        analogWrite(26, 500);
        delay(50);
        }//非正常情况下的报警蜂鸣延时

//DS18B20---------------------------------------------------
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;
   
  if ( !ds.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    ds.reset_search();
    delay(50);
    return;
  }
   
  //Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    //Serial.print(addr[i], HEX);
  }
 
  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();
  
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  }
 
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        //start conversion
   
  delay(750);     //750ms

   
  present = ds.reset();
  ds.select(addr);   
  ds.write(0xBE);         // Read Scratchpad
 
  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
  }
  celsius = (float)raw / 16.0;
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.println(" Celsius");

  
  analogWrite(26, 0);//蜂鸣器信号控制



//气压----------------------------------
  Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println("  C");
    int bmptemp;
    bmptemp=bmp.readTemperature();
    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.println(" meters");
    Serial.print("Pressure at sealevel (calculated) = ");
    Serial.print(bmp.readSealevelPressure());
    Serial.println(" Pa");
    Serial.print("Real altitude = ");
    Serial.print(bmp.readAltitude(101500));
    Serial.println(" meters");
    
    Serial.println();
    delay(500);
  

//屏幕显示----------------------------------------------
  int val;
  int dat;
  val=int(celsius);
  //dat=(125*val)>>8;

  uint16_t lux = lightMeter.readLightLevel();
 
  u8g.firstPage();  
  do {
    u8g.drawStr(0, 0, "Temperature:");
    u8g.drawStr(80, 0, " Celsius");
    u8g.setPrintPos(72, 0);
    u8g.print((val+bmptemp)/2);
    u8g.drawStr(0, 12, "Pressure:");
    u8g.drawStr(85, 12, "HPa");
    u8g.setPrintPos(58, 12);
    u8g.print(bmp.readPressure()/100); 
    u8g.drawStr(0, 25, "PM2.5:");
    u8g.drawStr(65, 25, " ug/m3");
    u8g.setPrintPos(35, 25);
    u8g.print(dustDensity*3); 
    u8g.drawStr(0, 37, "Humidity:");
    u8g.drawStr(68, 37, "%");
    u8g.setPrintPos(55, 37);
    u8g.print(int(h)); 
    if(digitalRead(MQD)==HIGH){
    u8g.drawStr(0, 49, "DangerGas:");
    u8g.setPrintPos(60, 49);
    u8g.print(valgasinput); }
    else{
    u8g.drawStr(0, 49, "DangerGas:Nothing");  
    }
  } while( u8g.nextPage() );
  delay(50);
}
