
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>            
#include <TimeLib.h>             
#include <ErriezRotaryFullStep.h>

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, 3600);

const char *ssid     = "SSID";
const char *password = "Password";

uint8_t din   = D7; // DA
uint8_t clk   = D5; // CK
uint8_t cs    = D8; // CS
uint8_t Reset = D0; // RS
uint8_t rot_left = D2;
uint8_t rot_right = D1;
uint8_t rot_but = D4;


char *str_time = "00:00:00";
String format_time = "00:00:00";

int lastBrightnessSwitch = 0;
volatile int brightness = 100;
unsigned long epoch = 0;
int ntpOffset = 3600;

RotaryFullStep rotary(rot_left, rot_right, true, 250);

void setup(){
  pinMode(clk, OUTPUT);
  pinMode(din, OUTPUT);
  pinMode(cs, OUTPUT);
  pinMode(Reset, OUTPUT);
  digitalWrite(Reset, LOW);
  delayMicroseconds(5);
  digitalWrite(Reset, HIGH);
  VFD_init();
  S1201_WriteStr(0, "Hello!");
  
  // Encoder bits
  pinMode(rot_left, INPUT);
  pinMode(rot_right, INPUT);
  pinMode(rot_but, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rot_left), rotaryInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rot_right), rotaryInterrupt, CHANGE);
  
  delay(2000);
  WiFi.begin(ssid, password);
  Serial.begin(115200);
  Serial.print("Connecting.");
  S1201_WriteStr(0, "Initing ");
  delay(1000);
  S1201_WriteStr(0, "WiFI");
  while ( WiFi.status() != WL_CONNECTED ) {
    S1201_WriteStr(0, "Beep    ");
    delay(500);
    S1201_WriteStr(0, "    Boop");
    delay(500);
  }
  S1201_WriteStr(0, "WiFi OK!");
  Serial.println("connected");
  delay(1000);
  S1201_WriteStr(0, "LoadTime");
  timeClient.begin();
  delay(1000);
}

void loop()
{
  if (digitalRead(rot_but) == 0) {
    if(ntpOffset == 3600) {
      ntpOffset = 7200;
    } else {
      ntpOffset = 3600;
    }
    timeClient.setTimeOffset(ntpOffset);
    delay(2000);
    timeClient.forceUpdate();
  }
  
  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();
  if(epoch != 0 && (epochTime - epoch) > 30) {
    epoch = epochTime; 
    struct tm *ptm = gmtime ((time_t *)&epochTime); 
    int monthDay = ptm->tm_mday;
    int currentMonth = ptm->tm_mon+1;
    int currentYear = ptm->tm_year+1900;
    String currentDate = preludeZero(monthDay) + "/" + preludeZero(currentMonth) + "/" + partOfYear(currentYear);
    char str_date[9];
    currentDate.toCharArray(str_date, 9);
    S1201_WriteStr(0, str_date);
    delay(5000);
  } else {
    if(epoch == 0) {
      epoch = epochTime; 
    }

    if((timeClient.getHours() > 19 || timeClient.getHours() < 8) && lastBrightnessSwitch != 20) {
      Serial.println("Set Brightness to 10");
      lastBrightnessSwitch = 20;
      brightness = 10;
    } else if ((timeClient.getHours() < 20 && timeClient.getHours() > 7) && lastBrightnessSwitch != 8) {
      Serial.println("Set Brightness to 100");
      lastBrightnessSwitch = 8;
      brightness = 100;
    }
    
    format_time = timeClient.getFormattedTime();
    char *str_time = &format_time[0]; 
    S1201_WriteStr(0, str_time);
    delay(500);
  }
  Serial.println(brightness);
  VFD_init();
}

String preludeZero(int mom) {
  if(mom < 10) {
    String result = "0";
    result.concat(String(mom));
    return result;
  } else {
    return String(mom);
  }
}
String partOfYear(int currentYear) {
  // This function will fail in 179 years as of writing this. If this device still works by then: Hahahaha.
  int yr = 0;
  if(currentYear > 2099) {
   yr = currentYear - 2100;
  } else {
    yr = currentYear - 2000;
  }
  return preludeZero(yr);
}

void write_6302(unsigned char w_data)
{
  unsigned char i;
  for (i = 0; i < 8; i++)
  {
    digitalWrite(clk, LOW);
    if ( (w_data & 0x01) == 0x01)
    {
      digitalWrite(din, HIGH);
    }
    else
    {
      digitalWrite(din, LOW);
    }
    w_data >>= 1;
    digitalWrite(clk, HIGH);
  }
}

void VFD_cmd(unsigned char command)
{
  digitalWrite(cs, LOW);
  write_6302(command);
  digitalWrite(cs, HIGH);
  delayMicroseconds(5);
}

void S1201_show(void)
{
  digitalWrite(cs, LOW);//开始传输
  write_6302(0xe8);     //地址寄存器起始位置
  digitalWrite(cs, HIGH); //停止传输
}

void VFD_init()
{
  //SET HOW MANY digtal numbers
  digitalWrite(cs, LOW);
  write_6302(0xe0);
  delayMicroseconds(5);
  write_6302(0x07);//8 digtal
  digitalWrite(cs, HIGH);
  delayMicroseconds(5);

  //set bright
  digitalWrite(cs, LOW);
  write_6302(0xe4);
  delayMicroseconds(5);
  write_6302(brightness);//leve 255 max
  digitalWrite(cs, HIGH);
  delayMicroseconds(5);
}

/******************************
  在指定位置打印一个字符(用户自定义,所有CG-ROM中的)
  x:0~11;chr:要显示的字符编码
*******************************/
void S1201_WriteOneChar(unsigned char x, unsigned char chr)
{
  digitalWrite(cs, LOW);  //开始传输
  write_6302(0x20 + x); //地址寄存器起始位置
  write_6302(chr + 0x30);
  digitalWrite(cs, HIGH); //停止传输
  S1201_show();
}
/******************************
  在指定位置打印字符串
  (仅适用于英文,标点,数字)
  x:0~11;str:要显示的字符串
*******************************/
void S1201_WriteStr(unsigned char x, char *str)
{
  digitalWrite(cs, LOW);  //开始传输
  write_6302(0x20 + x); //地址寄存器起始位置
  while (*str)
  {
    write_6302(*str); //ascii与对应字符表转换
    str++;
  }
  digitalWrite(cs, HIGH); //停止传输
  S1201_show();
}

ICACHE_RAM_ATTR void rotaryInterrupt()
{
  int rotaryState = rotary.read();
  int bright_start = brightness;
  int bright_new = bright_start + (rotaryState*10);
  if(bright_new < 5) {
    bright_new = 5;
  } else if(bright_new > 250) {
    bright_new = 250;
  }
  brightness = bright_new;
  
  // rotaryState = -3: Counter clockwise turn, multiple notches fast
  // rotaryState = -2: Counter clockwise turn, multiple notches
  // rotaryState = -1: Counter clockwise turn, single notch
  // rotaryState = 0:  No change
  // rotaryState = 1:  Clockwise turn, single notch
  // rotaryState = 2:  Clockwise turn, multiple notches
  // rotaryState = 3:  Clockwise turn, multiple notches fast
}
