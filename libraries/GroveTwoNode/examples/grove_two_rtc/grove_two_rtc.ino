#include <Wire.h>
#include <DS1307.h>

DS1307 clock;
String GetYMDTime()
{
  String year, month, day, ymd;
  clock.getTime();
  year = String(clock.year+2000);
  month = String(clock.month);
  day = String(clock.dayOfMonth);
  ymd = year + "/" + month  + "/" + day;
  return ymd;
}

String GetHMMTime()
{
    clock.getTime();
    String hour, minute, second, hmm;
    clock.getTime();
    hour = String(clock.hour);
    minute = String(clock.minute);
    second = String(clock.second);
    hmm = hour + ":" + minute  + ":" + second;
    return hmm;
}

void setup(){
  Serial.begin(9600);
  clock.begin();
  clock.fillByYMD(2020, 0, 0);
  clock.fillByHMS(12, 0, 0);
  clock.setTime();

}

void loop(){

  Serial.println(GetYMDTime());
  Serial.println(GetHMMTime());
  delay(1000);

}