/*****************************************************************************
* Berlin-Uhr                                                                 *
* Version 1.0 Stand Januar 2016                                              *
*Uhrzeit ermiteln und auf einer Reihe von LED in Form der Berlin-Uhr         *
* darstellen                                                                 *
*Die Uhrzeit wird über einen RTC-Baustein DS3231 ermittelt. Mit 2 Tastern    *
*können die Stunden und Minuten gestellt werden. Immer nach vorne und ggf.   *
*im Kreis. Die Taster werden über die 2 vom Arduino Uno beireitgestellten    *
*Interrupts abgefangen. Für die Kommunikation mit dem RTC-Baustein wird das  *
*I2C-Protokoll verwendet. Die LED werden mit einem MAX7219 über eine Bibli-  *
*othek angesteuert.                                                          *
*Die Sekungen werden über eine im 2-Sekundentakt blinkende LED dargestellt.  *
*Die Datumsinformationen werden nicht benötigt.                              *
*                                                                            *
*****************************************************************************/
#include "Wire.h"
#include "LedControl.h"
// 2 Basiswerte zum Entprellen der Stelltaster
volatile unsigned long alteZeit = 0,
                       entprell = 20;
// Globale Felder für die Zeitspeicherung
int GvStd, GvMin, GvSek, GvwTag, GvTag, GvMonat, GvJahr;
int Gv5StdAnz, Gv1StdAnz, GvStartMin, GvStartStd;
int Gv5MinAnz, Gv1MinAnz;
boolean GvChangeTimeSw = false;
// Zuordnen der Arduino Pins für MX7219
#define DInPin 12
#define ClkPin 11
#define LodPin 10
//Adresse des DS3231 setzen
#define DS3231_I2C_ADDRESS 0x68
//Erzeugen des Led Objektes mit Namen gr_lc
LedControl gr_lc=LedControl(DInPin,ClkPin,LodPin,1);
/*---------------------------------------------------------------------*/
void setup() {
// Aktionen für den DS3231
// DS3231 Startzeit setzen eine fiktive Zeit irgendwo muss
// man anfangen. Das Datum spielt keine Rolle im Sketch 
// Sekunden, Minuten, Stunden, Wochentag, 
// Tag, Monat, Jahr (spielen keine Rolle
  Wire.begin();
  GvStartMin = 00;

// Aktionen für den MAX7219
// Powersave Mode Abschalten
  gr_lc.shutdown(0,false);
// Mittlere Helligkeit einstellen (0 - 15)
  gr_lc.setIntensity(0,12);
// Alles abschalten
  gr_lc.clearDisplay(0);

//Einstellungen für die Taster zum Stellen der Uhrzeit
pinMode(2, INPUT);
pinMode(3, INPUT);
// Einschalten der internen Pullups
digitalWrite(2, HIGH);
digitalWrite(3, HIGH);
//Interrupts Registrieren
attachInterrupt(0, StundenHoch, LOW);
attachInterrupt(1, MinutenHoch, LOW);

}

void loop() {
// Zeitwerte lesen
    ZeitLesen();
// Bei ungerade Sekunden die Sekundenleuchte Aus bei geraden einschalten
  if (GvSek%2 == 0) {
    gr_lc.setLed(0,2,5,true);
  } else {
    gr_lc.setLed(0,2,5,false);
  }
// Eine neue Anzeige nur wenn die Minuten wechseln
// oder die Stunden wechseln (passiert beim Stellen der Uhr)
  if ((GvMin != GvStartMin) || 
      (GvStd != GvStartStd)) {
    GvStartMin = GvMin;
    GvStartStd = GvStd;
// Alles abschalten
    gr_lc.clearDisplay(0);
// Und die richtigen LED einschalten
    setLEDStd(); //Einschalten der StundenLED (Zeilen 4 und 5)
    setLEDMin(); //Einschalten der MinutenLED (Zeile 1 bis 3)
  }
// Ein klein wenig bremsen ich muss nicht zu oft lesen
  delay(400);
}

/*
 H I L F S F U N K T I O N E N
*/
/*
Die Stundenanzeige belegt die Zeilen 4 und 5 von unten nach oben
Zeile 4 belegt die Einzelstunden 1 - 4
Zeile 5 belegt die 5-Stunden 5 - 20
*/
void setLEDStd() {
  int idx;
  
  Gv5StdAnz = GvStd / 5; 
  Gv1StdAnz = GvStd % 5;
  for (idx = 3; idx < (Gv1StdAnz + 3); idx++) {
    gr_lc.setLed(0,idx,4,true);
  }
  for (idx = 3; idx < (Gv5StdAnz + 3); idx++) {
    gr_lc.setLed(0,idx,5,true);
  }
}

/*
Die Minutenanzeige belegt die Zeilen 1, 2 und 3 von unten nach oben
Zeile 1 hat die Punkte 1,1 - 4,1 mit den Einzelminuten
Zeile 2 hat die Punkte 0,2 - 6,2 mit den 5-Minuten 5 - 35
Zeile 3 hat die Punkte 1,3 - 4,3 mit den 5-Minuten 40 - 55 
*/
void setLEDMin() {
  int idx;
  int Lv5MinAnzA, Lv5MinAnzB;
    
  Gv5MinAnz = GvMin / 5;
// Aufteilen auf die beiden getrennten Zeilen 
  if (Gv5MinAnz > 7) {
    Lv5MinAnzA = 7;
    Lv5MinAnzB = Gv5MinAnz - 7;
  } else {
    Lv5MinAnzA = Gv5MinAnz;
    Lv5MinAnzB = 0;
  }
  Gv1MinAnz = GvMin % 5;
// Setzen der Zeile 1
  for (idx = 3; idx < (Gv1MinAnz + 3); idx++) {
    gr_lc.setLed(0,idx,1,true);
  }
// Setzen der Zeile 3 das ist der linke Teil der 5Minuten LED
  for (idx = 0; idx < Lv5MinAnzA; idx++) {
    gr_lc.setLed(0,idx,3,true);
  }

// Setzen der Zeile 2 das ist der rechte Teil der 5Minuten LED
  for (idx = 3; idx < (Lv5MinAnzB + 3); idx++) {
    gr_lc.setLed(0,idx,2,true);
  }
}

// Zeit lesen und in Integervariablen umsetzen
void ZeitLesen() {
  byte second, minute, hour, dayOfWeek, 
       dayOfMonth, month, year;
// Sollte ein Interrupt ausgelesen worden sein, dann erst
// die Zeit auf der RTC korrigieren und dann lesen
  if (GvChangeTimeSw) {
    setDS3231time(GvSek,
                  GvMin,
                  GvStd,
                  GvwTag,
                  GvTag,
                  GvMonat,
                  GvJahr);
    GvChangeTimeSw = false;
  }

  readDS3231time(&second, &minute, &hour, &dayOfWeek, 
                 &dayOfMonth, &month,&year);

  GvSek   = second;
  GvMin   = minute;
  GvStd   = hour;
  GvwTag  = dayOfWeek;
  GvTag   = dayOfMonth;
  GvMonat = month;
  GvJahr  = year;
}

// Hilfsfunktionen f. DS3231
// Datum Uhrzeit setzen
void setDS3231time(byte second, byte minute, byte hour, byte dayOfWeek, byte
dayOfMonth, byte month, byte year)
{
// sets time and date data to DS3231
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set next input to start at the seconds register
  Wire.write(decToBcd(second)); // set seconds
  Wire.write(decToBcd(minute)); // set minutes
  Wire.write(decToBcd(hour)); // set hours
  Wire.write(decToBcd(dayOfWeek)); // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(dayOfMonth)); // set date (1 to 31)
  Wire.write(decToBcd(month)); // set month
  Wire.write(decToBcd(year)); // set year (0 to 99)
  Wire.endTransmission();
}
void readDS3231time(byte *second,
                    byte *minute,
                    byte *hour,
                    byte *dayOfWeek,
                    byte *dayOfMonth,
                    byte *month,
                    byte *year)
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
}

//Auffangroutine für den Interrupt 0 Stunden
void StundenHoch() {
  if ((millis() - alteZeit) > entprell) {
    GvStd++;
    if (GvStd > 23) {
      GvStd = 0;
    }
    alteZeit = millis();
    GvChangeTimeSw = true;
  }
}

//Auffangroutine für den Interrupt 1 Minuten
void MinutenHoch() {
  if ((millis() - alteZeit) > entprell) {
    GvMin++;
    if (GvMin > 59) {
      GvMin = 0;
    }
    alteZeit = millis();
    GvChangeTimeSw = true;
  }
}


// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}

