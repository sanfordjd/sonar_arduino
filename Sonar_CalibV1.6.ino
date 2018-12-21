/* 
   Sonar Ultrasonic Long Range calibration tool.

Assumes Adafruit-RGB-LCD-Shield-Library

Do a Ultrasonic Sonar measurement and print it to the display.
Continuosly measure the distance and compare if the measured distance stays within a tolerance.
After a defined number of measurements with all distances within tolerance the sonar is stated ok.
 The circuit is described in a schematic document.
 The display pulls In/Out 10 to a voltage level. Out 10 must be separated from the display shield,
 otherwise the sonar measurement on Input 10 doesent work and will allways return 0.


Joe Sanford, Diversey Richmond
January 2018

     June 2018    V. 1.6: Changed speed of sound from 340m/s to 343m/s as this is what is assumed by the GenX+ Prox Code - JDS
                           Added in different LCD Screen functionality back in - JDS
                           Changed GPIO Ports to fit the default ports used in test fixture - JDS
     June 2018    V. 1.42: Ringing time must be below 2100 ms. Decreased margin. Several sonars calibrated show, that there would be massive amount of bad sonars.
                           In production the sonars with 2100 ms acceptance worked fine.
     April 2018   V. 1.41: Check long range sonar ringing time within Proximity board blanking limit of 2.2ms - ringing time must be below 2000ms for margin,
                           Change speed of sound from 333 to 340 m/s - MG 
                           Change pulse Duration from 400us to 300us and 70us to have similar behaviour as procimity board
    January 2018  V. 1.5 : Troubleshooting V2, corrected speed of sound 58kHz Sonar - JDS
    January 2018  V. 1.4 : Added in different LCD Screen - JDS
    December 2017 V. 1.32: Sonar stated ok if 80 measurements of a distance are within 0.5% tolerance
    December 2017 V. 1.30: Calibration of three different sonar types with different characteristics 
 
 
 */

#include <SoftwareSerial.h>   //Software Serial Port
//#include <LiquidCrystal.h>
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

//LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

const char versionText[17] = "LongRangeV. 1.6 ";

const int speedOfSound = 343;     // the speed of sound is 340 meters per second; Distance to my reference is 9.95 m to wall. With 340m/s I get 9.95 m

const int blankingTimeLongRange = 2100; // the blanking time in Prximity board is defined with 2200 microseconds. The measured ringing time must be smaller than this, otherwise the Prox will detect a stuck sonar.

const int sonar3TxPin = 25;      // the number of the Sonar3 send pin    -- Long Range
const int sonar3RxPin = 23;      // the number of the Sonar3 receive pin     -- Long Range
const int sonar2TxPin = 53;// 37;      // the number of the Sonar2 send pin    -- Kickplate
const int sonar2RxPin = 51;// 39;      // the number of the Sonar2 receive pin    -- Kickplate
const int sonar1TxPin =  23;        // the number of the Sonar1 send pin    -- 58kHz Autostar
const int sonar1RxPin =  25;        // the number of the Sonar1 receive pin    -- 58kHz Autostar

const int numberOfMeasurements = 80;
const int pulseDurationLong = 300;
//const int pulseDurationStd = 125; //pulse duration for some of the 58kHz sonars
const int pulseDurationStd  = 300;
const int pulseDurationKick = 70;
long lastDistance = 0;
long distance = 0;
long gringingMicros = 0;
int  progress = 0;
int  sonarTestNumber = 0;

long measureSonarMicros(int sonarOutputPin, int sonarInputPin, int pulseDuration, int echoNumber, int maxMillis, long& ringingMicros)
{
  // set the Output High for 0.4ms to actiate the sonar:
  digitalWrite(sonarOutputPin, HIGH);
  delayMicroseconds(pulseDuration);

  // set the Output Low to wait for echoes:
  digitalWrite(sonarOutputPin, LOW);

  long starttime = micros();  // Start time measurement after the start pulse.

  ringingMicros = 0;
  long duration = 0;
  long echoDuration = 0;
  int  counter = 0;
  int  lastSonarState = LOW;
  int  actualSonarState = LOW;
  do
  {
    actualSonarState = digitalRead(sonarInputPin);
    duration = micros()-starttime;
    // A sonar reading exists if the sonar pulls the signal LOW.
    // Depending on the sonar type the first time going to zero is not a valid reading and must be ignored.
    if (actualSonarState == LOW && actualSonarState != lastSonarState)
    {
      lastSonarState = actualSonarState;
      counter++;
    }
    else if (actualSonarState == HIGH && counter<echoNumber && actualSonarState != lastSonarState)
    {
      // This is the first time the sonar goes from Low to High, which means the ringing (firing of the sound) is over. The proximity board waits for a defined blanking time.
      // If the ringing takes longer than the blanking time the Proximity board will detect a stuck reading. Here we measure the duration of the ringing time.
      lastSonarState = actualSonarState;
      ringingMicros = duration + pulseDuration; // the pulseDuration microseconds must be added because the blanking time is defined in the Prox from the beginning of the Firing pulse.
    }
    else if (actualSonarState != lastSonarState)
    {
      lastSonarState = actualSonarState;
    }
    if (counter >= echoNumber && 0 == echoDuration)
    {
      echoDuration = duration;
    }
  }
  while (duration < ((long) maxMillis)*1000);
  return echoDuration;
}

long microsToMillis(long microsec)
{
  return microsec*speedOfSound/2000;
}

void showProgress()
{
  if (progress > 0)
  {
    lcd.setCursor(6, 1);
    lcd.setBacklight(RED);
    lcd.print("-");
  }
  if (progress > numberOfMeasurements/8)
  {
    lcd.setCursor(7, 1);
    lcd.setBacklight(RED);
    lcd.print("-");
  }
  if (progress > numberOfMeasurements/8*2)
  {
    lcd.setCursor(8, 1);
    lcd.setBacklight(RED);
    lcd.print("-");
  }
  if (progress > numberOfMeasurements/8*3)
  {
    lcd.setCursor(9, 1);
    lcd.setBacklight(RED);
    lcd.print("-");
  }
  if (progress > numberOfMeasurements/8*4)
  {
    lcd.setCursor(10, 1);
    lcd.setBacklight(RED);
    lcd.print("-");
  }
  if (progress > numberOfMeasurements/8*5)
  {
    lcd.setCursor(11, 1);
    lcd.setBacklight(RED);
    lcd.print("-");
  }
  if (progress > numberOfMeasurements/8*6)
  {
    lcd.setCursor(12, 1);
    lcd.setBacklight(RED);
    lcd.print("-");
  }
  if (progress > numberOfMeasurements/8*7)
  {
    lcd.setCursor(13, 1);
    lcd.setBacklight(GREEN);
    lcd.print(">OK");
    Serial.println(">OK");
    delay(500);
  }
}

void setup() {
  // set the digital pin as output:
  pinMode(sonar3TxPin, OUTPUT);
  pinMode(sonar2TxPin, OUTPUT);
  pinMode(sonar1TxPin, OUTPUT);
  pinMode(sonar3RxPin, INPUT);
  pinMode(sonar2RxPin, INPUT);
  pinMode(sonar1RxPin, INPUT);

  Serial.begin(57600);

  lcd.begin(16,2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.println(versionText);
  Serial.println(versionText);
  Serial.println("Sonar Calibration");
  lcd.setCursor(0, 1);
  lcd.println("Distances in mm ");
  delay(2000);
}

/* example of display:
 *  "Long   Ring:1753"
 *  "9963  ------->OK"
 */

void loop()
{
  // Long range sonar can measure every 60 milliseconds, valid distance on first going low; min: 350mm, max: 9999mm
  distance = microsToMillis(measureSonarMicros(sonar3TxPin, sonar3RxPin, pulseDurationLong, 1, 80, gringingMicros));
  if (distance > 0)
  {
    sonarTestNumber = 3;
    lcd.setCursor(0, 0);
    lcd.print("Long   Ring:    ");
  }
  else
  {
    // Kickplate sonar can measure every 30 milliseconds, valid distance on first going low; min: 100mm, max: 2000mm
    distance = microsToMillis(measureSonarMicros(sonar2TxPin, sonar2RxPin, pulseDurationKick, 1, 30, gringingMicros));
    if (distance > 0)
    {
      sonarTestNumber = 2;
      lcd.setCursor(0, 0);
      lcd.print("Kick   Ring:    ");
    }
    else
    {
      // Standard sonar can measure every 30 milliseconds, valid distance on second going low; min: 300mm, max: 2500mm
      distance = microsToMillis(measureSonarMicros(sonar1TxPin, sonar1RxPin, pulseDurationStd, 2, 30, gringingMicros));
      if (distance > 0)
      {
        sonarTestNumber = 1;
        lcd.setCursor(0, 0);
        lcd.print("Std    Ring:    ");
      }
      else
      {
        sonarTestNumber = 0;
        lcd.setCursor(0, 0);
        lcd.print("Pl'se con. sonar");
      }
    }
  }

  // now do a continuos testing on this sonar to see if it's within tolerance.
  do
  {
    if (3 == sonarTestNumber)
    {
      // Long range sonar can measure every 60 milliseconds, valid distance on first going low; min: 350mm, max: 9999mm
      distance = microsToMillis(measureSonarMicros(sonar3TxPin, sonar3RxPin, pulseDurationLong, 1, 100, gringingMicros));
      Serial.print("Long: ");
      if(gringingMicros > blankingTimeLongRange)  // The measured ringing time must be smaller than the blanking time, otherwise the Prox will detect a stuck sonar.
      {
        lcd.setCursor(0, 0);
        lcd.setBacklight(RED);
        lcd.print("Err! ");
        lcd.setCursor(12, 0);
        lcd.print(gringingMicros);
        delay(1000);
      }
    }
    else if (2 == sonarTestNumber)
    {
      // Kickplate sonar can measure every 30 milliseconds, valid distance on first going low; min: 100mm, max: 2000mm
      distance = microsToMillis(measureSonarMicros(sonar2TxPin, sonar2RxPin, pulseDurationKick, 1, 30, gringingMicros));
      Serial.print("Kick: ");
    }
    else if (1 == sonarTestNumber)
    {
      // Standard sonar can measure every 30 milliseconds, valid distance on second going low; min: 300mm, max: 2500mm
      distance = microsToMillis(measureSonarMicros(sonar1TxPin, sonar1RxPin, pulseDurationStd, 2, 30, gringingMicros));
      Serial.print("Std:  ");
    }
    else if (0 == sonarTestNumber)
    {
      Serial.print("Pl'se con. sonar: ");
    }

    Serial.print(distance);
    Serial.print(", Ringing time: ");
    Serial.println(gringingMicros);
    
    lcd.setCursor(0, 1);
    lcd.print("     ");       // blank the screen before the new measurement value.
    lcd.setCursor(0, 1);
    lcd.print(distance);

    if (sonarTestNumber > 0)
    {
      char float_str[5];
      dtostrf(gringingMicros,4,0,float_str);
      lcd.setCursor(12, 0);
      lcd.print(float_str);
    }
    
    showProgress();
    progress++;
  }
  // do this while the measured distance stays within 0.5% of the last value.    
  while (    (distance < (lastDistance + lastDistance * 0.005) )
           &&(distance > (lastDistance - lastDistance * 0.005) )
           &&(distance > 0)   );

  lastDistance = distance;
  progress = 0;
  lcd.setCursor(5, 1);
  lcd.print("           ");       // blank the screen before the new measurement value.
}
