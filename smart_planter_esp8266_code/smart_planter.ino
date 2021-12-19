#include "ThingSpeak.h"
#include <ESP8266WiFi.h>
#include <SFE_BMP180.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include<Wire.h>
#include <LiquidCrystal_I2C.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// GPIO where the DS18B20 is connected to
const int oneWireBus = 13;     

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

const long utcOffsetInSeconds = 6 * 60 * 60;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int sensor_pin = A0;

// You will need to create an SFE_BMP180 object, here called "pressure":
 
SFE_BMP180 pressure;

#define ALTITUDE 4.0 // Altitude of SparkFun's HQ in Boulder, CO. in meters
WiFiClient  client;
double* tpa;
unsigned long myChannelNumber = 1499063; // Channel ID here
const char * myWriteAPIKey = "9BICFD4YE48T6XL8"; // Your Write API Key here


void setup()
{
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  // Print a message to the LCD.
  Serial.begin(115200);
  WiFiManager wifiManager;
  Serial.println("Conecting.....");
  wifiManager.autoConnect("SmartPlanter");
  Serial.println("connected");
  
  ThingSpeak.begin(client);
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.
 
    Serial.println("BMP180 init fail\n\n");
    while(1); // Pause forever.
  }
  sensors.begin();

  pinMode(14, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(14,LOW);
  digitalWrite(12,LOW);

  
  timeClient.begin();
  timeClient.setTimeOffset(6*60*60);
}

void loop()
{
  
  Wire.setClock(10000);
  digitalWrite(12,HIGH);
  digitalWrite(14,LOW);
  delay(1000);
  int rawData = analogRead(A0);
  
//  int sensorValue = digitalRead(D4);   // read the input on analog pin 0

  float lum = rawData;   // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V)
  Serial.print("Luminosity: ");
  Serial.print(lum, 0);
  Serial.println();
  
  char status;
  double T,P,p0,a;

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);
 
    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.
 
    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      Serial.print("temperature: ");
      Serial.print(T,2);
      Serial.print(" deg C, ");
//      Serial.print((9.0/5.0)*T+32.0,2);
//      Serial.println(" deg F");
      
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.
 
      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);
 
        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.
 
        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          // Print out the measurement:
          Serial.print("absolute pressure: ");
          Serial.print(P,2);
          Serial.print(" mb, ");
          Serial.print(P*0.0295333727,2);
          Serial.println(" inHg");
 
          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb
 
          p0 = pressure.sealevel(P,ALTITUDE); // we're at 1655 meters (Boulder, CO)
          Serial.print("relative (sea-level) pressure: ");
          Serial.print(p0,2);
          Serial.print(" mb, ");
//          Serial.print(p0*0.0295333727,2);
//          Serial.println(" inHg");
 
          // On the other hand, if you want to determine your altitude from the pressure reading,
          // use the altitude function along with a baseline pressure (sea-level or other).
          // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
          // Result: a = altitude in m.
 
          a = pressure.altitude(P,p0);
          Serial.print("computed altitude: ");
          Serial.print(a,0);
          Serial.print(" meters, ");
//          Serial.print(a*3.28084,0);
//          Serial.println(" feet");
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
  float moisture_percentage;
  digitalWrite(14,HIGH);
  digitalWrite(12,LOW);
  
  delay(1000);
  moisture_percentage = ( 100.00 - ( (analogRead(sensor_pin)/1023.00) * 100.00 ) );

  Serial.print("Soil Moisture(in Percentage) = ");
  Serial.print(moisture_percentage);
  Serial.println("%");

  
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);
  Serial.print("Soil Temperature = ");
  Serial.print(temperatureC);
  Serial.println(" deg C");
  
  ThingSpeak.setField( 1, (float)T );
  ThingSpeak.setField( 2, (float)lum );
  ThingSpeak.setField( 3, (float)p0 );
  ThingSpeak.setField( 4, (float)temperatureC );
  ThingSpeak.setField( 5, (float)moisture_percentage );
  ThingSpeak.writeFields( myChannelNumber, myWriteAPIKey );
  timeClient.update();
  

  int currentHour = timeClient.getHours();
  Serial.print("Hour: ");
  Serial.println(currentHour);  
 
  int currentMinute = timeClient.getMinutes();
  Serial.print("Minutes: ");
  Serial.println(currentMinute); 

  String period="";
  if(currentHour==12){
    period="pm";
  }else if(currentHour>12){
    currentHour-=12;
    period="pm";
  }else{  
    period="am";
  }

  String current_time = String(currentHour)+"."+String(currentMinute)+" "+period;
  lcd.setCursor(0, 0);
  lcd.print("Time:   ");
  lcd.print(current_time);
    
  lcd.setCursor(0, 1);
  lcd.print("Temp:   ");
  lcd.print((int)T);
  lcd.print((char)223);
  lcd.print("c");

  delay(6000);
  lcd.clear();
  
  lcd.setCursor(0, 0);

  lcd.print("Soil temp:");
  lcd.print((int)temperatureC);
  lcd.print((char)223);
  lcd.print("c");
  
  lcd.setCursor(0, 1);

  // print the number of seconds since reset:
  lcd.print("Light:    ");
  lcd.print((int)lum);
  lcd.print(" lx");
  lcd.setCursor(0, 1);
  
  delay(6000);
  lcd.clear();  
  
  lcd.setCursor(0, 0);
  // print the number of seconds since reset:
  lcd.print("Moisture:");
  lcd.print((int)moisture_percentage);
  lcd.print("%");
  
  lcd.setCursor(0, 1);
 
  // print the number of seconds since reset:
  lcd.print("Pressure:");
  lcd.print((int)p0);
  lcd.print("bar");
  lcd.setCursor(0, 1);
  
    
    delay(6000);
    lcd.clear();
}
