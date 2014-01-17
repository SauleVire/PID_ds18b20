/********************************************************
 * PID RelayOutput Example
 * Same as basic example, except that this time, the output
 * is going to a digital pin which (we presume) is controlling
 * a relay.  The pid is designed to output an analog value,
 * but the relay can only be On/Off.
 *
 *   To connect them together we use "time proportioning
 * control"  Tt's essentially a really slow version of PWM.
 * First we decide on a window size (5000mS say.) We then 
 * set the pid to adjust its output between 0 and that window
 * size.  Lastly, we add some logic that translates the PID
 * output into "Relay On Time" with the remainder of the 
 * window being "Relay Off Time"
 ********************************************************/

#include <LiquidCrystal.h>  //this library is modified by Lady Ada to support I2C


#include <PID_v1.h>
#define RelayPin 4


//Define Variables we'll be connecting to
double Setpoint, Input, Output;
float Kp = 0.0; 
float Ki = 0.0;
float Kd = 0.0;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,0.5,100,4, DIRECT);

int WindowSize = 5000;
unsigned long windowStartTime;

  int thermoDO = 11;              // MAX31855 Data
  int thermoCS = 10;              // MAX31855 Chip Select
  int thermoCLK = 9;              // MAX31855 Clock
  
  double readC = 0.00;
  double readF = 0.00;

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
/* ************************************************************** */
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
/* ************************************************************** */

void setup()
{
  Serial.begin(9600);
  windowStartTime = millis();
  lcd.begin(16,2);              // initialize lcd
  lcd.clear();                      // clear lcd

  //initialize the variables we're linked to
  Setpoint = 220.00;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(-WindowSize, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
  sensors.begin();  
}

void loop()
{
  sensors.requestTemperatures(); // Send the command to get temperatures

  runPID();



  

   readC = sensors.getTempCByIndex(0);

 
     lcd.setCursor(0,0);
     //lcd.print("IT:");
     lcd.print("S:");
     lcd.setCursor(2,0);
    // lcd.print(thermocouple.readInternal());    // lcd print Internal Temp
     lcd.print(Setpoint);
     lcd.setCursor(9,0);
     lcd.print("F:");
     lcd.setCursor(11,0);
     lcd.print(readC);    // lcd print Celsius Temp


     Serial.print("C= ");
     Serial.print(readC);      // serial print Celsius Temp
   
}

void runPID()  {
  Input = readC;
  Serial.print("PID Input =");
  Serial.println(Input);
  myPID.Compute();

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  unsigned long now = millis();
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if(Output > now - windowStartTime) digitalWrite(RelayPin,HIGH);
  else digitalWrite(RelayPin,LOW);
}


