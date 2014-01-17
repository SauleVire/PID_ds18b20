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
#define RelayPinOpen 4
#define RelayPinClose 5

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

// Specify the links and initial tuning parameters
// buvo taip:
// PID myPID(&Input, &Output, &Setpoint,0.5,100,4, DIRECT);

PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

int WindowSize = 5000;
unsigned long windowStartTime;


LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
/* ************************************************************** */
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 36

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
  Setpoint = 25.00;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(-WindowSize, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  
  sensors.begin();
sensors.requestTemperatures(); 
Input = sensors.getTempCByIndex(0);
}

void loop()
{
  sensors.requestTemperatures(); // Send the command to get temperatures
Input = sensors.getTempCByIndex(0);

  double gap = abs(Setpoint-Input); //distance away from setpoint
  if(gap<10)
  {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }

  runPID();

  
 
     lcd.setCursor(0,0);
     lcd.print("Set:");
     lcd.print(Setpoint);
     lcd.setCursor(9,0);
     lcd.print("C:");
     lcd.print(Input);    // lcd print Celsius Temp


     Serial.print("C= ");
     Serial.print(Input);      // serial print Celsius Temp
   
}

void runPID()  {

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
if(Output==0)

{ //pid thinks we shouldn't activate either relay. do nothing
digitalWrite(RelayPinOpen,HIGH);
digitalWrite(RelayPinClose,HIGH);
}
else if(Output>0)

{ // pid thinks we should be activating relay1 for a time proportional 
  // to the value of output
if(Output > millis() - windowStartTime) digitalWrite(RelayPinOpen,LOW);

else digitalWrite(RelayPinOpen,HIGH);
digitalWrite(RelayPinClose,LOW);
}
else // pid thinks we should be activating relay2 for a time proportional 
     // to the value of -1 * output
{
if(-Output > millis() - windowStartTime) digitalWrite(RelayPinClose,LOW);
else digitalWrite(RelayPinClose,HIGH);
digitalWrite(RelayPinOpen,LOW);
}
}


