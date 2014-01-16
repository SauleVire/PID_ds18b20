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



void setup()
{
  Serial.begin(9600);
  windowStartTime = millis();
  lcd.begin(16,2);              // initialize lcd
  lcd.clear();                      // clear lcd

  //initialize the variables we're linked to
  Setpoint = 220.00;

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  readTemps();          // run readTemps
  runPID();
}

double readTemps()  {
  

   readC = thermocouple.readCelsius();    // read thermocouple in Celsius
   readF = thermocouple.readFarenheit();
   
   
   if (isnan(readC)) {                      // if readC is not a number
     lcd.setCursor(0,1);
     lcd.println("TC ERR!");            // thermocouple error!!
     Serial.print("THERMOCOUPLE ERROR!!!");
   } else {                            //  otherwise
     lcd.setCursor(0,0);
     //lcd.print("IT:");
     lcd.print("S:");
     lcd.setCursor(2,0);
    // lcd.print(thermocouple.readInternal());    // lcd print Internal Temp
     lcd.print(Setpoint);
     lcd.setCursor(9,0);
     lcd.print("F:");
     lcd.setCursor(11,0);
     lcd.print(thermocouple.readFarenheit());    // lcd print Celsius Temp
     Serial.print("IT= ");
     Serial.println(thermocouple.readInternal());    // serial print Internal Temp
     Serial.print("C= ");
     Serial.print(thermocouple.readCelsius());      // serial print Celsius Temp
     Serial.print("  F= ");
     Serial.println(thermocouple.readFarenheit());    // serial print Farenheit temp
     Serial.print("Error ");
     Serial.println(thermocouple.readError());        // serial print Read Error Bits
     
     
   }
   
}

void runPID()  {
  // Input = analogRead(0);
  double readTempF = thermocouple.readFarenheit();
  Input = readTempF;
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


