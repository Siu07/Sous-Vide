#include <PID_AutoTune_v0.h>
#include <LiquidCrystal.h>       //LCD 20x4
#include <PID_v1.h>              //Where the magic happens
#include <math.h>                //mapping numbers between differing scales
#include <DallasTemperature.h>   //Small temperature sensing module
#include <OneWire.h>
#include <Time.h>                //Counting from boot time
#include <TimeAlarms.h>          //Basic multithreading
#include <Bounce2.h>             //Deboounce inputs

enum PinAssignments {
  buttonPin = 2,    // Select
  backButton = 3    // back
};

#define buttonPin 2
#define backButton 3

Bounce debouncer1 = Bounce(); 
Bounce debouncer2 = Bounce(); 
//Display
LiquidCrystal lcd(12, 11, 7, 6, 5, 4);

//PID
int Temp;
double TBand = 8.17;  //(+-2c) Temperature's +- this value from the target will switch control over to bang bang control
double tempTemp = 20;
int hours = 0, mins =0, secs = 0;
long t = 0;

//Atune stuff
byte ATuneModeRemember=2;
boolean tuning = false;
double aTuneStep=750, aTuneNoise=1, aTuneStartValue=750; //KP Set aTuneStep to ensure noticeable heating and cooling from aTuneStartValue
unsigned int aTuneLookBack=200;

//Arduino hardware IO
int hePin = 14;     //Define output pins

long runTimer = 0;    //milli's since program start

//heater                                      Define default PID parameters
int windowSize = 10000;
unsigned long windowStartTime;
double heInput, heOutput, heSetpoint;
double heKp = 1, heKi = 50, heKd = 10;

unsigned long serialTime; //this will help us know when to talk with processing

PID heater(&heInput, &heOutput, &heSetpoint, heKp, heKi, heKd, DIRECT); 
PID_ATune aTune(&heInput, &heOutput);

#define ONE_WIRE_BUS 8
OneWire ourWire(ONE_WIRE_BUS);
DallasTemperature sensors(&ourWire);

void printscr() {
    time_t t = now()- runTimer;
    //Print run time information every seconds whilst calculating PID
    lcd.setCursor(0, 0);
    lcd.print(F("Temp:"));
    lcd.print(doubleMap(heInput, 0, 1023, -45.2142, 80),2);
    lcd.setCursor(0, 1);
    lcd.print(F("Target: "));
    lcd.print(doubleMap(heSetpoint, 0, 1023, -45.2142, 80),2);
    lcd.setCursor(0, 2);
    lcd.print(F("Window: "));
    lcd.print((heOutput/windowSize)*100);
    lcd.print(F("%"));
    lcd.setCursor(0, 3);
    lcd.print(F("Time:"));
    lcd.setCursor(6, 3);
    lcd.print(F("H:"));
    lcd.print(hour(t));
    lcd.setCursor(11, 3);
    lcd.print(F("M:"));
    lcd.print(minute(t));
    lcd.setCursor(16, 3);
    lcd.print(F("S:"));
    lcd.print(second(t));
    Alarm.timerOnce(1, printscr);  //display every second when a program is running rather than make everything else pause
}
  
double doubleMap(double in, double A, double B, double C, double D) {
  double out;
  out = (in-A)/(B-A)*(D-C)+C;
  return out;
}  

void getTemp(){
 //returns the temperature from one DS18S20 in DEG Celsius
 sensors.requestTemperatures();
 heInput = doubleMap(sensors.getTempCByIndex(0), -45.2142, 80, 0, 1023);
 process();
 Alarm.timerOnce(1, getTemp);
}

void process() {
  if(heInput >= heSetpoint + TBand){  //if outside of close control, use bang bang control
      heOutput = 0;
  }
  else if(heInput <= heSetpoint - TBand){
      heOutput = windowSize;
  }
  else{
    heater.Compute();
  }
  //Serial.println(heOutput); 
}

void setup() {
  Serial.begin(9600);
  sensors.begin();
  lcd.begin(20, 4);
  //Timing                                                                                       
  setTime(0,0,0,1,1,16); // set time to Saturday 0:00:00am Jan 1 2013   // setTime to be synced to RTC
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(backButton, INPUT_PULLUP);
  pinMode(14, OUTPUT);    //Heater output is digital, not PWM
  digitalWrite(buttonPin, HIGH);
  digitalWrite(backButton, HIGH);
   // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doButtonPin, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doBackButton, CHANGE);
  debouncer1.attach(buttonPin);
  debouncer1.interval(15); // interval in ms
  debouncer2.attach(backButton);
  debouncer2.interval(15); // interval in ms

  //PID
  windowStartTime = millis();
  heater.SetOutputLimits(0, windowSize);
  heater.SetMode(AUTOMATIC); //Should be AUTOMATIC, only manual during testing
  //output RH and Temp values to PID and initilise
  lcd.clear();
  heSetpoint = 1063.85;//892.28;//859.6;//696.2;//
  heater.SetSampleTime(1000);  //PID time between calculations in ms
  Alarm.timerOnce(1, getTemp);
  delay(10);
  Alarm.timerOnce(1, printscr);
}

void doButtonPin(){
  debouncer1.update();
  if (debouncer1.read() == HIGH){
    debouncer1.update();
    heSetpoint = heSetpoint + 2.0425;
    lcd.setCursor(0, 1);
    lcd.print(F("Target: "));
    lcd.print(doubleMap(heSetpoint, 0, 1023, -45.2142, 80),2);
  } 
}

void doBackButton(){
  debouncer2.update();
  if (debouncer2.read() == HIGH){
    debouncer2.update();
    heSetpoint = heSetpoint - 2.0425;
    lcd.setCursor(0, 1);
    lcd.print(F("Target: "));
    lcd.print(doubleMap(heSetpoint, 0, 1023, -45.2142, 80),2);
  }
}

void loop() {
  Alarm.delay(10); // check if timer has expired
//  if (digitalRead(buttonPin) == LOW){
//    
//  }
//  else if (digitalRead(backButton) == LOW){
//    
//  }
  unsigned long now = millis();
  if(now - windowStartTime>windowSize){ //time to shift the Relay Window
    windowStartTime += windowSize;
  }
  if(heOutput > now - windowStartTime){
    digitalWrite(hePin,HIGH); // 
  }
  else{
    digitalWrite(hePin,LOW); // 
  }  
  debouncer1.update();
  debouncer2.update();

  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
}

/********************************************
 * Serial Communication functions / helpers
 ********************************************/


union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array



// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2-5: float setpoint
//  6-9: float input
//  10-13: float output  
//  14-17: float P_Param
//  18-21: float I_Param
//  22-245: float D_Param
void SerialReceive()
{

  // read the bytes sent from Processing
  int index=0;
  byte Auto_Man = -1;
  //byte Direct_Reverse = -1;
  byte Tuning_Mode = -1; //KP Tuning Mode?
  while(Serial.available()&&index<26)
  {
    if(index==0) Auto_Man = Serial.read();
    //else if(index==1) Direct_Reverse = Serial.read();
    else if(index==1) Tuning_Mode = Serial.read(); //KP Tuning Mode?
    else foo.asBytes[index-2] = Serial.read();
    index++;
  } 
  
  // if the information we got was in the correct format, 
  // read it into the system
  if(index==26  && (Auto_Man==0 || Auto_Man==1)&& (Tuning_Mode==0 || Tuning_Mode==1))
  {
    heSetpoint=doubleMap(double(foo.asFloat[0]), -45.2142, 80, 0, 1023);
    //Input=double(foo.asFloat[1]);       // * the user has the ability to send the 
                                          //   value of "Input"  in most cases (as 
                                          //   in this one) this is not needed.
    if(Auto_Man==0)                       // * only change the output if we are in 
    {                                     //   manual mode.  otherwise we'll get an
      heOutput=doubleMap(double(foo.asFloat[2]), 0, 100, 0, windowSize);      //   output blip, then the controller will 
    }                                     //   overwrite.
    
    double p, i, d;                       // * read in and set the controller tunings
    p = double(foo.asFloat[3]);           //
    i = double(foo.asFloat[4]);           //
    d = double(foo.asFloat[5]);           //
    heater.SetTunings(p, i, d);            //
    
    if(Auto_Man==0) heater.SetMode(MANUAL);// * set the controller mode
    else heater.SetMode(AUTOMATIC);             //
    
//    if(Direct_Reverse==0) heater.SetControllerDirection(DIRECT);// * set the controller Direction
//    else heater.SetControllerDirection(REVERSE);          //
    if(Tuning_Mode == 0) tuning=false; // Set Tuning mode on/off
    else tuning=true;
  }
  Serial.flush();                         // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?

void changeAutoTune()
{
 if(!tuning)
  {
    //Set the Output to the desired starting frequency.
    aTuneStartValue = heOutput; //KP Initial aTuneStartValue will be = Output at Toggle
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = heater.GetMode();
  else
    heater.SetMode(ATuneModeRemember);
}

void SerialSend()
{
  Serial.print("PID ");
  Serial.print(doubleMap(heSetpoint, 0, 1023, -45.2142, 80));   
  Serial.print(" ");
  Serial.print(doubleMap(heInput, 0, 1023, -45.2142, 80));   
  Serial.print(" ");
  Serial.print(doubleMap(heOutput, 0, 10000, 0, 100));   
  Serial.print(" ");
  Serial.print(heater.GetKp());   
  Serial.print(" ");
  Serial.print(heater.GetKi());   
  Serial.print(" ");
  Serial.print(heater.GetKd());   
  Serial.print(" ");
  if(heater.GetMode()==AUTOMATIC) Serial.print("Automatic");
  else Serial.print("Manual");  
  Serial.print(" ");
//  if(heater.GetDirection()==DIRECT) Serial.println("Direct");
//  else Serial.println("Reverse");
  if(tuning==false) Serial.println("Off"); //KP Added the On/Off for Tuning Toggle
  else Serial.println("On");
}
