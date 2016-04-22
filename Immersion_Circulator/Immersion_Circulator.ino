//-------------------------------------------------------------------
// Sous Vide Controller
// Bill Earl - for Adafruit Industries
// Edited by SIU07CRD
//
// Based on the Arduino PID and PID AutoTune Libraries 
// by Brett Beauregard
//------------------------------------------------------------------

// PID Library
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <LiquidCrystal.h>       //LCD 16x4

// Libraries for the DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// So we can save and retrieve settings
#include <EEPROM.h>

//Menu Libary
#include <MenuBackend.h>         //Menu element. version 1.4 by AlphaBeta

// ************************************************
// Pin definitions
// ************************************************

// Output Relay
#define RelayPin 21

// One-Wire Temperature Sensor
#define ONE_WIRE_BUS 20

// interrupt service routine vars
#define encoder0PinA  2
#define encoder0PinB  3

//// motor connections
#define HG7881_B_IA 9 // D10 --> Motor B Input A --> MOTOR B +
#define HG7881_B_IB 8 // D11 --> Motor B Input B --> MOTOR B -
// 
//// functional connections
#define MOTOR_B_PWM HG7881_B_IA // Motor B PWM Speed high to run
#define MOTOR_B_DIR HG7881_B_IB // Motor B Direction low to run

boolean A_set = false;              
boolean B_set = false;

// Rotary Encoder vars
volatile unsigned int encoder0Pos = 0;
volatile unsigned int encoderValue = 0;  // a counter for the dial
static boolean rotating=false;      // debounce management
unsigned int lastReportedPos = 60;   // change management
volatile unsigned int encoderPos = 60;  // a counter for the dial

enum PinAssignments {
  encoderPinA = 3,   // scroll right
  encoderPinB = 2,   // scroll left
  buttonPin = 5,    // Select
  backButton = 6    // back
};



// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;
double spinRate; //motor pwm

volatile long onTime = 0;

// pid tuning parameters
double Kp;
double Ki;
double Kd;

// EEPROM addresses for persisted data
const int SpAddress = 0;
const int KpAddress = 8;
const int KiAddress = 16;
const int KdAddress = 24;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// 10 second Time Proportional Output window
int WindowSize = 10000; 
unsigned long windowStartTime;

// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember=2;

double aTuneStep=500;
double aTuneNoise=1;
unsigned int aTuneLookBack=20;

boolean tuning = false;

PID_ATune aTune(&Input, &Output);

// ************************************************
// Display Variables and constants
// ************************************************

LiquidCrystal lcd(10, 16, 14, 15, 18, 19); //19, 18, 15, 14);

unsigned long lastInput = 0; // last button press

byte degree[8] = // define the degree symbol 
{ 
 B00110, 
 B01001, 
 B01001, 
 B00110, 
 B00000,
 B00000, 
 B00000, 
 B00000 
}; 

const int logInterval = 10000; // log every 10 seconds
long lastLogTime = 0;

// ************************************************
// States for state machine
// ************************************************
enum operatingState { OFF = 0, RUN, TUNE_P, TUNE_I, TUNE_D, AUTO};
operatingState opState = OFF;

// ************************************************
// Sensor Variables and constants
// Data wire is plugged into port 2 on the Arduino

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempSensor;

//Menu  Elements
void menuChangeEvent(MenuChangeEvent changed);
void menuUseEvent(MenuUseEvent used);
MenuBackend menu = MenuBackend(menuUseEvent,menuChangeEvent);
  MenuItem TurnOff = MenuItem("Off");
    MenuItem DoRun = MenuItem("Run");
    MenuItem DoAutotune = MenuItem("Autotune");
    MenuItem DoTuneP = MenuItem("Tune P");
    MenuItem DoTuneI = MenuItem("Tune I");
    MenuItem DoTuneD = MenuItem("Tune D");
    
// ************************************************
// Setup and diSplay initial screen
// ************************************************
void setup()
{
   Serial.begin(9600);


//   //Initalize motor controller
   pinMode( MOTOR_B_DIR, OUTPUT );
   pinMode( MOTOR_B_PWM, OUTPUT ); 
   digitalWrite( MOTOR_B_DIR, LOW );
   digitalWrite( MOTOR_B_PWM, LOW );
   
   // Initialize Relay Control:
   pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
   digitalWrite(RelayPin, LOW);  // make sure it is off to start
   
  //boolean calculating = false;
  
  //rotary encoder
  pinMode(encoderPinA, INPUT); 
  pinMode(encoderPinB, INPUT); 
  pinMode(buttonPin, INPUT);
  pinMode(backButton, INPUT);
  
 // turn on pullup resistors
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);
  digitalWrite(buttonPin, HIGH);
  digitalWrite(backButton, HIGH);
  
  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);
  
   // Initialize LCD Display 
   lcd.begin(16, 4);
   lcd.createChar(1, degree); // create degree symbol from the binary
   
   //lcd.print(F("    Home"));
   lcd.setCursor(0, 1);
   lcd.print(F("   Sous Vide!"));

   // Start up the DS18B20 One Wire Temperature Sensor
   sensors.begin();
   if (!sensors.getAddress(tempSensor, 0)) 
   {
      lcd.setCursor(0, 1);
      lcd.print(F("Sensor Error"));
   }
   sensors.setResolution(tempSensor, 12); //12 for more accurate model.
   sensors.setWaitForConversion(false);
   LoadParameters();
   myPID.SetTunings(Kp,Ki,Kd);

   myPID.SetSampleTime(1000);
   myPID.SetOutputLimits(0, WindowSize);
  //Setup Menu
   menu.getRoot().add(DoRun);
     DoAutotune.addAfter(DoRun);
     DoTuneP.addAfter(DoAutotune);
     DoTuneI.addAfter(DoTuneP);
     DoTuneD.addAfter(DoTuneI);
     DoRun.addAfter(DoTuneD);
     //DoRun.addBefore(DoTuneD);
         
     DoAutotune.addLeft(DoRun);
     DoTuneD.addLeft(DoRun);
     DoTuneI.addLeft(DoRun);
     DoTuneP.addLeft(DoRun);
     DoRun.addLeft(DoRun);
     
   delay(2000);  // Splash screen

   // Initialize the PID and related variables

//     DoRun.addLeft(TurnOff);

//     TurnOff.addAfter(TurnOff);
//     TurnOff.addBefore(TurnOff);

//     menu.moveDown(); 
    //menu.use(); 
     menu.moveDown();
//     
   }

// ************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************
void loop()
{
  rotating = true;
  //See if any of the inputs have changed
  if (opState != RUN){
    if (lastReportedPos > encoderPos){
      menu.moveDown();                //goto previous menu choice
      lastReportedPos = encoderPos;
    }
    else if (lastReportedPos < encoderPos){
      menu.moveUp();                  //goto next menu choice
      lastReportedPos = encoderPos;
    }
    if (digitalRead(buttonPin) == LOW){
      while (digitalRead(buttonPin) == LOW) delay(10);  //hold code until input is done
      {
        menu.use();
        menu.moveRight();               //Select next menu down or run action
      }
    }
  }
  if (digitalRead(backButton) == LOW){
    while (digitalRead(backButton) == LOW) delay(10);
    Serial.print("backbutton");
    opState = OFF;
    lcd.clear();
    menu.moveLeft();                //go back to previous menu
  }
 
   //lcd.clear();
   switch (opState)
   {
   case OFF:
      Off();
      break;
    case RUN:
      Run();
      break;
   case TUNE_P:
      TuneP();
      break;
   case TUNE_I:
      TuneI();
      break;
   case TUNE_D:
      TuneD();
      break;
   }
   
}
// ************************************************
// Start the Auto-Tuning cycle
// ************************************************

void StartAutoTune()
{
   // Remember the mode we were in
   ATuneModeRemember = myPID.GetMode();

   // set up the auto-tune parameters
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
}
// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
   if (Setpoint != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, Setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }
}
// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}

// ************************************************
// Read Rotary Encoder inputs as interrupts
// ************************************************
void doEncoderA(){
  // debounce
  if ( rotating ) delay (1);  // wait a little until the bouncing is done
  // Test transition, did things really change? 
  if( digitalRead(encoderPinA) != A_set ){   // debounce once more
        A_set = !A_set;
      // adjust counter + if A leads B
      if ( A_set && !B_set ) 
          encoderPos ++;
      rotating = false;  // no more debouncing until loop() hits again
  }
}

// Interrupt on B changing state, same as A above
void doEncoderB(){
  if ( rotating ) delay (1);
  if( digitalRead(encoderPinB) != B_set ) {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if( B_set && !A_set ) 
      encoderPos --;
    rotating = false;
  }
}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   Setpoint = EEPROM_readDouble(SpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);
   
   // Use defaults if EEPROM values are invalid
   if (isnan(Setpoint))
   {
     Setpoint = 60;
   }
   if (isnan(Kp))
   {
     Kp = 850;
   }
   if (isnan(Ki))
   {
     Ki = 0.5;
   }
   if (isnan(Kd))
   {
     Kd = 0.1;
   }  
}

void userInput(int menuFlag, float scale, double prior) { //Flag to interperate where its from, scale of scroll, defualt 1 = one scroll =+1, scale of 0.1 = one scroll =+0.1 etc.
  while (digitalRead(buttonPin) == LOW) delay(10);
  float numInput;
  boolean inputFlag = true;
  encoderPos = prior / scale;
  //calculating = true;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("New Input: "));
  lcd.setCursor(0, 1);
  lcd.print(prior);
  //encoderValue = 1;
  while (inputFlag == true) {
    if (encoderPos < 0 || encoderPos > 1000) encoderPos = 0;
    if (lastReportedPos != encoderPos) {
      lcd.setCursor(0, 1);
      lcd.print(F("                   "));
      lcd.setCursor(0, 1);
      numInput = encoderPos * scale;
      lcd.print(numInput);
      lastReportedPos = encoderPos;
    }
    //Read the user's input
    if (digitalRead(buttonPin) == LOW){
      inputFlag = false;
      lcd.setCursor(2, 0);
      switch (menuFlag) {  //case number, first digit: equipment type, second didgit: P,I or D
      case 1: 
        Setpoint = numInput;
        break;
      case 2: 
        Kp = numInput;
        break;
      case 3: 
        Ki = numInput;
        break;
      case 4: 
        Kd = numInput;
        break;
      default:
        break;
      }
    }
  }
  //calculating = false;
  while (digitalRead(buttonPin) == LOW) delay(10);
  lcd.clear();
}
// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune()
{
   tuning = false;

   // Extract the auto-tune calculated parameters
   Kp = aTune.GetKp();
   Ki = aTune.GetKi();
   Kd = aTune.GetKd();

   // Re-tune the PID and revert to normal control mode
   myPID.SetTunings(Kp,Ki,Kd);
   myPID.SetMode(ATuneModeRemember);
   
   // Persist any changed parameters to EEPROM
   SaveParameters();
}

// ************************************************
// Execute the control loop
// ************************************************
void DoControl()
{
  // Read the input:
  if (sensors.isConversionAvailable(0))
  {
    if (sensors.getAddress(tempSensor, 0)){
      Input = sensors.getTempC(tempSensor);
      while (Input == -127) {
        Input = sensors.getTempC(tempSensor);
      }
      sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
    }
  }
  
  if (tuning) // run the auto-tuner
  {
     if (aTune.Runtime()) // returns 'true' when done
     {
        FinishAutoTune();
     }
  }
  else // Execute control algorithm
  {
     myPID.Compute();
  }
  
  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime = Output; 
}

// ************************************************
// Initial State - press RIGHT to enter setpoint
// ************************************************
void Off()
{
   digitalWrite( MOTOR_B_DIR, LOW );
   digitalWrite( MOTOR_B_PWM, LOW );
   myPID.SetMode(MANUAL);
   digitalWrite(RelayPin, LOW);  // make sure it is off

   //uint8_t buttons = 0;
//   while (digitalRead(backButton) == HIGH){
//    delay(10);
//    //put menu here
//   }

   //while (digitalRead(backButton) == LOW) delay(10);
}
// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{  
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
     windowStartTime += WindowSize;
  }
  if((onTime > 100) && (onTime > (now - windowStartTime)))
  {
     digitalWrite(RelayPin,HIGH);
  }
  else
  {
     digitalWrite(RelayPin,LOW);
  }
}

// ************************************************
// Proportional Tuning State
// ************************************************
void TuneP()
{
    userInput(2, 10, Kp);
    //if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
    //{
       opState = RUN;
       return;
    //}
    lcd.setCursor(0,1);
    lcd.print(Kp);
    lcd.print(" ");
    DoControl();
    DriveOutput();
}

// ************************************************
// Integral Tuning State
// ************************************************
void TuneI()
{
    userInput(3, 0.1, Ki);
    //if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
    //{
       opState = RUN;
       return;
    //}
    lcd.setCursor(0,1);
    lcd.print(Ki);
    lcd.print(" ");
    DoControl();
    DriveOutput();
}

// ************************************************
// Derivative Tuning State
// ************************************************
void TuneD()
{
    userInput(4, 0.1, Kd);
    //if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
    //{
       opState = RUN;
       return;
    //}
    lcd.setCursor(0,1);
    lcd.print(Kd);
    lcd.print(" ");
    DoControl();
    DriveOutput();
}


// ************************************************
// PID COntrol State
// SHIFT and RIGHT for autotune
// RIGHT - Setpoint
// LEFT - OFF
// ************************************************
void Run()
{
   // set up the LCD's number of rows and columns: 
   SaveParameters();
   myPID.SetTunings(Kp,Ki,Kd);
   digitalWrite( MOTOR_B_DIR, LOW );
   analogWrite( MOTOR_B_PWM, spinRate );

   //while(true)
//   {
//      //setBacklight();  // set backlight based on state
//      if ((digitalRead(buttonPin) == LOW) && (digitalRead(backButton) == LOW) && (abs(Input - Setpoint) < 0.5)){  // Should be at steady-state
//      StartAutoTune();
//      }
//      else if (digitalRead(buttonPin) == LOW){
//        opState = SETP;
//        return;
//      }
//      else if (digitalRead(backButton) == LOW){
//        opState = OFF;
//        return;
//      }
//    }

      DoControl();
      DriveOutput();
      lcd.setCursor(0,0);
      lcd.print(F("Sp: "));
      lcd.print(Setpoint);
      lcd.write(1);
      lcd.print(F("C : "));
      lcd.setCursor(0,1);
      lcd.print(Input);
      lcd.write(1);
      lcd.print(F("C : "));
      
      float pct = map(Output, 0, WindowSize, 0, 1000);
      spinRate = map(Output, 0, WindowSize, 127, 255);
      lcd.setCursor(10,1);
      lcd.print(F("      "));
      lcd.setCursor(10,1);
      lcd.print(pct/10);
      //lcd.print(Output);
      lcd.print("%");

      lcd.setCursor(15,0);
      if (tuning)
      {
        lcd.print("T");
      }
      else
      {
        lcd.print(" ");
      }
      
      // periodically log to serial port in csv format
      if (millis() - lastLogTime > logInterval)  
      {
        Serial.print(Input);
        Serial.print(",");
        Serial.println(Output);
      }
//    if (digitalRead(buttonPin) == LOW){
//      while (digitalRead(buttonPin) == LOW) delay(10);
    if (lastReportedPos != encoderPos){
      userInput(1,0.25,Setpoint);
      opState = RUN;
    }
    else if (digitalRead(backButton) == LOW){
      while (digitalRead(backButton) == LOW) delay(10);
      opState = OFF;
      lcd.clear();
      menu.moveLeft();
      //menu.use();
    }
    delay(100);
 }

 void menuUseEvent(MenuUseEvent used)
{
  Serial.print(F("Menu use "));
  Serial.println(used.item.getName());
//  if (used.item == TurnOff){
//    lcd.clear();
//    menu.moveRight();
//    //Off();
//  }
//  else 
if (used.item == DoRun) {
    opState = RUN;
    // Prepare to transition to the RUN state
   
    //turn the PID on
    myPID.SetMode(AUTOMATIC);
    windowStartTime = millis();
    //Run();
  }
  else if (used.item == DoAutotune) {
    // Prepare to transition to the RUN state
   
    //turn the PID on
    myPID.SetMode(AUTOMATIC);
    windowStartTime = millis();
    opState = RUN; // start control
    StartAutoTune();
  }
  else if (used.item == DoTuneP) {
    // Prepare to transition to the RUN state
   
    //turn the PID on
    myPID.SetMode(AUTOMATIC);
    windowStartTime = millis();
    opState = TUNE_P;
    digitalWrite( MOTOR_B_DIR, LOW );
    digitalWrite( MOTOR_B_PWM, LOW );
    //TuneP();
  }
  else if (used.item == DoTuneI) {
    // Prepare to transition to the RUN state
   
    //turn the PID on
    myPID.SetMode(AUTOMATIC);
    windowStartTime = millis();
    opState = TUNE_I;
    digitalWrite( MOTOR_B_DIR, LOW );
    digitalWrite( MOTOR_B_PWM, LOW );
    //TuneI();
  }
  else if (used.item == DoTuneD) {
    // Prepare to transition to the RUN state
       
    //turn the PID on
    myPID.SetMode(AUTOMATIC);
    windowStartTime = millis();
    opState = TUNE_D;
    digitalWrite( MOTOR_B_DIR, LOW );
    digitalWrite( MOTOR_B_PWM, LOW );
    //TuneD();
  }
}

void menuChangeEvent(MenuChangeEvent changed)
{
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print(changed.to.getName());
  Serial.println(changed.to.getName());
//  if (changed.to.getName() == TurnOff){
//    menu.moveRight();
//  }
//  else 
if (changed.to.getName() == DoRun) {
    lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print(changed.to.getName());
  }
  else if (changed.to.getName() == DoAutotune) {
    //lcd.print(F("Autotuning"));
  }
  else if (changed.to.getName() == DoTuneP) {
    //lcd.print(F("Set Kp"));
  }
  else if (changed.to.getName() == DoTuneI) {
    //lcd.print(F("Set Ki"));
  }
  else if (changed.to.getName() == DoTuneD) {
    //lcd.print(F("Set Kd"));
  }
}

