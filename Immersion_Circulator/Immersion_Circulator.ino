//-------------------------------------------------------------------
// Sous Vide Controller
// Created by SIU07CRD
//
// Credit to:
// Bill Earl - for Adafruit Industries Sous Vide Controller
// Brett Beauregard's Arduino PID and PID AutoTune Libraries
// Alexander Brevig's MenuBackend Libary
//
// GNU GENERAL PUBLIC LICENSE v3
//
// Last Updated 17/11/16
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

//Float switch
boolean fSwitch = TRUE; //start in safe mode

// Rotary Encoder vars
boolean A_set = false;
boolean B_set = false;
static boolean rotating = false;    // debounce management
unsigned int lastReportedPos = 60;   // change management
volatile unsigned int encoderPos = 60;  // a counter for the dial

enum Assignments {  //Unchangeing int's
  encoderPinA = 3,  // scroll right
  encoderPinB = 2,  // scroll left
  buttonPin = 5,    // Select
  backButton = 6,   // back
  RelayPin = 21,    // Output Relay
  ONE_WIRE_BUS = 20,// One-Wire Temperature Sensor
  encoder0PinA = 2, // interrupt service routine
  encoder0PinB = 3, // interrupt service routine
  fCutoff = 7,      // interrupt service routine
  MOTOR_B_PWM = 9,  // motor connections
  SpAddress = 0,    // EEPROM addresses for setpoint persisted data
  KpAddress = 8,    // EEPROM addresses for Kp persisted data
  KiAddress = 16,   // EEPROM addresses for Ki persisted data
  KdAddress = 24,   // EEPROM addresses for Kd persisted data
  WindowSize = 1000 // 1 second Time Proportional Output window
};

// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;
volatile long onTime = 0;   // output relay drive time

//define ramp soak varibles
double pMillis; //previous time calculations were performed
double drive;  //your ramping setpoint, set by calculation
double pDrive; // previous output of ramping setpoint

//pump motor PWM (always 100% for now)
int spinRate = 255;     //motor pwm

// pid tuning parameters
double Kp;
double Ki;
double Kd;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &drive, Kp, Ki, Kd, DIRECT);

unsigned long windowStartTime;

// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember = 2;

double aTuneStep = 500;
double aTuneNoise = 1;
unsigned int aTuneLookBack = 20;

boolean tuning = false;

PID_ATune aTune(&Input, &Output);

// ************************************************
// Display Variables and constants
// ************************************************

LiquidCrystal lcd(10, 16, 14, 15, 18, 19);

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

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempSensor;

//Menu  Elements
void menuChangeEvent(MenuChangeEvent changed);
void menuUseEvent(MenuUseEvent used);
MenuBackend menu = MenuBackend(menuUseEvent, menuChangeEvent);
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
  pinMode( MOTOR_B_PWM, OUTPUT );
  digitalWrite( MOTOR_B_PWM, LOW );

  // Initialize Relay Control:
  pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
  digitalWrite(RelayPin, LOW);  // make sure it is off to start

  //float switch initialize
  pinMode(fCutoff, INPUT);
  digitalWrite(fCutoff, HIGH);

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
  // encoder pin on interrupt 4 (pin 7)
  attachInterrupt(4, dofSwitch, RISING);

  // Initialize LCD Display
  lcd.begin(16, 4);  //need to change to (20, 4) for new screen
  lcd.createChar(1, degree); // create degree symbol from the binary

  //lcd.print(F("    Home"));
  lcd.setCursor(0, 1);
  lcd.print(F("   Sous Vide!")); //splash screen

  // Start up the DS18B20 One Wire Temperature Sensor
  sensors.begin();
  if (!sensors.getAddress(tempSensor, 0))
  {
    lcd.setCursor(0, 1);
    lcd.print(F("Sensor Error")); //alternate bad times splash screen
  }
  sensors.setResolution(tempSensor, 10); //12 for more accurate model.
  sensors.setWaitForConversion(false);
  fSwitch = digitalRead(fCutoff);
  LoadParameters();
  myPID.SetTunings(Kp, Ki, Kd);

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

  delay(2000);  // Splash screen to make it look super smart and pretend its doing super important background stuff

  menu.moveDown(); //move to main menu
  //
}

// ************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************
void loop()
{
  rotating = true; //unlock rotary encoders
  //See if any of the inputs have changed
  if (opState != RUN) { //if its not running, and its in the main loop it must be on the menu
    if (lastReportedPos > encoderPos) {
      menu.moveDown();                //goto previous menu choice
      lastReportedPos = encoderPos;
    }
    else if (lastReportedPos < encoderPos) {
      menu.moveUp();                  //goto next menu choice
      lastReportedPos = encoderPos;
    }
    if (digitalRead(buttonPin) == LOW) {
      while (digitalRead(buttonPin) == LOW) delay(10);  //hold code until input is done
      {
        menu.use();
        menu.moveRight();               //Select next menu down or run action
      }
    }
  }
  if (digitalRead(backButton) == LOW) {
    while (digitalRead(backButton) == LOW) delay(10);
    //Serial.print("backbutton");
    opState = OFF;
    lcd.clear();
    menu.moveLeft();                //go back to previous menu
  }

  //lcd.clear();
  switch (opState) //run or continue running current state
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
void SaveParameters()   //i should probably change this to the EEPROMex library at some point, but this works.
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
void doEncoderA() {
  // debounce
  if ( rotating ) delay (10);  // wait a little until the bouncing is done
  // Test transition, did things really change?
  if ( digitalRead(encoderPinA) != A_set ) { // debounce once more
    A_set = !A_set;
    // adjust counter + if A leads B
    if ( A_set && !B_set )
      encoderPos ++;
    rotating = false;  // no more debouncing until loop() hits again
  }
}

// Interrupt on B changing state, same as A above
void doEncoderB() {
  if ( rotating ) delay (10);
  if ( digitalRead(encoderPinB) != B_set ) {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if ( B_set && !A_set )
      encoderPos --;
    rotating = false;
  }
}

// Interrupt on float switch. Make device safe until fault clears.
void dofSwitch() {
  digitalWrite(RelayPin, LOW);
  digitalWrite( MOTOR_B_PWM, LOW );
  myPID.SetMode(MANUAL);
  lcd.clear();
  lcd.setCursor(2, 4);
  lcd.print("Water Low!");
  fSwitch = digitalRead(fCutoff);
  while (fSwitch == TRUE) fSwitch = digitalRead(fCutoff);
  lcd.clear();
  myPID.SetMode(AUTOMATIC);
  digitalWrite( MOTOR_B_PWM, spinRate); //start pump, but don't bother triggering heater until next pass of RUN
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
    Kp = 650;
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

void userInput(int menuFlag, float scale, double prior) { //Flag to interperate where its from, scale of scroll, default 1 = one scroll =+1, scale of 0.1 = one scroll =+0.1 etc.
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
    if (digitalRead(buttonPin) == LOW) {
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
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(ATuneModeRemember);

  // Persist any changed parameters to EEPROM
  SaveParameters();
}

void rampS(double roChange)  //where the mathgic happens
{
  if (millis() > pMillis) { //has enough time actually passed to bother calculating? These Arduino things are fast
    roChange = roChange / 1000; // change rate of change from user friendly seconds to milliseconds
    if (abs(pDrive - Setpoint) <= roChange * (millis() - pMillis)) { //if the rate of change is going to push the drive past the setpoint, just make it equal the setpoint otherwise it'll oscillate
      drive = Setpoint;
    }
    else { //If more ramping is required, calculate the change required for the time period passed to keep the rate of change constant, and add it to the drive.
      if (Setpoint > pDrive) { //possitive direction
        drive = pDrive + (roChange * (millis() - pMillis));
      }
      else {  //negative direction
        drive = pDrive - (roChange * (millis() - pMillis));
      }
    }
    pMillis = millis();
    pDrive = drive;
  }
}

// ************************************************
// Execute the control loop
// ************************************************
void DoControl()
{
  // Read the input:
  if (sensors.isConversionAvailable(0))
  {
    if (sensors.getAddress(tempSensor, 0)) {
      Input = sensors.getTempC(tempSensor);
      while (Input == -127) { //may get stuck here if sensor breaks any more
        Input = sensors.getTempC(tempSensor);
      }
      sensors.requestTemperatures(); // prime the sensor for the next one - but don't wait
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
void Off()  //don't run the system whilst it's on the main menu, or anywhere other than the "RUN" state
{
  digitalWrite( MOTOR_B_PWM, LOW );
  myPID.SetMode(MANUAL);
  digitalWrite(RelayPin, LOW);  // make sure it is off
}

// ************************************************
// Drive the output
// ************************************************
void DriveOutput()
{
  rampS(0.1); //ramp 0.1c per second. Min 3L, 40c temp rise, 1.5kW = ~6min's = 0.11 max ramp
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if (now - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if ((onTime > 100) && (onTime > (now - windowStartTime)))
  {
    digitalWrite(RelayPin, HIGH);
  }
  else
  {
    digitalWrite(RelayPin, LOW);
  }
}

// ************************************************
// Proportional Tuning State
// ************************************************
void TuneP()
{
  userInput(2, 10, Kp);
  opState = OFF;
  return;
}

// ************************************************
// Integral Tuning State
// ************************************************
void TuneI()
{
  userInput(3, 0.1, Ki);
  opState = OFF;
  return;
}

// ************************************************
// Derivative Tuning State
// ************************************************
void TuneD()
{
  userInput(4, 0.1, Kd);
  opState = OFF;
  return;
}


// ************************************************
// PID Control State
// ************************************************
void Run()
{
  // set up the LCD's number of rows and columns:
  SaveParameters();
  myPID.SetTunings(Kp, Ki, Kd);
  //digitalWrite( MOTOR_B_DIR, LOW );
  spinRate = 255; //overridden because pump now
  analogWrite( MOTOR_B_PWM, spinRate );
  DoControl();
  DriveOutput();
  lcd.setCursor(0, 0);
  lcd.print(F("Sp: "));
  lcd.print(Setpoint);
  lcd.write(1);
  lcd.print(F("C : "));
  lcd.setCursor(0, 1);
  lcd.print(Input);
  lcd.write(1);
  lcd.print(F("C : "));
  float pct = map(Output, 0, WindowSize, 0, 1000); //percentage drive to heater
  lcd.setCursor(10, 1);
  lcd.print(F("      "));
  lcd.setCursor(10, 1);
  lcd.print(pct / 10);
  lcd.print("%");
  lcd.setCursor(15, 0);
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
  if (lastReportedPos != encoderPos) {
    userInput(1, 0.25, Setpoint);
    opState = RUN;
  }
  else if (digitalRead(backButton) == LOW) {
    while (digitalRead(backButton) == LOW) delay(10);
    opState = OFF;
    lcd.clear();
    menu.moveLeft();
  }
  delay(100);
}

void menuUseEvent(MenuUseEvent used)
{
  Serial.print(F("Menu use "));
  Serial.println(used.item.getName());
  if (used.item == DoRun) {
    opState = RUN;      // Prepare to transition to the RUN state
    //turn the PID on
    myPID.SetMode(AUTOMATIC);
    windowStartTime = millis();
  }
  else if (used.item == DoAutotune) {
    //turn the PID on
    myPID.SetMode(AUTOMATIC);
    windowStartTime = millis();
    opState = RUN; // start control
    StartAutoTune();
  }
  else if (used.item == DoTuneP) {
    //turn the PID on
    myPID.SetMode(AUTOMATIC);
    windowStartTime = millis();
    opState = TUNE_P;
  }
  else if (used.item == DoTuneI) {
    //turn the PID on
    myPID.SetMode(AUTOMATIC);
    windowStartTime = millis();
    opState = TUNE_I;
  }
  else if (used.item == DoTuneD) {
    //turn the PID on
    myPID.SetMode(AUTOMATIC);
    windowStartTime = millis();
    opState = TUNE_D;
  }
}

void menuChangeEvent(MenuChangeEvent changed)
{
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print(changed.to.getName());
  Serial.println(changed.to.getName());
  if (changed.to.getName() == DoRun) {
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print(changed.to.getName());
  }
  else if (changed.to.getName() == DoAutotune) {
  }
  else if (changed.to.getName() == DoTuneP) {
  }
  else if (changed.to.getName() == DoTuneI) {
  }
  else if (changed.to.getName() == DoTuneD) {
  }
}
