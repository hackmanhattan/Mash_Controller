//-------------------------------------------------------------------
//
// Mash Controller
// Konstantin Avdashchenko for HackManhattan
// from:
// sous-vide controller by
// Bill Earl - for Adafruit Industries
//
// Based on the Arduino PID and PID AutoTune Libraries 
// by Brett Beauregard
//------------------------------------------------------------------

// PID Library
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

// Libraries for the LCD
#include <Wire.h>
#include <LiquidCrystalFast.h>

// Libraries for the DS18B20 Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// So we can save and retrieve settings
#include <EEPROM.h>

//right timer 
#include <TimerOne.h>

// ************************************************
// Pin definitions
// ************************************************

// Output Relay
#define RelayPin 11

// One-Wire Temperature Sensor
#define ONE_WIRE_BUS 0

// ************************************************
// development mode (constant 0 input)
// ************************************************
//#define dev
// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;

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
// DiSplay Variables and constants
// ************************************************

LiquidCrystalFast lcd(21,20,19,18,17,16,13);

//these defines specify pin number for the rgb
#define REDp 15
#define GREENp 14
#define BLUEp 12
// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

#define BUTTON_SHIFT BUTTON_SELECT

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
enum operatingState { OFF = 0, SETP, RUN, TUNE_P, TUNE_I, TUNE_D, AUTO};
operatingState opState = OFF;

enum terminalState { control = 0, logging};
terminalState tstate = control;


// ************************************************
// Terminal Variables
// ************************************************
//char Command[10]={0,}; //no need for global

// ************************************************
// Sensor Variables and constants
// Data wire is plugged into port 2 on the Arduino

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempSensor;

// ************************************************
// Setup and diSplay initial screen
// ************************************************
void setup()
{
   Serial.begin(9600);

   // Initialize Relay Control:

   pinMode(RelayPin, OUTPUT);    // Output mode to drive relay
   digitalWrite(RelayPin, LOW);  // make sure it is off to start
   pinMode(REDp, OUTPUT);    // Output mode to drive red light
   digitalWrite(REDp, LOW);  // make sure it is on to start

   pinMode(GREENp, OUTPUT);    // Output mode to drive green l
   digitalWrite(GREENp, LOW);  // make sure it is on to start

   pinMode(BLUEp, OUTPUT);    // Output mode to drive relay
   digitalWrite(BLUEp, LOW);  // make sure it is off to start


   // Set up Ground & Power for the sensor from GPIO pins

   //pinMode(ONE_WIRE_GND, OUTPUT);
   //digitalWrite(ONE_WIRE_GND, LOW);

   //pinMode(ONE_WIRE_PWR, OUTPUT);
   //digitalWrite(ONE_WIRE_PWR, HIGH);

   // Initialize LCD DiSplay 

   lcd.begin(16, 2);
   lcd.createChar(1, degree); // create degree symbol from the binary
   
   //setBacklight(VIOLET);
   lcd.setCursor(0, 0);
   lcd.print(F("  HackManhattan "));
   lcd.setCursor(0, 1);
   lcd.print(F("Mash Controller!"));

   // Start up the DS18B20 One Wire Temperature Sensor

   sensors.begin();
   if (!sensors.getAddress(tempSensor, 0)) 
   {
      lcd.setCursor(0, 1);
      lcd.print(F("Sensor Error"));
   }
   sensors.setResolution(tempSensor, 12);
   sensors.setWaitForConversion(false);

   delay(3000);  // Splash screen

   // Initialize the PID and related variables
   LoadParameters();
   myPID.SetTunings(Kp,Ki,Kd);

   myPID.SetSampleTime(1000);
   myPID.SetOutputLimits(0, WindowSize);

//  // Run timer2 interrupt every 15 ms 
//  TCCR2A = 0;
//  TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;
//
//  //Timer2 Overflow Interrupt Enable
//  TIMSK2 |= 1<<TOIE2;
  Timer1.initialize(15000);
  Timer1.attachInterrupt(TimerInterrupt);
}

// ************************************************
// Timer Interrupt Handler
// ************************************************
//SIGNAL(TIMER2_OVF_vect) 
//{
//  if (opState == OFF)
//  {
//    digitalWrite(RelayPin, LOW);  // make sure relay is off
//  }
//  else
//  {
//    DriveOutput();
//  }
//}
void TimerInterrupt() 
{
  if (opState == OFF)
  {
    digitalWrite(RelayPin, LOW);  // make sure relay is off
  }
  else
  {
    DriveOutput();
  }
}

// ************************************************
// Main Control Loop
//
// All state changes pass through here
// ************************************************
void loop()
{
   // wait for button release before changing state
   while(ReadButtons() != 0) {}

   lcd.clear();

   switch (opState)
   {
   case OFF:
      Off();
      break;
   case SETP:
      Tune_Sp();
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
// Initial State - press RIGHT to enter setpoint
// ************************************************
void Off()
{
   myPID.SetMode(MANUAL);
   //setBacight(0);
   digitalWrite(RelayPin, LOW);  // make sure it is off
   lcd.print(F("  HackManhattan "));
   lcd.setCursor(0, 1);
   lcd.print(F("  Mash Troller! "));
   uint8_t buttons = 0;
   
//   while(!(buttons & (BUTTON_RIGHT)))
//   {
//      buttons = ReadButtons();
//   }
   // Prepare to transition to the RUN state
   sensors.requestTemperatures(); // Start an asynchronous temperature reading

   //turn the PID on
   myPID.SetMode(AUTOMATIC);
   windowStartTime = millis();
   opState = RUN; // start control
}

// ************************************************
// Setpoint Entry State
// UP/DOWN to change setpoint
// RIGHT for tuning parameters
// LEFT for OFF
// SHIFT for 10x tuning
// ************************************************
void Tune_Sp()
{
   setLight(TEAL);
   lcd.print(F("Set Temperature:"));
   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();

      float increment = 0.1;
//      if (buttons & BUTTON_SHIFT)
//      {
//        increment *= 10;
//      }
//      if (buttons & BUTTON_LEFT)
//      {
//         opState = RUN;
//         return;
//      }
//      if (buttons & BUTTON_RIGHT)
//      {
//         opState = TUNE_P;
//         return;
//      }
//      if (buttons & BUTTON_UP)
//      {
//         Setpoint += increment;
//         delay(200);
//      }
//      if (buttons & BUTTON_DOWN)
//      {
//         Setpoint -= increment;
//         delay(200);
//      }
    
      if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Setpoint);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Proportional Tuning State
// UP/DOWN to change Kp
// RIGHT for Ki
// LEFT for setpoint
// SHIFT for 10x tuning
// ************************************************
void TuneP()
{
   setLight(TEAL);
   lcd.print(F("Set Kp"));

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();

      float increment = 1.0;
//      if (buttons & BUTTON_SHIFT)
//      {
//        increment *= 10;
//      }
//      if (buttons & BUTTON_LEFT)
//      {
//         opState = SETP;
//         return;
//      }
//      if (buttons & BUTTON_RIGHT)
//      {
//         opState = TUNE_I;
//         return;
//      }
//      if (buttons & BUTTON_UP)
//      {
//         Kp += increment;
//         delay(200);
//      }
//      if (buttons & BUTTON_DOWN)
//      {
//         Kp -= increment;
//         delay(200);
//      }
      if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Kp);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Integral Tuning State
// UP/DOWN to change Ki
// RIGHT for Kd
// LEFT for Kp
// SHIFT for 10x tuning
// ************************************************
void TuneI()
{
   setLight(TEAL);
   lcd.print(F("Set Ki"));

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();

      float increment = 0.01;
//      if (buttons & BUTTON_SHIFT)
//      {
//        increment *= 10;
//      }
//      if (buttons & BUTTON_LEFT)
//      {
//         opState = TUNE_P;
//         return;
//      }
//      if (buttons & BUTTON_RIGHT)
//      {
//         opState = TUNE_D;
//         return;
//      }
//      if (buttons & BUTTON_UP)
//      {
//         Ki += increment;
//         delay(200);
//      }
//      if (buttons & BUTTON_DOWN)
//      {
//         Ki -= increment;
//         delay(200);
//      }
      if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Ki);
      lcd.print(" ");
      DoControl();
   }
}

// ************************************************
// Derivative Tuning State
// UP/DOWN to change Kd
// RIGHT for setpoint
// LEFT for Ki
// SHIFT for 10x tuning
// ************************************************
void TuneD()
{
   setLight(TEAL);
   lcd.print(F("Set Kd"));

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      float increment = 0.01;
//      if (buttons & BUTTON_SHIFT)
//      {
//        increment *= 10;
//      }
//      if (buttons & BUTTON_LEFT)
//      {
//         opState = TUNE_I;
//         return;
//      }
//      if (buttons & BUTTON_RIGHT)
//      {
//         opState = RUN;
//         return;
//      }
//      if (buttons & BUTTON_UP)
//      {
//         Kd += increment;
//         delay(200);
//      }
//      if (buttons & BUTTON_DOWN)
//      {
//         Kd -= increment;
//         delay(200);
//      }
      if ((millis() - lastInput) > 3000)  // return to RUN after 3 seconds idle
      {
         opState = RUN;
         return;
      }
      lcd.setCursor(0,1);
      lcd.print(Kd);
      lcd.print(" ");
      DoControl();
   }
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
   lcd.print(F("Sp: "));
   lcd.print(Setpoint);
   lcd.write(1);
   lcd.print(F("C : "));

   SaveParameters();
   myPID.SetTunings(Kp,Ki,Kd);

   uint8_t buttons = 0;
   while(true)
   {
      setBacklight();  // set backlight based on state

      buttons = ReadButtons();
//      if ((buttons & BUTTON_SHIFT) 
//         && (buttons & BUTTON_RIGHT) 
//         && (abs(Input - Setpoint) < 0.5))  // Should be at steady-state
//      {
//         StartAutoTune();
//      }
//      else if (buttons & BUTTON_RIGHT)
//      {
//        opState = SETP;
//        return;
//      }
//      else if (buttons & BUTTON_LEFT)
//      {
//        opState = OFF;
//        return;
//      }
      
      DoControl();
      
      lcd.setCursor(0,1);
      lcd.print(Input);
      lcd.write(1);
      lcd.print(F("C : "));
      
      float pct = map(Output, 0, WindowSize, 0, 1000);
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
      
      switch(tstate){
        case control:
          terminal_control();
          break;
        case logging:
          // periodically log to serial port in csv format
          if (millis() - lastLogTime > logInterval)  
          {
            Serial.print(Input);
            Serial.print(",");
            Serial.println(Output);
          }
          break;
        default:
          break;
      }

      delay(100);
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
#ifdef dev
    Input = 0;
#else
    Input = sensors.getTempC(tempSensor);
#endif
    sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
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
// Set Backlight based on the state of control
// ************************************************
void setBacklight()
{
   if (tuning)
   {
      setLight(VIOLET); // Tuning Mode
   }
   else if (abs(Input - Setpoint) > 1.0)  
   {
      setLight(RED);  // High Alarm - off by more than 1 degree
   }
   else if (abs(Input - Setpoint) > 0.2)  
   {
      setLight(YELLOW);  // Low Alarm - off by more than 0.2 degrees
   }
   else
   {
      setLight(WHITE);  // We're on target!
   }
}
// actually drive the color
void setLight(char light)
{
   if (light&0x01) digitalWrite(REDp,LOW);
   else digitalWrite(REDp,HIGH);
   if (light&0x02) digitalWrite(GREENp,LOW);
   else digitalWrite(GREENp,HIGH);
   if (light&0x04) digitalWrite(BLUEp,LOW);
   else digitalWrite(BLUEp,HIGH);
}
// ************************************************
// Start the Auto-Tuning cycle
// ************************************************

void StartAutoTune()
{
   // REmember the mode we were in
   ATuneModeRemember = myPID.GetMode();

   // set up the auto-tune parameters
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
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
// Check buttons and time-stamp the last press
// ************************************************
uint8_t ReadButtons()
{
  uint8_t buttons = 0;//lcd.readButtons();
  if (buttons != 0)
  {
    lastInput = millis();
  }
  return buttons;
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
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   Setpoint = EEPROM_readDouble(SpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);
   
//   Setpoint = 54.44;
//   Kp = 850;
//   Ki = 0.5;
//   Kd = 0.1;
//   SaveParameters();

   // Use defaults if EEPROM values are invalid
   if (isnan(Setpoint))
   {
     Setpoint = 66.67;
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
// Control via terminal
// ************************************************
void terminal_control(){
//  Serial.write(27);       // ESC command
//  Serial.print("[2J");    // clear screen command
//  Serial.write(27);
//  Serial.print("[H");     // cursor to home command
//  Serial.write(27);
//  Serial.print("[32m");     // set to color green
//  Serial.print("Hello Green World");     
  
  Serial.write(27);
  Serial.print("[37m");     // set to color white
  Serial.write(27);       // ESC command
  Serial.print("[2J");    // clear screen command
  Serial.write(27);
  Serial.print("[H");     // cursor to home command
  Serial.println("***********************************");
  Serial.println("HackManhattan Temperature control");
  Serial.println("By:Konsgn Revision:1.2");
  Serial.println("Based on Adafruit Sous-Vide example");
  Serial.println("***********************************");
  Serial.print(F("Sp: "));
  Serial.print(Setpoint);
  Serial.println(F("C : "));
  if (abs(Input - Setpoint) > 1.0)  
   {
      Serial.write(27);
      Serial.print("[31m");     // set to color red
   }
   else if (abs(Input - Setpoint) > 0.2)  
   {
      Serial.write(27);
      Serial.print("[33m");     // set to color yellow
   }
   else
   {
      Serial.write(27);
      Serial.print("[32m");     // set to color green
   }
  Serial.print(Input);
  Serial.write(27);
  Serial.print("[37m");     // set to color white
  Serial.print(F("C : "));
  float pct = map(Output, 0, WindowSize, 0, 1000);
  Serial.print(pct/10);
  //lcd.print(Output);
  Serial.println("%");

  Serial.println("***********************************");
  Serial.print("Kp=");
  Serial.println(Kp);
  Serial.print("Ki=");
  Serial.println(Ki);
  Serial.print("Kd=");
  Serial.println(Kd);
  Serial.println("***********************************");
  Serial.println("Commands Available:");
  Serial.println("cTsxxx.xxx = set setpoint");
  Serial.println("cKpxxx.xxx = set Kp");
  Serial.println("cKixxx.xxx = set Ki");
  Serial.println("cKdxxx.xxx = set Kd");
  Serial.println("cLog = set to CSV log mode");
  Serial.println("cSave = Saves the current settings");
  Serial.println("***********************************");
  Serial.println("Notes:");
  Serial.println("The lowercase c frames the command.");
  Serial.println("Press enter to execute command.");
  Serial.println("To clear CSV:reset device");
  if ((Serial.available())&&(Serial.read()=='c'))serial_control();
  else Serial.flush();
  
  //tstate=logging;
}

void serial_control(){
  unsigned long time;
  char test[2], i=0;
  float temp;
  time=millis();
  Serial.println(">");
  while(((millis()-time)<=4000)&&(i!=2)){
    if (Serial.available()){
      test[i]=Serial.read();
      i++;
      if(test[i]!='/r')Serial.print(test[i]);
      else {Serial.print("/nIncomplete Command");break;}
    }
  }
  switch (test[1]){
    case 's':
      Serial.println("");
      Serial.println("Enter Temperature Setpoint");
      Serial.setTimeout(4000);
      temp=Serial.parseFloat();
      Serial.print("Setpoint=");
      Serial.println(temp);
      Setpoint = double(temp);
      delay(3000);
      break;
    case 'p':
      Serial.println("");
      Serial.println("Enter Kp Setpoint");
      Serial.setTimeout(4000);
      temp=Serial.parseFloat();
      Serial.print("Setpoint=");
      Serial.println(temp);
      Kp = double(temp);
      delay(3000);
      break;
    case 'i':
      Serial.println("");
      Serial.println("Enter Ki Setpoint");
      Serial.setTimeout(4000);
      temp=Serial.parseFloat();
      Serial.print("Setpoint=");
      Serial.println(temp);
      Ki = double(temp);
      delay(3000);
      break;
    case 'd':
      Serial.println("");
      Serial.println("Enter Kd Setpoint");
      Serial.setTimeout(4000);
      temp=Serial.parseFloat();
      Serial.print("Setpoint=");
      Serial.println(temp);
      Kd = double(temp);
      delay(3000);
      break;
    case 'o':
      Serial.println("");
      Serial.println("Entering Logging mode");
      Serial.println("Restart to exit logging");
      tstate=logging;
      delay(3000);
      break;
    case 'a':
      if(test[0]=='S'){
        SaveParameters();
        Serial.println("");
        Serial.println("Parameters Saved");
      }
      else Serial.println("/nCommand Invalid");
      break;
    default:break;
  }
  return;
}
  
  
