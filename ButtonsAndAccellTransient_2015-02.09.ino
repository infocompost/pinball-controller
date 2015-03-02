//Test proto-board button inputs and accellerometer
//2015-02.09 remove USB cmns from interrupt handlers to avoid timingproblems with USB messages. 
//  BUT... Still getting stuck/inverted keys with USB
//  Required delay in I2C readRegister routine to prevent sudden stops
//2015-02.05 Corrected key assignment errors but USB 'keys' get stuck on.
//2015-02.04 Conversion to USB output attempted.  Key assignments messed up.
//2015-02.02 Set up USB HID keyboard string for multiple keys on at once - Compiled but not tested
// Array posn, value: 0,0; 1,0; 2,Select; 3,Plunger; 4,Left Flipper; 5,Right Flipper; 6,Nudges; 7,0.
//2015-01.31 Change accelerometer function from tap to transient detection.
//2015-01.14 Acceleromter code added in to detect tap in the Y axis. Still using only serial comns.  
//  Sensitive to taps but not nudges.
//  Main loop is too slow.  Probably due to i2c communication to accellerometer.
//2015-01.13 All four buttons work with output to Serial Monitor
//  Flipper buttons are detected by interrupt.  The other two buttons are only detected in the main loop.
//  Release of all buttons is not by interrupt.


#include <Wire.h> // Used for I2C

#define USB_CMNS 1  //0 for serial, 1 for USB

//Key ID's
#define SPACE_BAR 44
#define Z_KEY 29  //55
#define SLASH_KEY 56 //27
#define ESC_KEY 41
#define RETURN_KEY 40
#define X_KEY 27
#define PERIOD_KEY 55

//Indexes for USB array
#define SELECT_IDX 2
#define PLUNGER_IDX 3
#define LFLIP_IDX 4
#define RFLIP_IDX 5
#define NUDGES_IDX 6

//Hardware locations
#define LFlip_BUTTON 2
#define RFlip_BUTTON 3
#define SELECT_BUTTON 4
#define PLUNGER_BUTTON 5
#define L_INTERRUPT 0
#define R_INTERRUPT 1
#define OffCountLimit 100

//Define registers for the MMA8452
#define OUT_X_MSB 0x01
#define XYZ_DATA_CFG  0x0E
#define WHO_AM_I  0x0D
#define CTRL_REG1 0x2A
#define CTRL_REG2 0x2B // 
#define CTRL_REG3 0x2C // 
#define CTRL_REG4 0x2D // 
#define CTRL_REG5 0x2E // 
#define PULSE_CFG 0x21 // pulse configuration register
#define PULSE_THSX 0x23 // x thresh
#define PULSE_THSY 0x24 // y thresh 
#define PULSE_THSZ 0x25 // z thresh 
#define PULSE_TMLT 0x26 // pulse time limit at 800Hz odr, this is very dependent on data rate, see the app note
#define PULSE_LTCY 0x27 // time between taps min, this also depends on the data rate
#define PULSE_WIND 0x28 // time between taps max
#define Transient_CFG 0x1D   //Transient Configuration - set to 0x16 to latch and enable x & y
#define Transient_THS 0x1F  //Transient threshold settings - 0x0F might be a good start
#define Transient_COUNT 0x20  //Transient debounce counter settings - tru 0x08 for 10ms
#define Transient_SRC 0x1E  //Transielnt Source Detection - read status here
#define EA_BIT 0x40     //Transient event active bit in Transient_SRC register  
#define XTRANS_BIT 0x02 //X axis transient detected 
#define XY_BITS 0x0F
#define GSCALE 2 // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
// The SparkFun breakout board defaults to 1, set to 0 if SA0 jumper on the bottom of the board is set
#define MMA8452_ADDRESS 0x1D  // 0x1D if SA0 is high, 0x1C if low

//Variables
int state = 1;
int LFlipState = 1;
int RFlipState = 1;
int PlungerState = 1;
int SelectState = 1;
volatile boolean LFlipDebouncing = false;
volatile boolean RFlipDebouncing = false;
boolean PlungerDebouncing = false;
boolean SelectDebouncing = false;
boolean nudgeDebouncing = false;      
int LFlipDebounceCount = 0;
int RFlipDebounceCount = 0;
int PlungerDebounceCount = 0;
int SelectDebounceCount = 0;
int LFlipOffCount = 0;
int RFlipOffCount = 0;
int PlungerOffCount = 0;
int SelectOffCount = 0;
int returnedKeyCode = 0;
volatile unsigned long LFlipDebounceTime;
volatile unsigned long RFlipDebounceTime;
volatile unsigned long nudgeDebounceTime;
unsigned long PlungerDebounceTime;
unsigned long SelectDebounceTime;
unsigned long DebounceTimeLimit = 10; //ms
unsigned long nudgeTimeLimit = 10; //ms
unsigned long TimeSample;
unsigned long TimeDelta;

uint8_t buf[8] = { 0 }; 	/* Keyboard report buffer from USB kbd HID demo */
boolean kbStatusUpdated = false;
uint8_t keyboardStatus[8] = { 0 }; 	/* Keyboard report array */
uint8_t allKeysOff[8] = { 0 };


/***********************************************************************************
************************************************************************************/
void setup() {
  if (!USB_CMNS) {
    Serial.begin(57600); //For serial communications
    Serial.println("Button & Accellerometer Test");
  }
  else if (USB_CMNS) {
    Serial.begin(9600);  //Setting for USB communication
    Serial.write(keyboardStatus, 8); //Ensure no keys on to start with
  }
  pinMode(LFlip_BUTTON, INPUT);
  pinMode(RFlip_BUTTON, INPUT);
  pinMode(SELECT_BUTTON, INPUT);
  pinMode(PLUNGER_BUTTON, INPUT);
  // enable internal pull-ups
  digitalWrite(LFlip_BUTTON, 1);
  digitalWrite(RFlip_BUTTON, 1);
  digitalWrite(SELECT_BUTTON, 1);
  digitalWrite(PLUNGER_BUTTON, 1);

  Wire.begin(); //Join the i2c bus as a master
  initMMA8452(); //Test and intialize the MMA8452

//Serial.println("Return from initMMA8452");
  
  delay(200);
  LFlipOffCount = 0;
  RFlipOffCount = 0;
  attachInterrupt(L_INTERRUPT, LFlipCatchPress, FALLING);
  attachInterrupt(R_INTERRUPT, RFlipCatchPress, FALLING);
}

/***********************************************************************************
Main loop
************************************************************************************/
void loop() {
TimeSample = micros();  //debug
  if (LFlipDebouncing) {   //enter here if an edge interrupt was completed
    LFlipOffCount = UpdateOffCount(LFlip_BUTTON, LFlipOffCount);
    if ((LFlipDebounceTime < (millis() - DebounceTimeLimit)) && (LFlipOffCount > OffCountLimit))  {
      LFlipDebouncing = false;  //reset flags
      LFlipOffCount = 0;
      ReleaseKey(LFLIP_IDX); 
      attachInterrupt(L_INTERRUPT, LFlipDummy, FALLING); //to clear interrupt flag
      detachInterrupt(L_INTERRUPT);
      attachInterrupt(L_INTERRUPT, LFlipCatchPress, FALLING);  //ready for next interrupt
    }
  }
  if (RFlipDebouncing) {   //enter here if an edge interrupt was completed
    RFlipOffCount = UpdateOffCount(RFlip_BUTTON, RFlipOffCount);
    if ((RFlipDebounceTime < (millis() - DebounceTimeLimit)) && (RFlipOffCount > OffCountLimit))  {
      RFlipDebouncing = false;  //reset flags
      RFlipOffCount = 0;
      ReleaseKey(RFLIP_IDX); 
      
//TimeDelta = micros() - TimeSample;  //debug
//Serial.println(TimeDelta);       //debug      
      
      attachInterrupt(R_INTERRUPT, RFlipDummy, FALLING); //to clear interrupt flag
      detachInterrupt(R_INTERRUPT);
      attachInterrupt(R_INTERRUPT, RFlipCatchPress, FALLING);  //ready for next interrupt
    }
  }   
  
  if (!PlungerDebouncing) {
    state = digitalRead(PLUNGER_BUTTON);
    if (state != 1) {
      PressKey(PLUNGER_IDX,RETURN_KEY); 
      PlungerDebounceTime = millis();
      PlungerDebouncing = true;      
    }
  }  
  else if (PlungerDebouncing) {   //enter here if button press detected
    PlungerOffCount = UpdateOffCount(PLUNGER_BUTTON, PlungerOffCount);
    if ((PlungerDebounceTime < (millis() - DebounceTimeLimit)) && (PlungerOffCount > OffCountLimit))  {
      PlungerDebouncing = false;  //reset flags
      PlungerOffCount = 0;
        ReleaseKey(PLUNGER_IDX); 
    }
  }       
  if (!SelectDebouncing) {
    state = digitalRead(SELECT_BUTTON);
    if (state != 1) {
      PressKey(SELECT_IDX,ESC_KEY); 
      SelectDebounceTime = millis();
      SelectDebouncing = true;

//debugDisplay();
      
    }
  }  
  else if (SelectDebouncing) {   //enter here if button press detected
    SelectOffCount = UpdateOffCount(SELECT_BUTTON, SelectOffCount);
    if ((SelectDebounceTime < (millis() - DebounceTimeLimit)) && (SelectOffCount > OffCountLimit))  {
      SelectDebouncing = false;  //reset flags
      SelectOffCount = 0;
      ReleaseKey(SELECT_IDX); 
      
//debugDisplay();      
      
    }
  }

  if (kbStatusUpdated) {
    sendUsbArray();
  }


//  if (!nudgeDebouncing) {
//    if (nudgeEvent(&returnedKeyCode)) {
//
////      delayMicroseconds(1000);
//
//      PressKey(NUDGES_IDX,returnedKeyCode); 
//      nudgeDebounceTime = millis();
//      nudgeDebouncing = true;     
//
////debugDisplay(); 
//      
//    }
//  }
//  else if (nudgeDebouncing) {
//    if (nudgeDebounceTime < (millis() - nudgeTimeLimit))  {
//      nudgeDebouncing = false;  //reset debounce flag
//      ReleaseKey(NUDGES_IDX); 
//
////debugDisplay();
//
//    }     
//  }  
}
//  accelTransientHandler(); //See if the accellerometer detected anything
//  tapHandler(); //See if the accellerometer detected anything
 
void debugDisplay() { 
int j;
  for (j = 0; j < 7; j++ )
  {
    Serial.print(keyboardStatus[j],DEC);
    Serial.print(",");    
  }
Serial.println("-"); 
 
} 
  
/***********************************************************************************
Interrupt handlers
************************************************************************************/
void LFlipCatchPress() {
  detachInterrupt(L_INTERRUPT); 
//  if (USB_CMNS) {
    PressKey(LFLIP_IDX,Z_KEY);
//  }
//  else if (!USB_CMNS) {
//    Serial.println("L");
//  }
  LFlipDebounceTime = millis();
  LFlipDebouncing = true;
}

void RFlipCatchPress() {
  detachInterrupt(R_INTERRUPT); 
//  if (USB_CMNS) {
    PressKey(RFLIP_IDX,SLASH_KEY);
//  }
//  else if (!USB_CMNS) {
//    Serial.println("R");
//  }
  RFlipDebounceTime = millis();
  RFlipDebouncing = true;
}

void LFlipDummy() {
  detachInterrupt(L_INTERRUPT); 
}

void RFlipDummy() {
  detachInterrupt(R_INTERRUPT); 
}

/***********************************************************************************
//Update the off count for a button 
************************************************************************************/
int UpdateOffCount(int Button, int OffCount){
  int ButtonState = 0;
  ButtonState = digitalRead(Button);  //record button state
  if (ButtonState == 1) {
   OffCount++; //count consecutive loops with the button off
  }
  else {
   OffCount = 0;  //reset off count
  }   
  return OffCount;
}  

/***********************************************************************************
Accelerometer initialization
************************************************************************************/
// Initialize the MMA8452 registers 
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8452Q
void initMMA8452()
{
  byte c = readRegister(WHO_AM_I);  // Read WHO_AM_I register
  if (c == 0x2A) // WHO_AM_I should always be 0x2A
  {  
    if (!USB_CMNS) { 
      Serial.println("MMA8452Q is online...");
    }
  }
  else if (c != 0x2A)
  {
    if (!USB_CMNS) {
      Serial.print("Could not connect to MMA8452Q: 0x");
      Serial.println(c, HEX);
    }
    while(1) ; // Loop forever if communication doesn't happen
  }

  MMA8452Standby();  // Must be in standby to change registers
  //Serial.println("Return from MMA8452Standby");
    // Set up the full scale range to 2, 4, or 8g.
  byte fsr = GSCALE;
  if(fsr > 8) fsr = 8; //Easy error check
  fsr >>= 2; // 00 = 2G, 01 = 4G, 10 = 8G  Shift by two bits to convert.
  writeRegister(XYZ_DATA_CFG, fsr);
 
/* Set up accelleration transient detection
Set Transient_CFG 0x1D Transient Configuration - set to 0x16 to latch and enable x & y
Set Transient_THS 0x1F Transient threshold settings - 0x0F might be a good start
Set Transient_COUNT 0x20 Transient debounce counter settings - tru 0x08 for 10ms
Set Transient_SRC 0x1E Transient Source Detection - read status here   */
  writeRegister(Transient_CFG, 0x16);
  writeRegister(Transient_THS, 0x04);
  writeRegister(Transient_COUNT, 0x0C);
  MMA8452Active();  // Set to active to start reading
}

/***********************************************************************************
Accellerometer modes
************************************************************************************/
// Sets the MMA8452 to standby mode. It must be in standby to change most register settings
void MMA8452Standby()
{

  Serial.println("Inside MMA8452Standby");  

  byte c = readRegister(CTRL_REG1);
  writeRegister(CTRL_REG1, c & ~(0x01)); //Clear the active bit to go into standby
}

// Sets the MMA8452 to active mode. Needs to be in this mode to output data
void MMA8452Active()
{
  byte c = readRegister(CTRL_REG1);
  writeRegister(CTRL_REG1, c | 0x01); //Set the active bit to begin detection
}
/***********************************************************************************
IC2 routines
************************************************************************************/
// Read bytesToRead sequentially, starting at addressToRead into the dest byte array
void readRegisters(byte addressToRead, int bytesToRead, byte * dest)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToRead);
  Wire.endTransmission(false); //endTransmission but keep the connection active

  Wire.requestFrom(MMA8452_ADDRESS, bytesToRead); //Ask for bytes, once done, bus is released by default

  while(Wire.available() < bytesToRead); //Hang out until we get the # of bytes we expect

  for(int x = 0 ; x < bytesToRead ; x++)
    dest[x] = Wire.read();    
}

// Read a single byte from addressToRead and return it as a byte
byte readRegister(byte addressToRead)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToRead);
  Wire.endTransmission(false); //endTransmission but keep the connection active

  Wire.requestFrom(MMA8452_ADDRESS, 1); //Ask for 1 byte, once done, bus is released by default

//debug
//Serial.println("rq"); 

  while(!Wire.available()) ; //Wait for the data to come back

//debug  
//Serial.println("rc"); 

  delayMicroseconds(700);  //This delay prevents hanging
  return Wire.read(); //Return this one byte
}

// Writes a single byte (dataToWrite) into addressToWrite
void writeRegister(byte addressToWrite, byte dataToWrite)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToWrite);
  Wire.write(dataToWrite);
  Wire.endTransmission(); //Stop transmitting
}

/***********************************************************************************
Read accellerometer 
************************************************************************************/
// This function will read the status of the transient source register.
/*
void accelTransientHandler()
{
  byte filteredSource = 0x00;
  byte rawSource = readRegister(Transient_SRC);  // Reads the Transient_SRC register
//TimeSample = micros();
  if ((rawSource & EA_BIT)==EA_BIT) { // If Event Active bit is set
    filteredSource = (rawSource & XY_BITS);
    switch (filteredSource) {
      case 0x08:  //Forward nudge
        ToggleKey(NUDGES_IDX,SPACE_BAR);
        break;
      case 0x02:  //Right nudge
        ToggleKey(NUDGES_IDX,PERIOD_KEY);
        break;
      case 0x03:  //Left nudge
        ToggleKey(NUDGES_IDX,X_KEY);
        break;
      case 0x0A:  //Forward + Right Nudge
        ToggleKey(NUDGES_IDX,SPACE_BAR);
        break;
      case 0x0B:  //Forward + Left Nudge
        ToggleKey(NUDGES_IDX,SPACE_BAR);
        break;      
      }
  }
}  
*/
// This function reads transient source register and returns the key code for the detected nudge.
boolean nudgeEvent(int *keyCode)
{
  boolean nudged = false;
  *keyCode = 0;
  byte filteredSource = 0x00;
  byte rawSource = readRegister(Transient_SRC);  // Reads the Transient_SRC register
//TimeSample = micros();
  if ((rawSource & EA_BIT)==EA_BIT) { // If Event Active bit is set
    filteredSource = (rawSource & XY_BITS);
    nudged = true;
    switch (filteredSource) {
      case 0x08:  //Forward nudge
        *keyCode = SPACE_BAR;
        break;
      case 0x02:  //Right nudge
        *keyCode = PERIOD_KEY;
        break;
      case 0x03:  //Left nudge
        *keyCode = X_KEY;
        break;
      case 0x0A:  //Forward + Right Nudge
        *keyCode = SPACE_BAR;
        break;
      case 0x0B:  //Forward + Left Nudge
        *keyCode = SPACE_BAR;
        break;      
      }
  }
  return nudged;
}  


/***********************************************************************************
USB functions
************************************************************************************/
void PressKey (int keyIndex,int keyCode)
{
  keyboardStatus[keyIndex] = keyCode;
  kbStatusUpdated = true;
//  if (USB_CMNS) {
//    Serial.write(keyboardStatus, 8);	// Send key on 
//  }
//  else if (!USB_CMNS) {
//    Serial.println(keyboardStatus[keyIndex]);    
//  }

}

void ReleaseKey (int keyIndex)
{
  keyboardStatus[keyIndex] = 0;
  kbStatusUpdated = true;
//  if (USB_CMNS) {
//    Serial.write(keyboardStatus, 8);	// Send key off
//  }
//  else if (!USB_CMNS) {
//    Serial.println(keyboardStatus[keyIndex]);    
//  }
}

void sendUsbArray()  {
  kbStatusUpdated = false;
  if (USB_CMNS) {
    
delay(1000);    
    
    Serial.write(keyboardStatus, 8);	// Send key status 
  }
  else if (!USB_CMNS) {
    int j;
    for (j = 0; j < 7; j++ )
    {
      Serial.print(keyboardStatus[j],DEC);
      Serial.print(",");    
    }
    Serial.println(" "); 
  }
}


