 // Dirty code by Daniel Schatzmayr @2009
// 30.01.2012
//#001-08.02.2012

#if _MSC_VER 
#include "VSPDE.h"
#include <math.h>
#define byte char
#endif

////////////////////////////////////

////////////////////////////////////
#if !_MSC_VER 
//#include "Servo.h"
#include <Servo.h> 
#include <EEPROM.h>

#endif

/* Old LayOut
 #define SENSOR_CENTER 		3
 #define SENSOR_UP	        4
 #define SENSOR_DOWN			1
 #define SENSOR_LEFT			2
 #define SENSOR_RIGHT		5 
 **/

// New Layout
#define SENSOR_CENTER 		3
#define SENSOR_UP	        4
#define SENSOR_DOWN			1
#define SENSOR_LEFT			5
#define SENSOR_RIGHT		2 

int SENSOR_CENTER_OFFSET =  (-4);
int SENSOR_UP_OFFSET  =     1;
int SENSOR_DOWN_OFFSET  =   0;
int SENSOR_LEFT_OFFSET   =  (7);
int SENSOR_RIGHT_OFFSET  =  (10);

#define PIN_SERVOTOP           10
//#define PIN_SERVOTOP           8
#define PIN_SERVOTOP_DIR       (0.95f)
#define PIN_SERVOMIDDLE        9
#define PIN_SERVOMIDDLE_DIR    (-0.70f)
//#define PIN_SERVOBOTTOM        10
#define PIN_SERVOBOTTOM        8
#define PIN_SERVOBOTTOM_DIR    (-0.95f)

#define LED_PIN 13

#define PIN_REFVOLTAGESUPPLY   12

int PIN_MENUBUTTON_LEFT		=	6;
int PIN_MENUBUTTON_RIGHT	=	7;

// This pwm values are the micro seconds for the pulse width 
#define PWM_SERVO_MIN			550
#define PWM_SERVO_MAX			2400
#define PWM_SERVO_MID			( PWM_SERVO_MIN + (PWM_SERVO_MAX - PWM_SERVO_MIN)/2)

#define PWM_SERVO_MIN_BORDER     (PWM_SERVO_MIN * 1.02f + 2)
#define PWM_SERVO_MAX_BORDER	 (PWM_SERVO_MAX * 0.98f - 2)

/// The minimun sernsor value for switching off (the to dark value)

/////////// START FUNCTION HEADER DEFINITION SECTION (CAUSE ALL FUNCTIONS ARE WRITTEN INTO THIS FILE)////////////////////////////////
/////////////////////
#define SENSOR_POWEROFFVALUE	150
/// Use this to attach (On=true) or detach (On=false) on of the 3 Servos ID Top=1, Middle=2, Bottom=3
void SaveDeAttach(int ServoNrTopMiddleBottom,bool On);
/// This Checks if movment is going on and if not it will detach the specific servo
void PowerSave();
/// This Checks if flower should go into Sleep Mode return if awake and false if sleeping
bool CheckForSleepingInTheDark();
/// This is doing the checking for the PowerOf
boolean TestPowerSave(int* prvPos,int* nowPos,long* timeCounter);
/// This is for Debug Information via Serial
//void DebugOutput(int sensorValue_Up,int sensorValue_Center, int sensorValue_Down, int sensorValue_Left, int sensorValue_Right);
void DebugOutput(struct SensorValues Sensor);
/// Setup And Calibration Code (Checks if buttons are pressed etc);
bool SetupAndMenu();
/// delay is used to let the Servo Move to a certain position
//void ServoSaveDelay(int DelayMiliSec);
/// Checks if the Bottom Servo needs to be turned 180 Degrees
void BottomServoSideChangingCheck();
// Reads all sensor inputs and stores it in the struct SensorValues
struct SensorValues GetSensorData();
// Buffers the input Value and returns the smoothed value struct SernsorValue
struct SensorValues BufferAndSmoothValues(const struct SensorValues input);
// Dirty Buffer Button Press Handler
int ButtonCheck(int ButtonPinLeft,int ButtonPinRight);
// CheckMenu() checks the button presses and the menu state switchings
bool CheckMenu();
// Moves all servos to the given position
void MoveToPositions(int temp_Pos_ServoBottom,int temp_Pos_ServoMiddle,int temp_Pos_ServoTop, int delayPerStepMicroSeconds,int PinID);
// Saves the Offset Value of Middle and Top Servo to EEPROM
void SaveOffsetValues();
// Saves the Offset Values of all Sensors to EEPROM
void SaveSensorValues();
// Blinks nice
void Blinky();
// More Blinky
void Blinky(int BlinkCounter,int msSpeed);
// Asynco Blinking
void BlinkAsync(int CountSpeedOn,int CountSpeedOff);
//Slowly Get All to Middle Positions 
void SlowlyGetAllToMiddlePosition();
// Checks if serial input is avalible and process the input data
bool SerialMenu();
// CalculateTheNewPositions given the new sensor values
void CalculateNewPositions(float PowerOf,float Scaling,int agility,bool vamp);
// Nicken 
void Nicken(bool Yes, int Count);
// To Reset the Button Buffer Values
void ButtonBufferReset();
//Should Make funky sinus dance
//void SinusDance();
//Switches all servo Safe off!
void AllServosOff();

/////////// END FUNCTION HEADER DEFINITION SECTION (CAUSE ALL FUNCTIONS ARE WRITTEN INTO THIS FILE)//////////////////////////////////
///////////////////
// Define the Servos
Servo ServoTop; // Servo Blume
Servo ServoMiddle; // Servo Blume
Servo ServoBottom;

#define FRAME_BREAK_MS		20

/// Menu Status -----
/// 0 ... normal mode
/// 1 ... calibration mode
/// 4 ... 90 degree calibration
int MenuStatus = 0;


unsigned long prvMillis;

/// Define bufferd data struct for smoothing
struct SensorValues
{
  int SensorUp;
  int SensorDown;
  int SensorLeft;
  int SensorRight;
  int SensorCenter;
}
SENSORVALUES, *PSENSORVALUES;



// Helping code must be made nicer

void EEPROMWriteInt(int p_address, int p_value)
{
  byte lowByte = ((p_value >> 0) & 0xFF);
  byte highByte = ((p_value >> 8) & 0xFF);

  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}


unsigned int EEPROMReadInt(int p_address)
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);

  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}


int Pos_ServoTop = PWM_SERVO_MID, Pos_ServoMiddle = PWM_SERVO_MID, Pos_ServoBottom = PWM_SERVO_MID;
int Offset_ServoMiddle = 0,Offset_ServoTop = 0;

// Used for the Blink effect
bool OnOff = true;int BlinkCounter = 0;

void setup()
{
  
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW);
  
  Blinky(3,200);

  // Flower Started 4 the very first time
  if(EEPROMReadInt(16) != 2)
  {
    // Write basic values into 
    SaveSensorValues();
    SaveOffsetValues();

    EEPROMWriteInt(16,2); 
  }
  //Load Calibration Data from EEPROM 	
  // [SC][SU][SD][SL][SR][DB][DM][DT]
  // SensorOffsetCenter-Up-Doen_left_right;DaempferServoBottom-Middel-Top
  SENSOR_CENTER_OFFSET    =		EEPROMReadInt(0) - 400; // The minus 400 is an offset for not getting and not dealing with unsigned int.(DirtyDirtyDirty)
  SENSOR_UP_OFFSET		=		EEPROMReadInt(2) - 400;
  SENSOR_DOWN_OFFSET		=		EEPROMReadInt(4) - 400;
  SENSOR_LEFT_OFFSET		=		EEPROMReadInt(6) - 400;
  SENSOR_RIGHT_OFFSET		=		EEPROMReadInt(8) - 400;
  // Offset to 90 degree
  Offset_ServoMiddle		=		EEPROMReadInt(10) - 1400;
  Offset_ServoTop			=		EEPROMReadInt(12) - 1400;

   int SetupDone			   =		EEPROMReadInt(14);
   int SwitchButtonPins			=		EEPROMReadInt(18);

  // Setup Serial Connection
  Serial.begin(9600);
  //Serial.println("start it up, fire it up!");
  // Attach the Servos to the pins	
  // Set servos to beginning position 

   SlowlyGetAllToMiddlePosition();
 
  //Set a pin to 5V for the reference Voltage
  // pinMode(PIN_REFVOLTAGESUPPLY, OUTPUT);
  // digitalWrite(PIN_REFVOLTAGESUPPLY, HIGH);

  // Setup Button Pin     
  pinMode(PIN_MENUBUTTON_RIGHT, INPUT);
  digitalWrite(PIN_MENUBUTTON_RIGHT, HIGH);

  pinMode(PIN_MENUBUTTON_LEFT, INPUT);
  digitalWrite(PIN_MENUBUTTON_LEFT, HIGH);
	
  if(SetupDone != 42)
  {   
    delay(500);
  
    
//	   Nicken(false,1);
	     AllServosOff();
	  // Get Button Connections
	  int Count = 0;
	  bool OnOff = true;
	  int BlinkCounter = 0;
	  while(true)
	  {		
		  if (millis() - prvMillis > FRAME_BREAK_MS ) 
		  {
			prvMillis = millis();

			BlinkAsync(20,20);

			int ButtonVal = ButtonCheck(PIN_MENUBUTTON_LEFT,PIN_MENUBUTTON_RIGHT);
			
			if(ButtonVal==1)
			{
				Count ++;
			}
			if(ButtonVal==2)
			{
				Count --;
			}
			
			SwitchButtonPins = 0;

			if(Count >= 3 ) break;
			
			// Switch buttons
			if(Count <= -3 )
			{
				EEPROMWriteInt(18,23);
				SwitchButtonPins = 23;
				break;
			}
		  }

		  
	  }
	  // Go into menu
	 
	  Nicken(true,1);
	  Blinky();

	  MenuStatus = 1;
  }

  if(SwitchButtonPins == 23)
  {
	  int tmp = PIN_MENUBUTTON_LEFT;
	  PIN_MENUBUTTON_LEFT = PIN_MENUBUTTON_RIGHT;
	  PIN_MENUBUTTON_RIGHT = tmp;
  }

  analogReference(INTERNAL); // analogReference(EXTERNAL); // analogReference(DEFAULT);


}

void BlinkAsync(int CountSpeedOn,int CountSpeedOff)
{
	BlinkCounter++;
	if(BlinkCounter > (OnOff?CountSpeedOn:CountSpeedOff))
	{  
		digitalWrite(LED_PIN,OnOff?LOW:HIGH);									
		BlinkCounter = 0;
		OnOff = !OnOff;
	}
}

//int sensorValue_Up,sensorValue_Center,sensorValue_Down,sensorValue_Left,sensorValue_Right;

// Servo Positions (0-180)
int Pos_ServoBottomPrv, Pos_ServoMiddlePrv, Pos_ServoTopPrv; 

/// Sensor Values
int sensorValue_Up = 0 , sensorValue_Center = 0,sensorValue_Down =0 ,sensorValue_Left = 0,sensorValue_Right =0 ;

bool FlowerIsAwake = true;

int inSetup = 0;

int MaxValues[] = {
  0,0,0,0,0};

int tmpValues[5];

int ButtonCounter = 0;
bool ButtonRelease = true;


unsigned int BehaviorIndex = 0;

bool UseBufferedInput = true;
int agility = 2;

// New Switch speed 
int SwitchSpeed = 500;

void loop()
{

  if (millis() - prvMillis > FRAME_BREAK_MS ) 
  {
    prvMillis = millis();   

    if(CheckMenu())		return;


    if(SerialMenu()) return;

    struct SensorValues values = GetSensorData();

    // Debug Output of all Analog Values
    DebugOutput(values);

    // If all sensor values are belwo SENSOR_POWEROFFVOLTAGE all Servos will deattach!		
    CheckForSleepingInTheDark();

    if(UseBufferedInput)
    {			
      values = BufferAndSmoothValues(values);

      // Now primitivly overweite with new values
      // Read All Sesor Values
      sensorValue_Up = values.SensorUp ;    // read the input pin
      sensorValue_Center = values.SensorCenter;    // read the input pin
      sensorValue_Down = values.SensorDown;    // read the input pin
      sensorValue_Left = values.SensorLeft ;    // read the input pin
      sensorValue_Right = values.SensorRight ;    // read the input pin
    }

    switch (BehaviorIndex)
    {

		  case 0:	
			SwitchSpeed = 500;
			CalculateNewPositions(0.5f,2.0f,-1,false);
      break;		
		case 1:
			BlinkAsync(3,3);
			SwitchSpeed = 250;
		   CalculateNewPositions(1,1,14,false);			
		  break;			
		case 2:		
			digitalWrite(LED_PIN,HIGH);

		  CalculateNewPositions(0.8f,2.0f,-122,true);// le vamp
		  break;
		case 3:
		   BlinkAsync(2,30);
			SwitchSpeed = 350;
		   CalculateNewPositions(0.6f,2.0f,120,false);
		  break;
		case -1: // Reset loop
		  BehaviorIndex = 3;
		  break;
/*
    case 0:			
      CalculateNewPositions(0.5f,2.0f,-1,false);
      break;		
    case 1:
      CalculateNewPositions(0.4f,2.4f,2,false);					
      break;			
    case 2:		
      CalculateNewPositions(1,1,14,false);
      break;
    case 3:		
      CalculateNewPositions(0.6f,2.0f,120,false);
      break;
    case 4:
		BlinkAsync(2,100);
      CalculateNewPositions(0.8f,2.0f,-122,true);// le vamp
      break;

    case -1: // Reset loop
      BehaviorIndex = 4;
      break;
*/
    default: 
      BehaviorIndex = 0; // Reset bahavior
      break;
    }

    BottomServoSideChangingCheck();

    // Make the PowerSave to check if on of the 3 Servo can be switched OFF
    PowerSave();

    // Finaly set the current servo position
    ServoBottom.writeMicroseconds(Pos_ServoBottom);
    ServoMiddle.writeMicroseconds(Pos_ServoMiddle);                    
    ServoTop.writeMicroseconds(Pos_ServoTop);
  }

  //Servo::refresh();
}
////////////////////////////////////////////////////////////////////

unsigned long lastServoChange = 0;

void BottomServoSideChangingCheck()
{
  if(millis() - lastServoChange < 1750)
  {
    // No no change now
  }
  else
    if(Pos_ServoBottom < PWM_SERVO_MIN - 4)
    {		

      //Serial.println("Changing Sites High");        

      SaveDeAttach(1,true);		
      SaveDeAttach(2,true);	
      SaveDeAttach(3,true);	

      int temp_Pos_ServoBottom = PWM_SERVO_MAX  - 15 ; 
      int temp_Pos_ServoMiddle = PWM_SERVO_MAX - (Pos_ServoMiddle - PWM_SERVO_MIN);
      int temp_Pos_ServoTop = PWM_SERVO_MAX - (Pos_ServoTop - PWM_SERVO_MIN);

      MoveToPositions(temp_Pos_ServoBottom,temp_Pos_ServoMiddle,temp_Pos_ServoTop,SwitchSpeed,SENSOR_LEFT);		
    }
    else
      if(Pos_ServoBottom > PWM_SERVO_MAX + 4)
      {
        //Serial.println("Changing Sites Low"); 

        SaveDeAttach(1,true);		
        SaveDeAttach(2,true);	
        SaveDeAttach(3,true);	

        int temp_Pos_ServoBottom = PWM_SERVO_MIN + 15;
        int temp_Pos_ServoMiddle = PWM_SERVO_MAX - (Pos_ServoMiddle + Offset_ServoMiddle - PWM_SERVO_MIN);
        int temp_Pos_ServoTop = PWM_SERVO_MAX - (Pos_ServoTop - Offset_ServoTop - PWM_SERVO_MIN);

        MoveToPositions(temp_Pos_ServoBottom,temp_Pos_ServoMiddle,temp_Pos_ServoTop,SwitchSpeed,SENSOR_RIGHT);
      }

  // Finally value Limiting
  if(Pos_ServoBottom < PWM_SERVO_MIN)Pos_ServoBottom = PWM_SERVO_MIN;
  if(Pos_ServoBottom > PWM_SERVO_MAX)Pos_ServoBottom = PWM_SERVO_MAX;

  if(Pos_ServoMiddle < PWM_SERVO_MIN)Pos_ServoMiddle = PWM_SERVO_MIN;                                                                                                                                                                                                                                   
  if(Pos_ServoMiddle > PWM_SERVO_MAX)Pos_ServoMiddle = PWM_SERVO_MAX;

  if(Pos_ServoTop < PWM_SERVO_MIN)Pos_ServoTop=PWM_SERVO_MIN;
  if(Pos_ServoTop > PWM_SERVO_MAX)Pos_ServoTop=PWM_SERVO_MAX;
}
////////////////////////////////////////////////////////////

void MoveToPositions(int temp_Pos_ServoBottom,int temp_Pos_ServoMiddle,int temp_Pos_ServoTop, int delayPerStepMicroSeconds,int PinID)
{
	Blinky(1,40);

  int TempValue = analogRead(PinID);
  delayMicroseconds(15);
  TempValue += analogRead(PinID);
  delayMicroseconds(36);
  TempValue += analogRead(PinID);
  TempValue/=3;
  int maxValueToGo = TempValue + 15;

  int InterceptLock = 0;

  while(true)
  {				
    if(temp_Pos_ServoBottom < Pos_ServoBottom) Pos_ServoBottom--;
    if(temp_Pos_ServoBottom > Pos_ServoBottom) Pos_ServoBottom++;

    if(temp_Pos_ServoTop < Pos_ServoTop) Pos_ServoTop--;
    if(temp_Pos_ServoTop > Pos_ServoTop) Pos_ServoTop++;

    if(temp_Pos_ServoMiddle < Pos_ServoMiddle) Pos_ServoMiddle--;
    if(temp_Pos_ServoMiddle > Pos_ServoMiddle) Pos_ServoMiddle++;

    ServoBottom.writeMicroseconds(Pos_ServoBottom);				
    ServoMiddle.writeMicroseconds(Pos_ServoMiddle);   				
    ServoTop.writeMicroseconds(Pos_ServoTop);		

    if(Pos_ServoBottom == temp_Pos_ServoBottom &&
      Pos_ServoMiddle == temp_Pos_ServoMiddle &&
      Pos_ServoTop == temp_Pos_ServoTop)
    {
      break;
    }
    delayMicroseconds(delayPerStepMicroSeconds);

    TempValue = analogRead(PinID);
    delayMicroseconds(15);
    TempValue += analogRead(PinID);
    delayMicroseconds(36);
    TempValue += analogRead(PinID);
    TempValue/=3;

    // if it found a brighter light source it will stop the switching mo
	InterceptLock++;
    if(TempValue>maxValueToGo && InterceptLock > 700)
    {
      //Serial.print("Intercepted!\n");
      break;
    }
  }

  // This should prevend the next movement 
  lastServoChange = millis();
}

//////////////////////delay////////////////////////
/* For Software Servo Lib Only
 void delay(int DelayMiliSec)
 {
 	unsigned long startTime = millis();
 	
 	while(startTime + DelayMiliSec > millis())
 	{	
 		delay(1);
 		Servo::refresh();
 	}
 }
 */
//////////////////// POWER SAVE SERVO ///////////////////////////////
unsigned int deltaMovement = 1;
unsigned int deltaTime = 300;

long timeCounterTop,timeCounterMiddle,timeCounterBottom;
int prvServoPosTop,prvServoPosMiddle,prvServoPosBottom;
boolean servo_Top_attached = false, servo_Middle_attached = false, servo_Bottom_attached = false;

void PowerSave()
{
  // cheap disable return;
  if(TestPowerSave(&prvServoPosTop,&Pos_ServoTop,&timeCounterTop))
    SaveDeAttach(1,true);
  else 
    SaveDeAttach(1,false);

  if(TestPowerSave(&prvServoPosMiddle,&Pos_ServoMiddle,&timeCounterMiddle))
    SaveDeAttach(2,true);
  else 
    SaveDeAttach(2,false);

  if(TestPowerSave(&prvServoPosBottom,&Pos_ServoBottom,&timeCounterBottom))
    SaveDeAttach(3,true);
  else 
    SaveDeAttach(3,false);
}

void SaveDeAttach(int ServoNrTopMiddleBottom,bool On)
{
  // cheap disable return;

  switch(ServoNrTopMiddleBottom)
  {
  case 1:
    if(On)
    {
      if(!servo_Top_attached)
      {
        ServoTop.attach(PIN_SERVOTOP);
        servo_Top_attached = true;
        //Serial.println(" Top Servo on ");
      }
    }
    else  if(servo_Top_attached){ 
      ServoTop.detach();
      servo_Top_attached = false;  
      //Serial.println(" Top Servo off "); 
    }

    break;

  case 2:
    if(On)
    {
      if(!servo_Middle_attached)
      {
        ServoMiddle.attach(PIN_SERVOMIDDLE);
        servo_Middle_attached = true;
        //Serial.println(" Middle Servo on ");
      }
    }
    else  if(servo_Middle_attached){ 
      ServoMiddle.detach();
      servo_Middle_attached = false;
      //Serial.println(" Middle Servo off ");
    }

    break;
  case 3:
    if(On)
    {
      if(!servo_Bottom_attached)
      {
        ServoBottom.attach(PIN_SERVOBOTTOM);
        servo_Bottom_attached = true;
        //Serial.println(" Bottom Servo on ");
      }
    }
    else  if(servo_Bottom_attached){ 
      ServoBottom.detach();
      servo_Bottom_attached = false;
      //Serial.println(" Bottom Servo off ");
    }

    break;
  default:
    break;
  }
}

boolean TestPowerSave(int* prvPos,int* nowPos,long* timeCounter)
{
  if(*nowPos >= *prvPos + (int)deltaMovement || *nowPos <= *prvPos - (int)deltaMovement)
  {
    *prvPos = *nowPos;
    *timeCounter = millis();
    return true;   
  }
  else
    if(millis() - *timeCounter > deltaTime)
    {
      *nowPos = *prvPos;
      return false; 
    }

  return true;
}
/////////////////////////////////////////////////////////////////////
///////////////// Debug Output //////////////////////////////////////
int DelayCounter = 0;
void DebugOutput(struct SensorValues Sensor)
{
  // Debug OutPut
  if(DelayCounter > 20)
  {
    DelayCounter = 0;
    //Serial.print(BehaviorIndex); 
    //Serial.print(" -:- ");

    //Serial.print(Sensor.SensorUp); 
    //Serial.print(" U ");     // debug value
    //Serial.print(Sensor.SensorCenter); 
    //Serial.print(" C ");     // debug value
    //Serial.print(Sensor.SensorDown); 
    //Serial.print(" D ");     // debug value
    //Serial.print(Sensor.SensorLeft); 
    //Serial.print(" L ");     // debug value
    //Serial.print(Sensor.SensorRight); 
    //Serial.print(" R\n");     // debug value

    //Serial.print(Pos_ServoBottom); 
    //Serial.print(" B ");     // debug value
    //Serial.print(Pos_ServoMiddle); 
    //Serial.print(" M ");     // debug value
    //Serial.print(Pos_ServoTop); 
    //Serial.println(" T ");     // debug value

  }
  else DelayCounter++;
}

bool CheckForSleepingInTheDark()
{
  // cheap disable return true;
  if(		 sensorValue_Center < SENSOR_POWEROFFVALUE
    &&   sensorValue_Down < SENSOR_POWEROFFVALUE
    &&   sensorValue_Left < SENSOR_POWEROFFVALUE
    &&   sensorValue_Right < SENSOR_POWEROFFVALUE
    &&   sensorValue_Up < SENSOR_POWEROFFVALUE)
  {
    if(FlowerIsAwake)
    {
      SaveDeAttach(1,false);
      SaveDeAttach(2,false);
      SaveDeAttach(3,false);
    }
    FlowerIsAwake = false;
    return false;
  }
  else
  {
    if(!FlowerIsAwake)
    {
      SaveDeAttach(1,true);
      SaveDeAttach(2,true);
      SaveDeAttach(3,true);
    }
    FlowerIsAwake = true;
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

struct SensorValues GetSensorData()
{
  SensorValues sensInput;

  sensInput.SensorUp  = analogRead(SENSOR_UP) + SENSOR_UP_OFFSET ;    // read the input pin
  sensInput.SensorCenter  = analogRead(SENSOR_CENTER) + SENSOR_CENTER_OFFSET ;    // read the input pin
  sensInput.SensorDown  = analogRead(SENSOR_DOWN) + SENSOR_DOWN_OFFSET ;    // read the input pin
  sensInput.SensorLeft  = analogRead(SENSOR_LEFT) + SENSOR_LEFT_OFFSET ;    // read the input pin
  sensInput.SensorRight  = analogRead(SENSOR_RIGHT) + SENSOR_RIGHT_OFFSET  ;    // read the input pin

  return sensInput;
}
/* // alt for sensor reading
 void GetSensorData(struct SensorValues *fill)
 {
 	if(fill==NULL) return;
 	
 	fill->SensorUp  = analogRead(SENSOR_UP) + SENSOR_UP_OFFSET ;    // read the input pin
 	fill->SensorCenter  = analogRead(SENSOR_CENTER) + SENSOR_CENTER_OFFSET ;    // read the input pin
 	fill->SensorDown  = analogRead(SENSOR_DOWN) + SENSOR_DOWN_OFFSET ;    // read the input pin
 	fill->SensorLeft  = analogRead(SENSOR_LEFT) + SENSOR_LEFT_OFFSET ;    // read the input pin
 	fill->SensorRight  = analogRead(SENSOR_RIGHT) + SENSOR_RIGHT_OFFSET  ;    // read the input pin
 }
 */

#define INPUT_BUFFER_SIZE		10

struct SensorValues InputBuffer[INPUT_BUFFER_SIZE];
int BufferPointer = -1;

bool stripMinMax = true;
// This Methode buffers and smoothes the input value;
// Round Robin Buffer size is defined in INPUT_BUFFER_SIZE
// Min and Max Values are stiped when stripMinMax is true
struct SensorValues BufferAndSmoothValues(const struct SensorValues input)
{		
  if(BufferPointer == -1)
  {
    memset(InputBuffer,0,sizeof(SensorValues) * INPUT_BUFFER_SIZE);
    BufferPointer = 0;
  }

  InputBuffer[BufferPointer] = input; // if this does not work then make a copying in old primitive way :: InputBuffer[BufferPointer].SensorCenter = input.SensorCenter;
  BufferPointer = (BufferPointer + 1) % INPUT_BUFFER_SIZE; // Round Robine Buffer

  delayMicroseconds(133);
  InputBuffer[BufferPointer] = GetSensorData(); // if this does not work then make a copying in old primitive way :: InputBuffer[BufferPointer].SensorCenter = input.SensorCenter;
  BufferPointer = (BufferPointer + 1) % INPUT_BUFFER_SIZE; // Round Robine Buffer
  delayMicroseconds(33);
  InputBuffer[BufferPointer] = GetSensorData(); // if this does not work then make a copying in old primitive way :: InputBuffer[BufferPointer].SensorCenter = input.SensorCenter;
  BufferPointer = (BufferPointer + 1) % INPUT_BUFFER_SIZE; // Round Robine Buffer


  // Now calculate mean and remove peaks
  struct SensorValues minPeak;
  struct SensorValues maxPeak;
  struct SensorValues SumValue; 
  memset(&SumValue,0,sizeof(SensorValues));

  // Sumup and collect min max
  for(int i = 0; i < INPUT_BUFFER_SIZE; i++)
  {
    if(stripMinMax)
    {
      if(i == 0) {
        minPeak = InputBuffer[i];
        maxPeak = InputBuffer[i];
      }
      else
      {
        // Compare sensor (up,down,left,right,center) with all the values!
        // In a min max way;

        // Center
        if(minPeak.SensorCenter > input.SensorCenter) minPeak.SensorCenter = input.SensorCenter;
        if(maxPeak.SensorCenter < input.SensorCenter) maxPeak.SensorCenter = input.SensorCenter;
        // SensorDown
        if(minPeak.SensorDown > input.SensorDown) minPeak.SensorDown = input.SensorDown;
        if(maxPeak.SensorDown < input.SensorDown) maxPeak.SensorDown = input.SensorDown;
        // Left
        if(minPeak.SensorLeft > input.SensorLeft) minPeak.SensorLeft = input.SensorLeft;
        if(maxPeak.SensorLeft < input.SensorLeft) maxPeak.SensorLeft = input.SensorLeft;
        // Rigth
        if(minPeak.SensorRight > input.SensorRight) minPeak.SensorRight = input.SensorRight;
        if(maxPeak.SensorRight < input.SensorRight) maxPeak.SensorRight = input.SensorRight;
        // UP
        if(minPeak.SensorUp > input.SensorUp) minPeak.SensorUp = input.SensorUp;
        if(maxPeak.SensorUp < input.SensorUp) maxPeak.SensorUp = input.SensorUp;

      }
    }

    SumValue.SensorCenter += InputBuffer[i].SensorCenter;
    SumValue.SensorDown   += InputBuffer[i].SensorDown;
    SumValue.SensorLeft   += InputBuffer[i].SensorLeft;
    SumValue.SensorRight  += InputBuffer[i].SensorRight;
    SumValue.SensorUp     += InputBuffer[i].SensorUp;
  }

  int NumberOfDataPoints = INPUT_BUFFER_SIZE;
  //Strip Min Max Value;
  if(stripMinMax)
  {
    SumValue.SensorCenter -= minPeak.SensorCenter + maxPeak.SensorCenter ;
    SumValue.SensorDown   -= minPeak.SensorDown   + maxPeak.SensorDown ;
    SumValue.SensorLeft   -= minPeak.SensorLeft   + maxPeak.SensorLeft;
    SumValue.SensorRight  -= minPeak.SensorRight  + maxPeak.SensorRight ;
    SumValue.SensorUp     -= minPeak.SensorUp     + maxPeak.SensorUp ;

    NumberOfDataPoints -= 2;
  }

  //Calculate Mean
  SumValue.SensorCenter /= NumberOfDataPoints; 
  SumValue.SensorDown   /= NumberOfDataPoints;
  SumValue.SensorLeft   /= NumberOfDataPoints;
  SumValue.SensorRight  /= NumberOfDataPoints;
  SumValue.SensorUp     /= NumberOfDataPoints;

  return SumValue;
}

/// This returns an int value if an button has been pressed
/// 0 ... No button
/// 1 ... Left button pressed short
/// 2 ... Right button pressed short
/// 3 ... Both button pressed short
/// 4 ... Left button pressed long
/// 5 ... Right button pressed long
/// 6 ... Both button pressed long
/// 11 .. Still on Left Button
/// 12 .. Still on Right Button
/// 13 .. Still Both Buttons pressed
/// 14 ... Reached Right button pressed long
/// 15 ... Reached Left button pressed long
/// 16 ... Reached Both button pressed long


int ButtonBufferLeft  = 0;
int ButtonBufferRight = 0;
int ButtonBufferLeftOff  = 0;
int ButtonBufferRightOff = 0;

int ButtonStateLeft  = 0;
int ButtonStateRight = 0;

void ButtonBufferReset()
{
 ButtonBufferLeft  = 0;
 ButtonBufferRight = 0;
 ButtonBufferLeftOff  = 0;
 ButtonBufferRightOff = 0;

 ButtonStateLeft  = 0;
 ButtonStateRight = 0;
}

int ButtonCheck(int ButtonPinLeft,int ButtonPinRight)
{
  /// Button press short 0.125sec on 0.075sec off
  /// Button press long 4sec on 0.3sec off
  bool LeftValue =  !digitalRead(ButtonPinLeft);
  bool RightValue = !digitalRead(ButtonPinRight);

#define MIN_BUTTON_COUNT      (20 / FRAME_BREAK_MS + 1)
#define MID_BUTTON_COUNT	  (40 / FRAME_BREAK_MS + 1)
#define MAX_BUTTON_COUNT	  (1000 / FRAME_BREAK_MS + 1)

  int Res = 0;
  //////////// LEFT BUTTON //////////////////////////////////////////

  if(LeftValue) {
    ButtonBufferLeft++; 
    ButtonBufferLeftOff = 0; 
    Res = 11;
  }
  else {
    ButtonBufferLeft--; 
    ButtonBufferLeftOff++;
  }

  if(ButtonStateLeft > 0 ) 
    if(!LeftValue) ButtonBufferLeftOff++;
    else ButtonBufferLeftOff--;


  if(ButtonBufferLeft > MID_BUTTON_COUNT && ButtonStateLeft == 0) ButtonStateLeft = 1;
  if(ButtonBufferLeft > MAX_BUTTON_COUNT && ButtonStateLeft == 1) {
    ButtonStateLeft = 2; 
    Res = 14;
  }

  if(ButtonBufferLeftOff > MIN_BUTTON_COUNT && ButtonStateLeft == 1) 
  {		
    // short pressed and released
    ButtonBufferLeftOff = 0; 
    ButtonBufferLeft = 0; 
    ButtonStateLeft = 0;

    Res = 1;
  }

  if(ButtonBufferLeftOff  > MID_BUTTON_COUNT && ButtonStateLeft == 2 && ButtonBufferRight < MID_BUTTON_COUNT) 
  {
    // long pressed and released
    ButtonBufferLeftOff = 0; 
    ButtonBufferLeft = 0; 
    ButtonStateLeft = 0;

    Res = 4;
  }

  if(ButtonBufferLeft < 0) ButtonBufferLeft = 0; 
  if(ButtonBufferLeft > MAX_BUTTON_COUNT + 2) ButtonBufferLeft = MAX_BUTTON_COUNT;
  if(ButtonBufferLeftOff > MAX_BUTTON_COUNT + 2) ButtonBufferLeftOff = MAX_BUTTON_COUNT;

  //////////// RIGHT BUTTON /////////////////////////////////////////

  if(RightValue) {
    ButtonBufferRight++; 
    ButtonBufferRightOff = 0; 
    Res = Res==11?13: 12;
  }
  else {
    ButtonBufferRight--; 
  }

  if(ButtonStateRight > 0 ) 
    if(!RightValue) ButtonBufferRightOff++;
    else ButtonBufferRightOff--;


  if(ButtonBufferRight > MID_BUTTON_COUNT && ButtonStateRight == 0) ButtonStateRight = 1;
  if(ButtonBufferRight > MAX_BUTTON_COUNT && ButtonStateRight == 1) {
    ButtonStateRight = 2; 
    Res = 15;
  }

  if(ButtonBufferRightOff > MIN_BUTTON_COUNT && ButtonStateRight == 1) 
  {		
    // short pressed and released
    ButtonBufferRightOff = 0; 
    ButtonBufferRight = 0; 
    ButtonStateRight = 0;

    Res = Res == 1 ?3: 2;
  }

  if(ButtonBufferRightOff > MID_BUTTON_COUNT && ButtonStateRight == 2 && ButtonBufferLeft < MID_BUTTON_COUNT) 
  {
    // long pressed and released
    ButtonBufferRightOff = 0; 
    ButtonBufferRight = 0; 
    ButtonStateRight = 0;
    Res = Res == 4 ? 6: 5;
  }

  if(ButtonBufferRight >= MAX_BUTTON_COUNT && ButtonBufferLeft >= MAX_BUTTON_COUNT) Res = 16;

  if(ButtonBufferRight < 0) ButtonBufferRight = 0; 
  if(ButtonBufferRight > MAX_BUTTON_COUNT + 2) ButtonBufferRight = MAX_BUTTON_COUNT;
  if(ButtonBufferRightOff > MAX_BUTTON_COUNT + 2) ButtonBufferRightOff = MAX_BUTTON_COUNT;

  return Res;
}


int ServoSelection = 0;


bool CheckMenu()
{
  int ButtonVal = ButtonCheck(PIN_MENUBUTTON_LEFT,PIN_MENUBUTTON_RIGHT);
  ////Serial.println(ButtonVal);

  //// BEHAVOIR SWITCHING ////////////////////////////////////////////////////////////////////////////
  if(MenuStatus == 0 && ButtonVal == 1)
  { 
    BehaviorIndex++;
    //Debug Output
    //Serial.println("Behavior switched! (up)");
    // Switch to next behavior
	Blinky(1,50);// Bugy
  }
  else if(MenuStatus == 0 && ButtonVal == 2)
  {
    BehaviorIndex--;
    //Switch to previous behavior
    //Serial.println("Behavior switched! (down)");
	Blinky(1,50);// Bugy
  }

  //// SENSOR CALIBRATION MODE ////
  if(MenuStatus == 0 && (ButtonVal == 5 || ButtonVal == 15))
    //if(MenuStatus == 0 && ButtonVal == 16)
  {
    // Almost in Menu
    MenuStatus = 1;
    // Menu Blinking

  }
  else if(MenuStatus == 1 && ButtonVal == 0)	
  {
    //SaveDeAttach(1,true);
    //SaveDeAttach(2,true);
    //SaveDeAttach(3,true);
    // Now in Menu Mode
    MenuStatus = 2;    
    // Debug output (serial)
    //Serial.println("setup called!");

    SlowlyGetAllToMiddlePosition();
	// Menu Blinking
    Blinky(10,50);

	AllServosOff(); // Dont need to be on

    for(int i = 0; i < 5;i++)
    {
      MaxValues[i] = 0;
    }

    //Serial.println("alles auf 90 grad");
	ButtonBufferReset();

  }
  else if(MenuStatus == 2)
  {
	 BlinkAsync(40,10);
    for(int i = 0; i < 40 ; i++)
    {
      // Collect Max Values
      tmpValues[0] = analogRead(SENSOR_CENTER);
      tmpValues[1] = analogRead(SENSOR_UP);
      tmpValues[2] = analogRead(SENSOR_DOWN);
      tmpValues[3] = analogRead(SENSOR_LEFT);
      tmpValues[4] = analogRead(SENSOR_RIGHT);

      for(int i = 0; i < 5;i++)
      {
        if(tmpValues[i] > MaxValues[i] && (tmpValues[i]<1020))  MaxValues[i] = tmpValues[i];
      }
    }

    //Serial.print("C "); 
    //Serial.print(MaxValues[0]);
    //Serial.print(" - ");
    //Serial.print(tmpValues[0]); 
    //Serial.print(" U ");     // debug value
    //Serial.print(MaxValues[1]);
    //Serial.print(" - ");
    //Serial.print(tmpValues[1]); 
    //Serial.print(" D ");     // debug value
    //Serial.print(MaxValues[2]);
    //Serial.print(" - ");
    //Serial.print(tmpValues[2]); 
    //Serial.print(" L ");     // debug value
    //Serial.print(MaxValues[3]);
    //Serial.print(" - ");
    //Serial.print(tmpValues[3]); 
    //Serial.print(" R ");     // debug value
    //Serial.print(MaxValues[4]);
    //Serial.print(" - ");
    //Serial.println(tmpValues[4]);   

    //if(ButtonVal == 5 || ButtonVal == 16) MenuStatus = 3;
	if( ButtonVal == 16) MenuStatus = 3;
    // Back
	if(ButtonVal == 3 || ButtonVal == 5 )
	{	
		MenuStatus = 0;
		ButtonVal= 0;
		Blinky(1,100);// Bugy
		
		Nicken(false,1);		
	}

  }
  else if(MenuStatus == 3 && ButtonVal == 0)	
  {
    // Done with finding max values 
    MenuStatus = 0;

    // Menu Blinking 
    //Serial.println("save values to eeprom"); 		
    Blinky();	

    // Debug output (serial)
    //Serial.print("MaxValues C ");
    //Serial.print(MaxValues[0] ); 
    //Serial.print(" U ");     // debug value
    //Serial.print(MaxValues[1]); 
    //Serial.print(" D ");     // debug value
    //Serial.print(MaxValues[2]); 
    //Serial.print(" L ");     // debug value
    //Serial.print(MaxValues[3]); 
    //Serial.print(" R ");     // debug value
    //Serial.println(MaxValues[4]);    

    // ReCalculate Offsets
    SENSOR_CENTER_OFFSET = - 5;
    SENSOR_UP_OFFSET = MaxValues[0] - MaxValues[1];
    SENSOR_DOWN_OFFSET = MaxValues[0] - MaxValues[2];
    SENSOR_LEFT_OFFSET = MaxValues[0] - MaxValues[3];
    SENSOR_RIGHT_OFFSET = MaxValues[0] - MaxValues[4];
    // Save Max values
    SaveSensorValues();
	EEPROMWriteInt(14,42);

	Blinky(8,50);

	Nicken(true,1);
    ////Serial.println("done.");		
  }

  //// 90 DEGREE CALIBRATION MODE ////////////////////////////////////////////////////////////////////////////////////
  if(MenuStatus == 0 && (ButtonVal == 4 || ButtonVal == 14))
  {
    // Almost in Menu
    MenuStatus = 4;
    // Menu Blinking

  }
  else if(MenuStatus == 4 && ButtonVal == 0)	
  {
    // Now in Menu Mode
    MenuStatus = 5;
    // Menu Blinking
    //Serial.println("setup 90degree cal called!");
    SlowlyGetAllToMiddlePosition();

    // Debug output (serial)
	ButtonBufferReset();

  }
  else if(MenuStatus == 5)
  {
	 BlinkAsync(3,30);
    // Navigate thru the servos
    if(ButtonVal == 1) // Left Button Increase Offset
    {
      if(ServoSelection==0) // Middle Servo 
      {
        Offset_ServoMiddle  += 2;// I add 2 cause micro seconds are so small steps (will take long too)
        //Serial.print("Servos Offset Middle +: ");
        //Serial.println(Offset_ServoMiddle); 
      }
      else
      {	// Top servo
        Offset_ServoTop     -= 2;
        //Serial.print("Servos Offset Top +: ");
        //Serial.println(Offset_ServoTop); 
      }

    }
    if(ButtonVal == 2) //Right Button Decrease Offset
    {
      if(ServoSelection==0) // Middle Servo 
      {
        Offset_ServoMiddle  -= 2;// I add 2 cause micro seconds are so small steps (will take long too)
        //Serial.print("Servos Offset Middle -: ");
        //Serial.println(Offset_ServoMiddle); 
      }
      else
      {	// Top servo
        Offset_ServoTop     += 2;
        //Serial.print("Servos Offset Top -: ");
        //Serial.println(Offset_ServoTop); 
      }	 
    }

    //if(ButtonVal == 4 || ButtonVal == 5)// Switch between Servo Top and Middle
	if(ButtonVal == 4)// Switch between Servo Top and Middle
    {
      ServoSelection = (ServoSelection==0)?1:0;
    }
	
	Pos_ServoTop = PWM_SERVO_MID + Offset_ServoTop;
	Pos_ServoMiddle = PWM_SERVO_MID + Offset_ServoMiddle;
	
	PowerSave();

    // Get Servos to position
    ServoTop.writeMicroseconds(Pos_ServoTop);
    ServoMiddle.writeMicroseconds(Pos_ServoMiddle);

    if(ButtonVal == 16) MenuStatus = 6; // Done and ready for saving

	  if(ButtonVal == 3 || ButtonVal == 5 )
	  {		
		  ButtonVal = 1;
		  MenuStatus = 0; // Done and ready for saving
		  //Blinky(1,100);// Bugy
		  Nicken(false,1);
	  }

  }
  else if(MenuStatus == 6 && ButtonVal == 0)	
  {	 
    // Done with finding max values 
    MenuStatus = 0;
    // Menu Blinking 
    //Serial.println("save 90 offset values to eeprom"); 		
    Blinky(); 
    // Debug output (serial)
    // Save Servo Offset values
    SaveOffsetValues();
    //Serial.println("done"); 	
	 Nicken(true,1);

  }

  if(MenuStatus == 7  && ButtonVal == 0)
  {
	 MenuStatus=0;
  }

  return (bool)(MenuStatus == 0 ? false : true);
}
/*
void SinusDance()
{
	
	double SinVal = sin( (double) millis()/ 1500.0f);
	
	Pos_ServoBottom = PWM_SERVO_MIN + (double)((PWM_SERVO_MAX - PWM_SERVO_MIN)/2.0f) * (double)(SinVal + 1.0f);
	Pos_ServoMiddle = Pos_ServoBottom;// (PWM_SERVO_MIN + (PWM_SERVO_MAX - PWM_SERVO_MIN)/2 * (SinVal +1));
	Pos_ServoTop = Pos_ServoBottom;//(PWM_SERVO_MIN + (PWM_SERVO_MAX - PWM_SERVO_MIN)/2 * (SinVal +1));
	
}
*/
void AllServosOff()
{
	SaveDeAttach(1,false);
	SaveDeAttach(2,false);
	SaveDeAttach(3,false);
}

void Nicken(bool Yes, int Count)
{
	SlowlyGetAllToMiddlePosition();

	int delayMicron = 500;
	for(int j = 0; j < Count;j++)
	{
		for(int i = 0; i< (PWM_SERVO_MID - PWM_SERVO_MIN)/2;i++)
		{
			ServoTop.writeMicroseconds(Pos_ServoTop-- + Offset_ServoTop);
			ServoMiddle.writeMicroseconds(Pos_ServoMiddle++ + Offset_ServoMiddle);
			delayMicroseconds(delayMicron);
		}

		delay(200);

		if(!Yes)
		{
			for(;j < Count;j++)
			{
				for(int i = 0; i< (PWM_SERVO_MID - PWM_SERVO_MIN)/2;i++)
				{
					ServoBottom.writeMicroseconds(Pos_ServoBottom--);	
					delayMicroseconds(delayMicron);
				}
				for(int i = 0; i< (PWM_SERVO_MID - PWM_SERVO_MIN);i++)
				{
					ServoBottom.writeMicroseconds(Pos_ServoBottom++);	
					delayMicroseconds(delayMicron);
				}
				for(int i = 0; i< (PWM_SERVO_MID - PWM_SERVO_MIN)/2;i++)
				{
					ServoBottom.writeMicroseconds(Pos_ServoBottom--);	
					delayMicroseconds(delayMicron);
				}
			}
		}

		for(int i = 0; i< (PWM_SERVO_MID - PWM_SERVO_MIN)/2;i++)
		{
			ServoTop.writeMicroseconds(Pos_ServoTop++ + Offset_ServoTop);
			ServoMiddle.writeMicroseconds(Pos_ServoMiddle-- + Offset_ServoMiddle);
			delayMicroseconds(delayMicron);
		}	
	}
	
	
	ButtonBufferReset();
}

void Blinky()
{
  digitalWrite(LED_PIN,HIGH);	
  delay(250);	
  digitalWrite(LED_PIN,LOW);
  delay(500);
  digitalWrite(LED_PIN,HIGH);
  delay(250);
  digitalWrite(LED_PIN,LOW); 
}

void Blinky(int BlinkCounter,int msSpeed)
{	
	digitalWrite(LED_PIN,LOW);	
	for(int i = 0; i< BlinkCounter; i++)
	{
		delay(msSpeed);
		digitalWrite(LED_PIN,HIGH);
		delay(msSpeed);
		digitalWrite(LED_PIN,LOW);
	}
}


void SaveOffsetValues()
{
  EEPROMWriteInt(10,Offset_ServoMiddle+1400); // dirty offset
  EEPROMWriteInt(12,Offset_ServoTop+1400); // dirty offset
}

void SaveSensorValues()
{
  EEPROMWriteInt(0,SENSOR_CENTER_OFFSET+400); // dirty offset
  EEPROMWriteInt(2,SENSOR_UP_OFFSET+400); // dirty offset
  EEPROMWriteInt(4,SENSOR_DOWN_OFFSET+400); // dirty offset
  EEPROMWriteInt(6,SENSOR_LEFT_OFFSET+400); // dirty offset
  EEPROMWriteInt(8,SENSOR_RIGHT_OFFSET+400); // dirty offset
}
// power consumption 

int SerialMenuMode = 0;

/**
 * Added Serial Menu
 * 
 * q .. a      Left Sensor + -
 * w .. s      Center Sensor + -
 * e .. d      Right Sensor + -
 * r .. f      Up Sensor + -
 * t .. g      Down Sensor + -
 * 
 * u .. j      Middle Servo + -
 * i .. k      Top Servo + -
 * 
 * p .. save all values to epprom
 * c .. cancel from 90 degree mode
 */
bool SerialMenu()
{
  // TODO Serial Feedback on commands

  if(Serial.available()<1) // No input
  {
    if (SerialMenuMode==0) return false;
    else return true;		
  }

  char Input = Serial.read();

  switch(Input)
  {
  case 'q':
    SENSOR_LEFT_OFFSET ++;
    break;
  case 'a':
    SENSOR_LEFT_OFFSET --;
    break;

  case 'w':
    SENSOR_CENTER_OFFSET ++;
    break;
  case 's':
    SENSOR_CENTER_OFFSET --;
    break;

  case 'e':
    SENSOR_RIGHT_OFFSET ++;
    break;
  case 'd':
    SENSOR_RIGHT_OFFSET --;
    break;

  case 'r':
    SENSOR_UP_OFFSET ++;
    break;
  case 'f':
    SENSOR_UP_OFFSET --;
    break;

  case 't':
    SENSOR_DOWN_OFFSET ++;
    break;
  case 'g':
    SENSOR_DOWN_OFFSET --;
    break;

  case 'u':
    SerialMenuMode = 1;
    Offset_ServoMiddle ++;
    break;
  case 'j':
    SerialMenuMode = 1;
    Offset_ServoMiddle --;

  case 'i':
    SerialMenuMode = 1;
    Offset_ServoTop ++;
    break;
  case 'k':
    SerialMenuMode = 1;
    Offset_ServoTop--;
    break;

  case 'p':// persist = save all values
    SerialMenuMode = 0;
    SaveOffsetValues();
    SaveSensorValues();

  case 'c':// cancel
    SerialMenuMode = 0;
    break;

  default:
    break;
  }

  if(SerialMenuMode == 1)
  {
    SerialMenuMode = 2;

    SlowlyGetAllToMiddlePosition();
  }
  if(SerialMenuMode==2)
  {
    ServoTop.writeMicroseconds(PWM_SERVO_MID + Offset_ServoTop);

    ServoMiddle.writeMicroseconds(PWM_SERVO_MID + Offset_ServoMiddle);
  }
  return SerialMenuMode==0?false:true;
}

void SlowlyGetAllToMiddlePosition()
{					
  SaveDeAttach(1,true);
  ServoTop.writeMicroseconds(PWM_SERVO_MID + Offset_ServoTop);
  delay(400);
  SaveDeAttach(2,true);	
  ServoMiddle.writeMicroseconds(PWM_SERVO_MID + Offset_ServoMiddle);
  delay(400);
  SaveDeAttach(3,true);	
  ServoBottom.writeMicroseconds(PWM_SERVO_MID);

  Pos_ServoBottom = PWM_SERVO_MID;
  Pos_ServoMiddle = PWM_SERVO_MID + Offset_ServoMiddle;
  Pos_ServoTop = PWM_SERVO_MID + Offset_ServoTop;
}

void CalculateNewPositions(float PowerOf,float Scaling,int agility,bool vamp)
{
  sensorValue_Center-=agility;

  if((!vamp && sensorValue_Up > sensorValue_Center)   || (vamp && sensorValue_Up < sensorValue_Center) )
  {
    // Move middel servos
    Pos_ServoMiddle+= (int)((float)pow((float)(sensorValue_Up - sensorValue_Center)*(vamp?-1.0f:1.0f) ,PowerOf) * PIN_SERVOMIDDLE_DIR * Scaling) ;
    // Moce top Servo
    Pos_ServoTop+= (int)((float)pow((float)(sensorValue_Up - sensorValue_Center)*(vamp?-1.0f:1.0f),PowerOf) * PIN_SERVOTOP_DIR * Scaling);
  }
  if((!vamp && sensorValue_Down > sensorValue_Center) || (vamp && sensorValue_Down < sensorValue_Center)) 
  {
    // Move middel servos
    Pos_ServoMiddle-= (int)((float)pow((float)(sensorValue_Down - sensorValue_Center)*(vamp?-1.0f:1.0f),PowerOf) * PIN_SERVOMIDDLE_DIR * Scaling) ;
    // Moce top Servo
    Pos_ServoTop-= (int)((float)pow((float)(sensorValue_Down - sensorValue_Center)*(vamp?-1.0f:1.0f),PowerOf) * PIN_SERVOTOP_DIR * Scaling);
  }	

  if((!vamp && sensorValue_Left > sensorValue_Center) || (vamp && sensorValue_Left < sensorValue_Center) ) Pos_ServoBottom-= (int)((float)(Pos_ServoMiddle +  (PWM_SERVO_MAX - Pos_ServoTop) +Offset_ServoMiddle+Offset_ServoTop >(PWM_SERVO_MIN + (PWM_SERVO_MAX - PWM_SERVO_MIN))?-pow((float)(sensorValue_Left-sensorValue_Center)*(vamp?-1.0f:1.0f),PowerOf):
  pow((float)(sensorValue_Left-sensorValue_Center)*(vamp?-1.0f:1.0f),PowerOf))   * PIN_SERVOBOTTOM_DIR *Scaling);
  if((!vamp && sensorValue_Right> sensorValue_Center) || (vamp && sensorValue_Right< sensorValue_Center) ) Pos_ServoBottom+= (int)((float)(Pos_ServoMiddle +  (PWM_SERVO_MAX-Pos_ServoTop) +Offset_ServoMiddle+Offset_ServoTop >(PWM_SERVO_MIN + (PWM_SERVO_MAX - PWM_SERVO_MIN))?-pow((float)(sensorValue_Right-sensorValue_Center)*(vamp?-1.0f:1.0f),PowerOf):
  pow((float)(sensorValue_Right-sensorValue_Center)*(vamp?-1.0f:1.0f),PowerOf)) * PIN_SERVOBOTTOM_DIR*Scaling);
}
