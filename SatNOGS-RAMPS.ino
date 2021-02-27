#define LCD 1

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <AccelStepper.h>

#ifdef LCD
#include <ClickEncoder.h>
#include <TimerOne.h>
#include <LiquidCrystal.h>
#endif


#define DIR_AZ A1 /*PIN for Azimuth Direction*/
#define STEP_AZ A0 /*PIN for Azimuth Steps*/
#define DIR_EL A7 /*PIN for Elevation Direction*/
#define STEP_EL A6 /*PIN for Elevation Steps*/

#define EN_AZ 38 /*PIN for Enable or Disable Stepper Motors*/
#define EN_EL A2 /*PIN for Enable or Disable Stepper Motors*/

//#define SPR 3200  /*Step Per Revolution (200*16 microsteps)*/
//#define RATIO 60 /*Gear ratio*/

#define SPRG 33 /* SPR * RATIO / 360 */ 

#define T_DELAY 60000 /*Time to disable the motors in millisecond*/

#define HOME_AZ 14 /*Homing switch for Azimuth*/
#define HOME_EL 15 /*Homing switch for Elevation*/

#define MAX_AZ_ANGLE 365 /*Maximum Angle of Azimuth for homing scanning*/
#define MAX_EL_ANGLE 180 /*Maximum Angle of Elevation for homing scanning*/

#define MIN_AZ_ANGLE -365
#define MIN_EL_ANGLE -180

#define MAX_SPEED 250
#define MAX_ACCELERATION 100 

#define MIN_PULSE_WIDTH 1000 /*in microsecond*/

#define DEFAULT_HOME_STATE HIGH /*Change to LOW according to Home sensor*/

#define HOME_DELAY 6000 /*Time for homing Decceleration in millisecond*/

#define BufferSize 256
#define BaudRate 19200

/* SMART LCD Controller Pin Defs */
#ifdef LCD

#define KILL_PIN 41 // STOP / KILL button  

#define LCD_PINS_RS 16 //[RAMPS14-SMART-ADAPTER]  
#define LCD_PINS_ENABLE 17 //[RAMPS14-SMART-ADAPTER]  
#define LCD_PINS_D4 23 //[RAMPS14-SMART-ADAPTER]  
#define LCD_PINS_D5 25 //[RAMPS14-SMART-ADAPTER]  
#define LCD_PINS_D6 27 //[RAMPS14-SMART-ADAPTER]  
#define LCD_PINS_D7 29 //[RAMPS14-SMART-ADAPTER]  

 //encoder pins  
#define BTN_EN1 31 //[RAMPS14-SMART-ADAPTER]  
#define BTN_EN2 33 //[RAMPS14-SMART-ADAPTER]  
#define BTN_ENC 35 //[RAMPS14-SMART-ADAPTER]  

#define LCD_CHARS   20
#define LCD_LINES    4

LiquidCrystal lcd(LCD_PINS_RS, LCD_PINS_ENABLE, LCD_PINS_D4, LCD_PINS_D5, LCD_PINS_D6, LCD_PINS_D7);

ClickEncoder *encoder;
long lastAZ, valueAZ, lastEL, valueEL, value;
bool currentPos; // false = AZ, true = EL
#endif


/*Global Variables*/
unsigned long t_DIS = 0; /*time to disable the Motors*/
/*Define a stepper and the pins it will use*/
AccelStepper AZstepper(1, STEP_AZ, DIR_AZ);
AccelStepper ELstepper(1, STEP_EL, DIR_EL);


#ifdef LCD
void timerIsr() {
  encoder->service();
}

void displayAzEl() {
  lcd.setCursor(0, 2);  
  lcd.print("Curr: ");
  lcd.print(step2deg(AZstepper.currentPosition()));
  lcd.print("   ");
  lcd.print(step2deg(ELstepper.currentPosition()));
  lcd.print("  ");  
}
#endif



void setup()
{  
  /*Change these to suit your stepper if you want*/
  AZstepper.setMaxSpeed(MAX_SPEED);
  AZstepper.setAcceleration(MAX_ACCELERATION);
  
  /*Change these to suit your stepper if you want*/
  ELstepper.setMaxSpeed(MAX_SPEED);
  ELstepper.setAcceleration(MAX_ACCELERATION);
  
  /*Set minimum pulse width*/
  AZstepper.setMinPulseWidth(MIN_PULSE_WIDTH);
  ELstepper.setMinPulseWidth(MIN_PULSE_WIDTH);

  /*Enable Motors*/
  pinMode(EN_AZ, OUTPUT);pinMode(EN_EL, OUTPUT);
  digitalWrite(EN_AZ, LOW);digitalWrite(EN_EL, LOW);
  
  AZstepper.setEnablePin(EN_AZ); 
  AZstepper.setPinsInverted(false, false, true); //invert logic of enable pin
  AZstepper.enableOutputs();

  /*Homing switch*/
  pinMode(HOME_AZ, INPUT_PULLUP);
  pinMode(HOME_EL, INPUT_PULLUP);
  
  /*Serial Communication*/
  Serial.begin(BaudRate); 
    
#ifdef LCD
  encoder = new ClickEncoder(BTN_EN2, BTN_EN1, BTN_ENC);
  encoder->setAccelerationEnabled(false);
  
  lcd.begin(LCD_CHARS, LCD_LINES);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("SATNOGS v3 RAMPS");
  lcd.setCursor(7,1);
  lcd.print("Az       El");
  lcd.setCursor(0,3);
  lcd.print(" Tgt: ");
  lcd.blink();
  displayAzEl();

  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr); 
  
  lastAZ = 0;
  lastEL = 0;
  value = 0;
  currentPos = false;
#endif
  /*Initial Homing*/
  Homing(deg2step(-MAX_AZ_ANGLE), deg2step(-MAX_EL_ANGLE));
  AZstepper.setCurrentPosition(0);
}

void loop()
{ 
  /*Define the steps*/
  static long AZstep = 0;
  static long ELstep = 0;
  /*Time Check*/
  if (t_DIS == 0)
    t_DIS = millis();

  /*Disable Motors*/
  if (AZstep == AZstepper.currentPosition() && ELstep == ELstepper.currentPosition() && millis()-t_DIS > T_DELAY) {
    digitalWrite(EN_AZ, HIGH);
    digitalWrite(EN_EL, HIGH);
  } else {
    digitalWrite(EN_AZ, LOW);
    digitalWrite(EN_EL, LOW);

  }
  /*Read the steps from serial*/
  cmd_proc(AZstep, ELstep);
  /*Move the Azimuth & Elevation Motor*/

#ifdef LCD
  // LCD: handle update valAZ valEL
  check_encoder(AZstep, ELstep);
  displayAzEl();
#endif

  stepper_move(AZstep, ELstep);
}

#ifdef LCD
void check_encoder(long &stepAZ, long &stepEL)
{  
    value = encoder->getValue();

    if(currentPos)
    {
      valueEL += value;
      
      if (valueEL != lastEL) {
        if((valueEL > MIN_EL_ANGLE) && (valueEL < MAX_EL_ANGLE))
        {
          lastEL = valueEL;
          
          stepEL = deg2step(valueEL);
          
          lcd.setCursor(14, 3);
          lcd.print("    ");
          lcd.setCursor(14, 3);
          lcd.print(valueEL);
        }
        else valueEL -= value;
      }
    }
    else
    {
        valueAZ += value;
  
        if (valueAZ != lastAZ) { 
          if ((valueAZ > MIN_AZ_ANGLE) && (valueAZ < MAX_AZ_ANGLE)) 
          {        
            lastAZ = valueAZ;

            stepAZ = deg2step(valueAZ);
            
            lcd.setCursor(6, 3);
            lcd.print("    ");
            lcd.setCursor(6,3);
            lcd.print(valueAZ);
          }
          else valueAZ -= value;
        }
    }
  
  ClickEncoder::Button b = encoder->getButton();
  
  if (b != ClickEncoder::Open) {
    switch (b) {
      case ClickEncoder::Pressed:
      case ClickEncoder::Clicked:
        currentPos = !currentPos;

        if(currentPos)
          lcd.setCursor(13,3);
        else lcd.setCursor(5,3);

        lcd.blink();
                  
        break;
    }
  }
}
#endif

/*Homing Function*/
void Homing(long AZsteps, long ELsteps)
{
  long value_Home_AZ = DEFAULT_HOME_STATE;
  long value_Home_EL = DEFAULT_HOME_STATE;
  boolean isHome_AZ = false;
  boolean isHome_EL = false;
  
  AZstepper.moveTo(AZsteps);
  ELstepper.moveTo(ELsteps);
  
  while(isHome_AZ == false || isHome_EL == false)
  {
    value_Home_AZ = digitalRead(HOME_AZ);
    value_Home_EL = digitalRead(HOME_EL);

    /*Change to LOW according to Home sensor*/
    if (value_Home_AZ == DEFAULT_HOME_STATE)
    {
      AZstepper.moveTo(AZstepper.currentPosition());
      isHome_AZ = true;
    }   
    
    /*Change to LOW according to Home sensor*/
    if (value_Home_EL == DEFAULT_HOME_STATE)
    {
      ELstepper.moveTo(ELstepper.currentPosition());
      isHome_EL = true;
    }
    
    if (AZstepper.distanceToGo() == 0 && !isHome_AZ)
    {
      error(0);
      break;
    }
    
    if (ELstepper.distanceToGo() == 0 && !isHome_EL)
    {
      error(1);
      break;
    }
    
    AZstepper.run();
    ELstepper.run();
  }
  
  /*Delay to Deccelerate*/
  long time = millis();  
  while(millis() - time < HOME_DELAY)
  {  
    AZstepper.run();
    ELstepper.run();
  }
  
  /*Reset the steps*/
  AZstepper.setCurrentPosition(0);
  ELstepper.setCurrentPosition(0); 
}
 
/*EasyComm 2 Protocol & Calculate the steps*/
void cmd_proc(long &stepAz, long &stepEl)
{
  /*Serial*/
  char buffer[BufferSize];
  char incomingByte;
  char *Data = buffer;
  char *rawData;
  static int BufferCnt = 0;
  char data[100];
  
  double angleAz, angleEl;
  
  /*Read from serial*/
  while (Serial.available() > 0)
  {
    incomingByte = Serial.read();
    /* XXX: Get position using custom and test code */
    if (incomingByte == '!')
    {
      /*Get position*/
      Serial.print("TM");
      Serial.print(1);
      Serial.print(" ");
      Serial.print("AZ");
      Serial.print(1*step2deg(AZstepper.currentPosition()), 1);
      Serial.print(" ");
      Serial.print("EL");
      Serial.println(1*step2deg(ELstepper.currentPosition()), 1);
    }
    /*new data*/
    else if (incomingByte == '\n')
    {
      buffer[BufferCnt] = 0;
      if (buffer[0] == 'A' && buffer[1] == 'Z')
      {
        if (buffer[2] == ' ' && buffer[3] == 'E' && buffer[4] == 'L')
        {
          /*Get position*/
          Serial.print("AZ");
          Serial.print(step2deg(AZstepper.currentPosition()), 1);
          Serial.print(" ");
          Serial.print("EL");
          Serial.print(step2deg(ELstepper.currentPosition()), 1);
          Serial.println(" ");
        }
        else
        {
          /*Get the absolute value of angle*/
          rawData = strtok_r(Data, " " , &Data);
          strncpy(data, rawData+2, 10);
          if (isNumber(data))
          {
            angleAz = atof(data);
            /*Calculate the steps*/
            stepAz = deg2step(angleAz);
          }
          /*Get the absolute value of angle*/
          rawData = strtok_r(Data, " " , &Data);
          if (rawData[0] == 'E' && rawData[1] == 'L')
          {
            strncpy(data, rawData+2, 10);
            if (isNumber(data))
            {
              angleEl = atof(data);
              /*Calculate the steps*/
              stepEl = deg2step(angleEl);
            }
          }
        }
      }
      /*Stop Moving*/
      else if (buffer[0] == 'S' && buffer[1] == 'A' && buffer[2] == ' ' && buffer[3] == 'S' && buffer[4] == 'E')
      {
        /*Get position*/
        Serial.print("AZ");
        Serial.print(step2deg(AZstepper.currentPosition()), 1);
        Serial.print(" ");
        Serial.print("EL");
        Serial.println(step2deg(ELstepper.currentPosition()), 1);
        stepAz = AZstepper.currentPosition();
        stepEl = ELstepper.currentPosition();
      }
      else if (buffer[0] == 'V' && buffer[1] == 'E') 
      {
                Serial.print("VE");
                Serial.println("SatNOGS-v3-RAMPS");                
      }
      /*Reset the rotator*/
      else if (buffer[0] == 'R' && buffer[1] == 'E' && buffer[2] == 'S' && buffer[3] == 'E' && buffer[4] == 'T')
      {
        /*Get position*/
        Serial.print("AZ");
        Serial.print(step2deg(AZstepper.currentPosition()), 1);
        Serial.print(" ");
        Serial.print("EL");
        Serial.println(step2deg(ELstepper.currentPosition()), 1);
        /*Move the steppers to initial position*/
        Homing(deg2step(-MAX_AZ_ANGLE), deg2step(-MAX_EL_ANGLE));
        /*Zero the steps*/
        stepAz = 0;
        stepEl = 0;
      }      
      BufferCnt = 0;
      /*Reset the disable motor time*/
      t_DIS = 0;
    }
    /*Fill the buffer with incoming data*/
    else {
      buffer[BufferCnt] = incomingByte;
      BufferCnt++;
    }
  }
}

/*Error Handling*/
void error(int num_error)
{
  switch (num_error)
  {
    /*Azimuth error*/
    case (0):
      while(1)
      {
        Serial.println("AL001");
        delay(100);
      }
    /*Elevation error*/
    case (1):
      while(1)
      {
        Serial.println("AL002");
        delay(100);
      }
    default:
      while(1)
      {
        Serial.println("AL000");
        delay(100);
      }
  }
}

/*Send pulses to stepper motor drivers*/
void stepper_move(long stepAz, long stepEl)
{
  AZstepper.moveTo(stepAz);
  ELstepper.moveTo(stepEl);
  
  //AZstepper.runToNewPosition(stepAz);
  
  //Serial.println(AZstepper.currentPosition());
  
  AZstepper.run();
  ELstepper.run();
}

/*Convert degrees to steps*/
long deg2step(double deg)
{
//  Serial.print("Deg2step:");
//  Serial.println(deg);
  return(long(SPRG*deg));
}

/*Convert steps to degrees*/
double step2deg(long Step)
{ 
//  Serial.print("Step2deg:");
//  Serial.println(Step);
   
  return(double(Step/SPRG));
}

/*Check if is argument in number*/
boolean isNumber(char *input)
{
  for (int i = 0; input[i] != '\0'; i++)
  {
    if (isalpha(input[i]))
      return false;
  }
   return true;
}
