/* Author: Maarten Weyn
   Repo: https://github.com/maartenweyn/DajiJug
   */


#include <Servo.h>
#include <string.h>

#define logUSB 1
#define logBT 0

#define RRearCoxaServo 0
#define RRearFemurServo 1
#define RRearTibiaServo 2
#define RCenterCoxaServo 3
#define RCenterFemurServo 4
#define RCenterTibiaServo 5
#define RFrontCoxaServo 6
#define RFrontFemurServo 7
#define RFrontTibiaServo 8
#define LRearCoxaServo 9
#define LRearFemurServo 10
#define LRearTibiaServo 11
#define LCenterCoxaServo 12
#define LCenterFemurServo 13
#define LCenterTibiaServo 14
#define LFrontCoxaServo 15
#define LFrontFemurServo 16
#define LFrontTibiaServo 17


struct RobotStatus {
  int GaitType;  
  //Init position of the leg
  int RFGaitLegNr;          
  int RMGaitLegNr;          
  int RRGaitLegNr;          
  int LFGaitLegNr;          
  int LMGaitLegNr;          
  int LRGaitLegNr; 
  boolean LastLeg;
  
  int StepsInGait; // Nr of steps in gait
  int TLDivFactor; // Nr of steps that a leg is on the floor while waling
  boolean HalfLiftHeight; // does the step sequence has half lifted legs
  double LegLiftHeight;     //Current Travel height
  int TravelSpeed;       //Current Move Speed [ms]
  double TravelLengthX;     //Current Travel length X (sideways)
  double TravelLengthZ;     //Current Travel length Z (front - back)
  double TravelRotationY;   //Current Travel Rotation Y
  
  boolean IsOffRoadLiftHeight; //Allows user to switch between off-road and on-road step heights
  int CycleLegs;
  
  // Body position
  double BodyPosX;      //Global Input for the position of the body
  double BodyPosY;      //Global Input for the position of the body
  double BodyPosZ;      //Global Input for the position of the body
        
        //Body Inverse Kinematics
  double BodyRotX;      //Global Input pitch of the body
  double BodyRotY;      //Global Input rotation of the body
  double BodyRotZ;      //Global Input roll of the body
  double BodyIKPosX;    //Output Position X of feet with Rotation
  double BodyIKPosY;    //Output Position Y of feet with Rotation
  double BodyIKPosZ;    //Output Position Z of feet with Rotation

        //Leg Inverse Kinematics
  boolean IKSolution;        //Output true if the solution is possible
  boolean IKSolutionWarning;  //Output true if the solution is NEARLY possible
  boolean IKSolutionError;   //Output true if the solution is NOT possible
  double IKFemurAngle;  //Output Angle of Femur in degrees
  double IKTibiaAngle;  //Output Angle of Tibia in degrees
  double IKCoxaAngle;   //Output Angle of Coxa in degrees
  
  // Actual positions, messured in mm from the coxa offset to the tips of the feet
  double  RFPosX;           //Actual Position of the Right Front Leg
  double  RFPosY;
  double  RFPosZ;
  double  RMPosX;          //Actual Position of the Right Middle Leg
  double  RMPosY;
  double  RMPosZ;
  double  RRPosX;           //Actual Position of the Right Rear Leg
  double  RRPosY;
  double  RRPosZ;
  double  LFPosX;           //Actual Position of the Left Front Leg
  double  LFPosY;
  double  LFPosZ;
  double  LMPosX;          //Actual Position of the Left Middle Leg
  double  LMPosY;
  double  LMPosZ;
  double  LRPosX;           //Actual Position of the Left Rear Leg
  double  LRPosY;
  double  LRPosZ;  
  
  // Angles
  double*  RFCoxaAngle;       //Actual Angle of the Right Front Leg
  double*  RFFemurAngle;
  double*  RFTibiaAngle;

  double*  RMCoxaAngle;       //Actual Angle of the Right Middle Leg
  double*  RMFemurAngle;
  double*  RMTibiaAngle;

  double*  RRCoxaAngle;       //Actual Angle of the Right Rear Leg
  double*  RRFemurAngle;
  double*  RRTibiaAngle;

  double*  LFCoxaAngle;       //Actual Angle of the Left Front Leg
  double*  LFFemurAngle;
  double*  LFTibiaAngle;

  double*  LMCoxaAngle;       //Actual Angle of the Left Middle Leg
  double*  LMFemurAngle;
  double*  LMTibiaAngle;

  double*  LRCoxaAngle;       //Actual Angle of the Left Rear Leg
  double*  LRFemurAngle;
  double*  LRTibiaAngle;
  
  double GaitStep;          //Global Input Gait step
  double TravelMulti;       //Multiplier for the length of the step
  
  
  // Relative position corresponding to the Gait
  double RFGaitPosX;        
  double RFGaitPosY;
  double RFGaitPosZ;
  double RFGaitRotY;        //Relative rotation corresponding to the Gait

  double RMGaitPosX;
  double RMGaitPosY;
  double RMGaitPosZ;
  double RMGaitRotY;

  double RRGaitPosX;
  double RRGaitPosY;
  double RRGaitPosZ;
  double RRGaitRotY;

  double LFGaitPosX;
  double LFGaitPosY;
  double LFGaitPosZ;
  double LFGaitRotY;

  double LMGaitPosX;
  double LMGaitPosY;
  double LMGaitPosZ;
  double LMGaitRotY;

  double LRGaitPosX;
  double LRGaitPosY;
  double LRGaitPosZ;
  double LRGaitRotY;
  
};




// Coxa, Femur, Tibia of   RR          RC          RF          LR          LC          LF
const int servoPins[18] = {56, 55, 54, 51, 50, 49, 48, 47, 46, 11, 12, 13, 24, 25, 26, 27, 28, 29};

Servo servos[18];

int servoTimingDirection[18] = {   -1, -1, -1, 
                                   -1,  -1, -1, 
                                   -1, -1, -1,
                                  1,  1, 1,
                                  1, 1, 1,
                                   1,  1,  1}; 
                                   
int servoAngleDirection[18] = {   -1, -1, -1, 
                                   -1,  -1, -1, 
                                   -1, -1, -1,
                                  1,  1, 1,
                                  1, 1, 1,
                                   1,  1,  1};

// Settings                                   
int servoAngleOffset[18] = {     -93, -93, -91, 
                                 -83, -85, -79, 
                                 -90, -86, -87,
                                 -87, -98, -104,
                                 -83, -99, -105,
                                 -72, -93, -100};                                    
                                   
double servoAngleRanges[18][2] = {  {-45, 45}, {-55, 65}, {-70, 90}, // R Rear OK
                                    {-45, 45}, {-70, 75}, {-70, 90}, // R Center tibia servo not working
                                    {-45, 45}, {-55, 75}, {-70, 90}, // R Front ok
                                    {-45, 45}, {-55, 65}, {-65, 90}, // L Rear tibio servo sometimes wrong
                                    {-45, 45}, {-70, 75}, {-70, 90}, // L Center OK
                                    {-45, 45}, {-55, 75}, {-70, 90}}; // L Front coxa servo not working
// Body dimensions (in mm)
double CoxaLength = 22;       //Length of the Coxa [mm]
double FemurLength = 100;      //Length of the Femur [mm]
double TibiaLength = 145;     //Lenght of the Tibia [mm]
double WakeStandingHeightY = 50; //Typical stading height (body above ground) in [mm]

double RFOffsetX = 57;       //Distance X from center of the body to the Right Front coxa
double RFOffsetZ = -103;       //Distance Z from center of the body to the Right Front coxa
double RMOffsetX = 76;       //Distance X from center of the body to the Right Middle coxa
double RMOffsetZ = 0;         //Distance Z from center of the body to the Right Middle coxa
double RROffsetX = 57;       //Distance X from center of the body to the Right Rear coxa
double RROffsetZ = 103;        //Distance Z from center of the body to the Right Rear coxa

double LFOffsetX = -57;        //Distance X from center of the body to the Left Front coxa
double LFOffsetZ = -103;       //Distance Z from center of the body to the Left Front coxa
double LMOffsetX = -76;        //Distance X from center of the body to the Left Middle coxa
double LMOffsetZ = 0 ;        //Distance Z from center of the body to the Left Middle coxa
double LROffsetX = -57;        //Distance X from center of the body to the Left Rear coxa
double LROffsetZ = 103;        //Dista

double RFCoxaOrrientation = 45;
double RMCoxaOrrientation = 0;
double RRCoxaOrrientation = -45;
double LFCoxaOrrientation = 135;
double LMCoxaOrrientation = 180;
double LRCoxaOrrientation = -135;
double TibiaOffset = 90;                                    
                                    
double servoAngleFactors[18];                                    
double servoAngles[18];
double servoPrevAngles[18];

// serial data
String inputString = ""; // string to hold incoming data
boolean stringComplete = false;
bool moving = true;

RobotStatus Status;

void setup()
{   
  Serial.begin(115200);
  //Serial.begin(9600);
  inputString.reserve(100); 
  
  Serial1.begin(9600);
  
  initializeValues();
  attachServos(); 
}
  
void initializeValues()
{
  for (int i=0;i<18;i++)
  {
    servoPrevAngles[i] = -200;
    servoAngles[i] = 0;
    servoAngleFactors[i] = servoTimingDirection[i] * 10;
  } 
  
  //Feet Positions
  //Measured in mm from the coxa offset of the robot to the tips of the feet)
  Status.RFPosX = 65;           
  Status.RFPosY = -35;
  Status.RFPosZ = -65;
  Status.RMPosX = 92;        
  Status.RMPosY = -35;
  Status.RMPosZ = 0;
  Status.RRPosX = 65;          
  Status.RRPosY = -35;
  Status.RRPosZ = 65;
  Status.LFPosX = -65;          
  Status.LFPosY = -35;
  Status.LFPosZ = -65;
  Status.LMPosX = -92;        
  Status.LMPosY = -35;
  Status.LMPosZ = 0;
  Status.LRPosX = -65;          
  Status.LRPosY = -35;
  Status.LRPosZ = 65;  
  
  Status.GaitType = 0;
  // Body position
  Status.BodyPosY = 0;
  Status.BodyPosX = 0;
  Status.BodyPosZ = 0;
  Status.BodyRotY = 0;
  Status.BodyRotX = 0;
  Status.BodyRotZ = 0;
  // Movements
  Status.TravelLengthX = 0;
  Status.TravelLengthZ = 0;
  Status.TravelRotationY = 0;
  
  //Body Positions
  Status.BodyPosX = 0;
  Status.BodyPosY = 0;
  Status.BodyPosZ = 0;

  //Body Rotations
  Status.BodyRotX = 0;
  Status.BodyRotY = 0;
  Status.BodyRotZ = 0;

  //Gait
  Status.LegLiftHeight = WakeStandingHeightY * (2.0 / 3.0);
  Status.GaitStep = 1;
  
  Status.RFCoxaAngle = &servoAngles[RFrontCoxaServo];
  Status.RFFemurAngle = &servoAngles[RFrontFemurServo];
  Status.RFTibiaAngle = &servoAngles[RFrontTibiaServo];
  Status.RMCoxaAngle = &servoAngles[RCenterCoxaServo];
  Status.RMFemurAngle = &servoAngles[RCenterFemurServo];
  Status.RMTibiaAngle = &servoAngles[RCenterTibiaServo];
  Status.RRCoxaAngle = &servoAngles[RRearCoxaServo];
  Status.RRFemurAngle = &servoAngles[RRearFemurServo];
  Status.RRTibiaAngle = &servoAngles[RRearTibiaServo];
  Status.LFCoxaAngle = &servoAngles[LFrontCoxaServo];
  Status.LFFemurAngle = &servoAngles[LFrontFemurServo];
  Status.LFTibiaAngle = &servoAngles[LFrontTibiaServo];
  Status.LMCoxaAngle = &servoAngles[LCenterCoxaServo];
  Status.LMFemurAngle = &servoAngles[LCenterFemurServo];
  Status.LMTibiaAngle = &servoAngles[LCenterTibiaServo];
  Status.LRCoxaAngle = &servoAngles[LRearCoxaServo];
  Status.LRFemurAngle = &servoAngles[LRearFemurServo];
  Status.LRTibiaAngle = &servoAngles[LRearTibiaServo];
  
  // Speed
  Status.TravelSpeed = 1000;
  GaitSelect(0); 
}

void attachServos()
{
  for (int i=0;i<18;i++)
  {
    servos[i].attach(servoPins[i], 600, 2400);
  }  
}

void detachServos()
{
  for (int i=0;i<18;i++)
  {
    servos[i].detach();
  }  
}

void loop() {
  
  // read inputs
  if (stringComplete) {
    logString("SerialInput: ");
    logString(inputString);
    logString("\n");
    int startPos = 0;
    int cmdPos = inputString.indexOf('=', startPos);
    String command = inputString.substring(startPos, cmdPos);
    int endPos = inputString.indexOf(';', startPos);
    String value = inputString.substring(cmdPos+1, endPos);     
    
    if (command == "ver")
    {
      logString("0.1");
      
    } 
   else if  (command == "servoangles")
   {
      double values[18];
      splitString(value, 18, values);      
      double angles[18];
      
      for (int i=0;i<18;i++)
      {
        //Serial.print(i);
        //Serial.print(": ");
        //Serial.print(values[i]);
        //Serial.print(" -> ");
        angles[i] = values[i]; 
        //Serial.println(angles[i]);
      }
      
      setPositions(angles);
   }
   else if  (command == "servotimes")
   {
      double times[18];
      splitString(value, 18, times);
      
      setServoTimes(times);
   }
   else if  (command == "move")
   {
     if (value == "wake")
     {
      wake();
      logString("wake\n ");
     } else if (value == "sleep")
     {
      sleep();
      logString("sleep\n ");
     } 
   } 
   else if  (command == "motion")
   {
      double motion[10];
      splitString(value, 10, motion);
      
      Status.BodyPosY = motion[0];
      Status.BodyPosX = motion[1];
      Status.BodyPosZ = motion[2];
      Status.BodyRotY = motion[3];
      Status.BodyRotX = motion[4];
      Status.BodyRotZ = motion[5];
      Status.TravelLengthX = motion[6];      
      Status.TravelLengthZ = motion[7];
      Status.TravelRotationY = motion[8]; 
      Status.TravelSpeed = motion[9];      
   } 
   else if  (command == "gaittype")
   {
      int type = (int) stringToNumber(value);
      GaitSelect(type);
   } else if  (command == "onroad")
   {
      if (value == "0") // offroad
      {
        Status.IsOffRoadLiftHeight = true;
        Status.LegLiftHeight = Status.BodyPosZ * (1.0 / 3.0);
      }
      else // onroad
      {
        Status.IsOffRoadLiftHeight = true;
        Status.LegLiftHeight = Status.BodyPosZ * (2.0 / 3.0);
      }
   } 
    
    // clear the string:
    inputString = "";
    stringComplete = false;
  } 
  
  // Calculate Angles
  DoInverseKinetics();
  
  // Send servoangles to serial for debug
  LogAngles("calculatedangles");
  
  setPositions(servoAngles);
  
  
  delay(Status.TravelSpeed);
}


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if ((inChar == '\n' || inChar == ';')  && (inputString.length() > 0))
    {
      stringComplete = true;
    } 
  }
}

void serialEvent1() {
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n' || inChar == ';') {
      stringComplete = true;
    } 
  }
}

void LogAngles(String name)
{
  logString(name);
  logString("=");
  logInt(*Status.RRCoxaAngle); logString(","); logInt(*Status.RRFemurAngle); logString(","); logInt(*Status.RRTibiaAngle); logString(","); 
  logInt(*Status.RMCoxaAngle); logString(","); logInt(*Status.RMFemurAngle); logString(","); logInt(*Status.RMTibiaAngle); logString(","); 
  logInt(*Status.RFCoxaAngle); logString(","); logInt(*Status.RFFemurAngle); logString(","); logInt(*Status.RFTibiaAngle); logString(","); 
  logInt(*Status.LRCoxaAngle); logString(","); logInt(*Status.LRFemurAngle); logString(","); logInt(*Status.LRTibiaAngle); logString(","); 
  logInt(*Status.LMCoxaAngle); logString(","); logInt(*Status.LMFemurAngle); logString(","); logInt(*Status.LMTibiaAngle); logString(","); 
  logInt(*Status.LFCoxaAngle); logString(","); logInt(*Status.LFFemurAngle); logString(","); logInt(*Status.LFTibiaAngle); logString(";\n");  
}
