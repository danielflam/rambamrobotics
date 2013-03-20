#ifndef SOCCER_PID
#define SOCCER_PID

// 
// based on http://playground.arduino.cc/Code/PIDLibrary
// Arduino PID Library
// by Brett Beauregard
// contact: br3ttb@gmail.com
// 

//Constants used in some of the functions below
#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1

class PID
{

public:


  //commonly used functions **************************************************************************
  // * constructor.  links the PID to the Input, Output, and 
  //   Setpoint.  Initial tuning parameters are also set here

  PID()
  {
    setup (5,3,1,DIRECT, 0.001);
  }

  void setup(
/*double myInput_, double myOutput_, double mySetpoint_,  */
  double P, double I, double D, byte dir, double reductionFactor_) /*:     
  myInput(myInput_),
  myOutput(myOutput_),
  mySetpoint(mySetpoint_)*/
  {
    inAuto = false;
    controllerDirection = dir;
    SetTuning(P, I, D);
    SetOutputLimits(0, 255);				//default output limit corresponds to 
    SampleTime = 5;							//default Controller Sample Time is 50 ms
    lastTime = millis()-SampleTime;				
    reductionFactor = reductionFactor_;
  }
  
  void SetMode(int Mode)               // * sets PID to either Manual (0) or Auto (non-0)
  {
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
      Initialize();
    }
    inAuto = newAuto;
  }

  bool Compute()                       // * performs the PID calculation.  it should be
      //   called every time loop() cycles. ON/OFF and
    //   calculation frequency can be set using SetMode
    //   SetSampleTime respectively
  {
    if(!inAuto) return false;
    unsigned long now = millis();
    unsigned long timeChange = (now - lastTime);
    if(timeChange>=SampleTime)
    {
      /*Compute all the working error variables*/
      double input = myInput;
      double error = mySetpoint - input;
      

      ITerm = constrain(ITerm + ki * error, outMin, outMax);
//      if(ITerm > outMax) ITerm= outMax;
//      else if(ITerm < outMin) ITerm= outMin;
//      double dInput = (input - lastInput);
      diffError = 0.9*diffError + 0.1*(input - lastInput);

      /*Compute PID Output*/
      double output = reductionFactor*(kp * error + ITerm - kd * diffError);

      if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
      myOutput = output;

//       Serial.print("PID in: ");
//        Serial.print(myInput);
//        Serial.print(" out: ");
//        Serial.print(output);
//        Serial.print(" ");
//        Serial.print(myOutput);
//        Serial.print("\n");

      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
      return true;
    }
    else return false;

  }

  void reset()
  {
    diffError = 0.0;
    lastInput = mySetpoint;
    ITerm = 0.0;
  }

  void SetOutputLimits(double Min, double Max) //clamps the output to a specific range. 0-255 by default, but
    //it's likely the user will want to change this depending on
    //the application
  {
    if(Min >= Max) return;
    outMin = Min;
    outMax = Max;

    if(inAuto)
    {
      if(myOutput > outMax) myOutput = outMax;
      else if(myOutput < outMin) myOutput = outMin;

      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
    }
  }



  //available but not commonly used functions ********************************************************
  // * While most users will set the tunings once in the 
  void SetTuning(double Kp, double Ki, double Kd )         	  
    //   constructor, this function gives the user the option
    //   of changing tunings during runtime for Adaptive control
  {
    if (Kp<0 || Ki<0 || Kd<0) return;

    dispKp = Kp; 
    dispKi = Ki; 
    dispKd = Kd;

    double SampleTimeInSec = ((double)SampleTime)/1000;  
    kp = Kp;
    ki = Ki * SampleTimeInSec;
    kd = Kd / SampleTimeInSec;

    if(controllerDirection ==REVERSE)
    {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
    }    
  }

  void SetControllerDirection(int Direction)	  // * Sets the Direction, or "Action" of the controller. DIRECT
    //   means the output will increase when error is positive. REVERSE
    //   means the opposite.  it's very unlikely that this will be needed
    //   once it is set in the constructor.
  {
    if(inAuto && Direction !=controllerDirection)
    {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
    }   
    controllerDirection = Direction;

  }

  void SetSampleTime(int NewSampleTime)              // * sets the frequency, in Milliseconds, with which 
    //   the PID calculation is performed.  default is 100
  {
    if (NewSampleTime > 0)
    {
      double ratio  = (double)NewSampleTime
        / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
    }
  }


  //Display functions ****************************************************************
  double GetKp(){
    return kp;
  }						  // These functions query the pid for interal values.
  double GetKi(){
    return ki;
  }						  //  they were created mainly for the pid front-end,
  double GetKd(){
    return kd;
  }						  // where it's important to know what is actually 
  int GetMode(){ 
    return  inAuto ? AUTOMATIC : MANUAL;
  }			  //  inside the PID.
  int GetDirection() {
    return controllerDirection;
  }			  //


  double myInput;              // * Pointers to the Input, Output, and Setpoint variables
  double myOutput;             //   This creates a hard link between the variables and the 
  double mySetpoint;           //   PID, freeing the user from having to constantly tell us

private:
  void Initialize()
  {
    ITerm = myOutput;
    lastInput = myInput;
    if(ITerm > outMax) 
      ITerm = outMax;

    else if(ITerm < outMin) 
      ITerm = outMin;

  }

  double dispKp;				// * we'll hold on to the tuning parameters in user-entered 
  double dispKi;				//   format for display purposes
  double dispKd;				//

  double kp;                  // * (P)roportional Tuning Parameter
  double ki;                  // * (I)ntegral Tuning Parameter
  double kd;                  // * (D)erivative Tuning Parameter

  byte controllerDirection;

  //   what these values are.  with pointers we'll just know.

  unsigned long lastTime;
  double ITerm, lastInput;

  unsigned long SampleTime;
  double outMin, outMax;
  bool inAuto;
  double reductionFactor;
  double diffError;
};
#endif













