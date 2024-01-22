//version 11 - code for the UROP simulator
//No Delay() are used to achieve non-block code operation:
  //command from serial port only switch component working-states
  //type of specific operation are determined internally within functions by working-states
//IR sensor is aborted for the complexity to modify stepper motor speed

//version 11.1: introduce softStart stuff for actuator

//STATING CONST AND VAR
//-----------------------------------------------------------------------------------------------------------------------------------------------
// Motor Const-------------------------------------------------------
int M1dirpin = 7; //Motor X direction pin
int M1steppin = 6; //Motor X step pin
int M1en=8; //Motor X enable pin
int M2dirpin = 4; //Motor Y direction pin
int M2steppin = 5; //Motor Y step pin
int M2en=12; //Motor Y enable pin
//Motor Var
int Direction = 1; // CW = -1; CCW = 1;
int StepDelay1 = 1; //delay after flusing LOW input
int StepDelay2 = 2; //delay after step
//Motor Working States
enum MotorState{
  MOTORREADY,
  MOTORSET,
  MOTORDELAY1,
  MOTORDRIVE,
  MOTORDELAY2};
MotorState MotorCurrentState = MOTORREADY;
unsigned long MotorTimestamp = 0;
bool MotorRun = false;

//Actuator Const-----------------------------------------------------
const int IN1=11;
const int IN2=9;
const int PWM=10;
const int ExtendSpeed = 50; // PWM values, int 0 - 255
const int RetractSpeed = 50;
const int ResetUpSpeed = 150;
const int ResetDownSpeed = 150;
int currentPWM = 0;
int targetPWM = 100;  // For smooth transition(Extend/Retract/ResetUp/ResetDown)
const int rampRate = 2;  // increment of smooth transition
const float lowerThreshold = 9.5; //9.5cm, for usSensor
const float upperThreshold = 10.5; // 10.5cm
//Limit Switch Const
const int upperLSPin = 18; // A4 analog
const int lowerLSPin = 19; // A5 analog
int upperLSState = 0; //LOW means the switch is pressed.
int lowerLSState = 0;
//Actuator Working States
enum ActuatorState {
  EXTENDING,
  RETRACTING,
  RESETTING };
ActuatorState AcCurrentState = RESETTING;
ActuatorState AcPreviousState = RESETTING;
bool upperLimitReached = false;
bool lowerLimitReached = false;
bool AcGoExtend = false;
bool AcGoRetract = false;
bool AcGoReset = false;


//Infrared Sensor Const----------------------------------------------
const int irSensorPin = A2;         // Constants for infrared sensor pin
const int BLACK_THRESHOLD = 400;    // Constants for calibration
//PID Var 
//tbd

//Ultrasonic Sensor Const--------------------------------------------
const int trigPin = 3;    // Trigger
const int echoPin = 2;    // Echo
//Ultrasonic Sensor Working States
enum UltrasonicState {
  PREPARING,
  TRIGGERING,
  ECHOING};
UltrasonicState UsCurrentState = PREPARING;
unsigned long UsTimestamp = 0;
const int prepareDuration = 5;
const int triggerDuration = 10;
//EMA 
float cm;
float previous_EMA = 0;  // Initial value
float current_EMA = 0;  // current_EMA is the Final Distance to use
const float alpha = 0.2;  // weight factor, should be between 0 and 1 Adjust as necessary.



//SETUP------------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  // Initialize the infrared sensor pin as input
  pinMode(irSensorPin, INPUT);

  // Serial Communication Initialization
  Serial.begin(9600); // 9600 bits per second data transmission

  //Actuator Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  //Limit Switch Pins
  pinMode(upperLSPin, INPUT_PULLUP);  // Enable internal pull-up resistor
  pinMode(lowerLSPin, INPUT_PULLUP);  // Enable internal pull-up resistor

  //Motor Pins
  pinMode(M1dirpin,OUTPUT);
  pinMode(M1steppin,OUTPUT);
  pinMode(M1en,OUTPUT);
  pinMode(M2dirpin,OUTPUT);
  pinMode(M2steppin,OUTPUT);
  pinMode(M2en,OUTPUT);
  digitalWrite(M1en,LOW);// Low Level Enable
  digitalWrite(M2en,LOW);// Low Level 

  //Ultrasonic Sensor Pins
  //pinMode(trigPin, OUTPUT);
  //pinMode(echoPin, INPUT);
}


//STATING Functions
//-----------------------------------------------------------------------------------------------------------------------------------------------
//Motor Control Funtions---------------------------------------------
void MotorStep(MotorState state){
  if (state == MOTORSET){
    if (Direction < 0) { //Direction is set in by serial command
      digitalWrite(M1dirpin,HIGH);
      digitalWrite(M2dirpin,HIGH);
    } else {
      digitalWrite(M1dirpin,LOW);
      digitalWrite(M2dirpin,LOW);
    }
    digitalWrite(M1steppin,LOW);
    digitalWrite(M2steppin,LOW);
  } else if (state == MOTORDRIVE) {
    digitalWrite(M1steppin,HIGH); //Rising step
    digitalWrite(M2steppin,HIGH);
  } else {
    //Motor remains idle in MOTORREADY state
    //Motor remains idle in MOTORDELAY1 state
    //Motor remains still in MOTORDELAY2 state
    return;  
  }  
}

//Actuator Control Functions-----------------------------------------
void adjustPWM() {
    if (currentPWM < targetPWM) {
        currentPWM += rampRate;
        if (currentPWM > targetPWM) {  // prevent overshooting
            currentPWM = targetPWM;
        }
    } else if (currentPWM > targetPWM) {
        currentPWM -= rampRate;
        if (currentPWM < targetPWM) {  // prevent overshooting
            currentPWM = targetPWM;
        }
    }
    analogWrite(PWM, currentPWM);  // update the actuator's PWM
}
void AcBrake(){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
}
void AcExtend(){//int Speed) { //LS Limited Extension
  if (!upperLimitReached){
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
  } else {
    AcBrake();  
  }
}
void AcRetract(){//LS Limited Retraction
  if (!lowerLimitReached){
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
  } else {
    AcBrake();
  }
}
void AcMasterControl(ActuatorState state){
  // Adjusting the PWM during each cycle
  adjustPWM();  
  if (state == RESETTING){ //Reseting ac based on usSensor
    //if (current_EMA >= upperThreshold){
    //  AcRetract(ResetDownSpeed);  
    //} else if (current_EMA <= lowerThreshold){
    //  AcExtend(ResetUpSpeed);
    //} else {
      targetPWM = 0;
      AcBrake();
    //}  
  } else if (state == RETRACTING){ //retract ac
    targetPWM = RetractSpeed;
    AcRetract();
  } else if (state == EXTENDING){ //extend ac
    targetPWM = ExtendSpeed;
    AcExtend();  
  } 
}

//Ultrasonic Distance Detection--------------------------------------
void UsFindDistance(UltrasonicState state){
  if (state == PREPARING){
    digitalWrite(trigPin, LOW);
  } else if (state == TRIGGERING){
    digitalWrite(trigPin, HIGH);
  } else if (state == ECHOING){
    digitalWrite(trigPin, LOW);
    pinMode(echoPin, INPUT);
    // Detect High Pulse and Record time-lapse
    pinMode(echoPin, INPUT);
    float duration = pulseIn(echoPin, HIGH);
    cm = (duration/2) / 29.1; // Divide by 29.1 for cm
    //Refine Result with EMA
    current_EMA = EMA_filter(cm, previous_EMA, alpha);
    previous_EMA = current_EMA; //current_EMA is the desired value
    //Serial.print("Distance = "); Serial.println(current_EMA); 
  }
}

//EMA Filter---------------------------------------------------------
float EMA_filter(float new_value, float previous_EMA, float alpha) {
    return (alpha * new_value) + ((1.0 - alpha) * previous_EMA);
}  




//MAIN LOOP--------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------
void loop() {

  //Time Lapse from the start of the program-------------------------
  unsigned long currentMillis = millis();

  // Check limit switch states and update flags----------------------
  upperLSState = digitalRead(upperLSPin);  // Read the state of the switch
  lowerLSState = digitalRead(lowerLSPin);  // LOW means the switch is pressed
  upperLimitReached = (upperLSState == LOW && AcCurrentState == EXTENDING);
  lowerLimitReached = (lowerLSState == LOW && AcCurrentState == RETRACTING);


  //Ultrasonic Sensor Working-state Switch---------------------------
  //sswitch (UsCurrentState) {
  //  case PREPARING:
  //    if (currentMillis - UsTimestamp >= prepareDuration) { 
  //      UsCurrentState = TRIGGERING;
  //      UsTimestamp = currentMillis; 
  //    } break;
  //  case TRIGGERING:
  //    if (currentMillis - UsTimestamp >= triggerDuration) { 
  //      UsCurrentState = ECHOING;
  //      UsTimestamp = currentMillis; 
  //    } break;
  //  case ECHOING:
  //    // Directly transition to PREPARING after one loop iteration
  //    UsCurrentState = PREPARING;
  //    UsTimestamp = currentMillis; 
  //    break;
  //  }
  //Ultrasonic Find Distance with EMA filter
  //Cause Great Delay if sensor not plugged in
  //Longer Distance, Longer it took for this to execute
  //UsFindDistance(UsCurrentState); //Result recorded with var: current_EMA


  //Motor Working-state Switch--------------------------------------
  switch (MotorCurrentState) {
    case MOTORREADY:
      if (MotorRun) {
        MotorCurrentState = MOTORSET;
      } break;
    case MOTORSET:
      MotorCurrentState = MOTORDELAY1;
      MotorTimestamp = currentMillis;
      break;
    case MOTORDELAY1:
      if (currentMillis - MotorTimestamp >= StepDelay1) {
        MotorCurrentState = MOTORDRIVE;
      } break;
    case MOTORDRIVE:
      MotorCurrentState = MOTORDELAY2;
      MotorTimestamp = currentMillis;
      break;
    case MOTORDELAY2:
      if (currentMillis - MotorTimestamp >= StepDelay2) {
        MotorCurrentState = MOTORREADY;
        MotorRun = false;
      } break;
    }
  //Running Motor
  MotorStep(MotorCurrentState);


  //Actuator Working-state Switch-------------------------------------
  switch (AcCurrentState) {
    case RESETTING:
      if (AcGoExtend){
        AcCurrentState = EXTENDING;
      } else if (AcGoRetract){
        AcCurrentState = RETRACTING;
      } break;
    case EXTENDING:
      if (!AcGoExtend){
        AcCurrentState = RESETTING;
      } break;
    case RETRACTING:
      if (!AcGoRetract){
        AcCurrentState = RESETTING;
      } break;
    }
  //Running Actuator
  AcMasterControl(AcCurrentState);


  //Commands from Serial Port-----------------------------------------
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    //Command for Motor Control
    if (command.equals("CWR")){
      MotorRun = true;
      Direction = -1;
    } else if (command.equals("CCR")){
      MotorRun = true;
      Direction = 1;
    }

    //Command for Actuator Control
    if (command.equals("EXT")){ // extend actuator
      AcGoExtend = true;
      AcGoRetract = false;
      AcGoReset = false;
    } else if (command.equals("RET")) { //retract actuator
      AcGoExtend = false;
      AcGoRetract = true;
      AcGoReset = false;
    } else if (command.equals("RST")) { //reset actuator
      AcGoExtend = false;
      AcGoRetract = false;
      AcGoReset = true;
    }
    
  }


}
