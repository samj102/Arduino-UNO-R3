/*
  few improvement:
    1. find out why keep turning when moving backwards and light turns purple 
    2. Fix line tracking
    3. Add Candle -done
    4. ultrasonic sensors?
    5. maybe imporve flame detection

*/


#include <FastLED.h>
#include <SharpIR.h>
#include <string.h>

// ---------------------------------------------------------
//                      OUTPUT PINS
// ---------------------------------------------------------

// ---- Movement ----
#include "Movement_Module.h"

#define LED_PIN 4  // Pin connected to the LED
#define NUM_LEDS 1 // Number of LEDs
#define BRIGHTNESS 20 // Initial brightness (adjust as needed)

CRGB leds[NUM_LEDS]; // Define an array to store the LED colors

// ---------------------------------------------------------
//                      INPUT PINS
// ---------------------------------------------------------

// ---- Sensors ----
#include "Sensor_Module.h"

// ---------------------------------------------------------
//                     Program Logic
// ---------------------------------------------------------

#define MaxHeight 10
#define MinHeight 3

#define MinDistance 20

// Keep track of how many times the robot is moving in a straight line
// if (number > 4) : turn right to take it out of the loop of going back and forth (explore the field)
unsigned short int straightLineTracker = 0;

long int TurnBackTime = 3000.0;

// ---------------------------------------------------------
//                      Helper Functions
// ---------------------------------------------------------


enum ProtocolTypes {
  EdgeAvoidance,
  ObjectAvoidance,
  LineTracking
};

class ProtocolInterface {
  public:
    ProtocolInterface(ProtocolTypes Protocol) {
      m_numOfOutputs = 0;
      madeAnyDecisions = false;
    }
    int Log() {
      m_numOfOutputs++;
    }
  private:
    int m_numOfOutputs;
    bool madeAnyDecisions;
};

class Timer {
  public:
    Timer() {
      m_valueSet = false;
    }

    void m_Start() {
      if (!m_valueSet) {
        m_startTime = millis();
        m_valueSet = true;
      }
    }

    void m_ResetSetting() {
      // m_startTime = ;
      m_valueSet = false;
    }

    bool m_TimerDone(unsigned long timeLimit) {
      unsigned long currentTime = millis();

      if (((currentTime - m_startTime) > timeLimit) && (m_valueSet)) {
        return true;
      }
      return false;
    }

    void m_Print(bool endLine = true) {
      unsigned long currentTime = millis();
      if (endLine) Serial.println(currentTime - m_startTime);
      else Serial.print(currentTime - m_startTime);
    }

    bool m_AlreadyStarted() {
      return m_valueSet;
    }
  private:
    bool m_valueSet;
    unsigned long m_startTime;
};

// Function to change the color and brightness of the LED
void changeLedColor(CRGB color, uint8_t brightness = BRIGHTNESS) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = color;                // Set the LED color
  }
  FastLED.setBrightness(brightness); // Set the brightness
  FastLED.show();                   // Update the LED with the new color and brightness
}

Timer backwardTimer;

inline bool CheckSensorVal(float DistanceVal) {
  if ((DistanceVal > MaxHeight) || (DistanceVal <= MinHeight)) {
    return true;
  }
  return false;
}

// inline bool EdgeAvoidanceLogic() {
//   bool madeAnyDecisions = false;

//   // if all wheels are off the floor then stop
//   // if (CheckSensorVal(Edge_U_FL) && CheckSensorVal(Edge_U_FR) && CheckSensorVal(Edge_U_BL) && CheckSensorVal(Edge_U_BR)) {
//   //   Serial.println("------------ ALL SENSORS HIT ------------ ");
//   //   MovementFun::Stop();
//   // }
//   // else

//   if (direction != MovementDir::backward) {
//     backwardTimer.m_ResetSetting();
//     SensorFun::Read_EdgeAvoidance_FrontSensors();
//     if (CheckSensorVal(Edge_U_FL) && CheckSensorVal(Edge_U_FR)) {
//       Serial.println("------------ BOTH FORWARD SENSORS HIT ------------ ");
//       Serial.print("before: ");
//       Serial.print(direction + "\n");
//       MovementFun::Backward();
//       Serial.print(direction);
//       straightLineTracker++;
//       madeAnyDecisions = true;
//     } else if (CheckSensorVal(Edge_U_FL)) {
//       Serial.println("------------ FORWARD LEFT SENSOR HIT ------------ ");
//       if (direction == MovementDir::turnL) {
//         madeAnyDecisions = true;
//         MovementFun::Backward();
//       } else {
//         MovementFun::TurnRight45();
//         straightLineTracker = 0;
//         MovementFun::Forward();
//         madeAnyDecisions = true;
//       }

//     } else if (CheckSensorVal(Edge_U_FR)) {
//       Serial.println("------------ FORWARD RIGHT SENSOR HIT ------------ ");
//       if (direction == MovementDir::turnR) {
//         madeAnyDecisions = true;
//         MovementFun::Backward();
//       } else {
//         MovementFun::TurnLeft45();
//         straightLineTracker = 0;
//         MovementFun::Forward();
//         madeAnyDecisions = true;
//       }
//     }
//   } else if ((direction != MovementDir::forward)) {
//     SensorFun::Read_EdgeAvoidance_BackSensors();

//     //used to prevent the robot from going backward, when backward sensor is weaker
//     if (direction == MovementDir::backward) {
//       backwardTimer.m_Start();
//       // madeAnyDecisions = true;
//       if (backwardTimer.m_TimerDone(TurnBackTime)) {
//         // MovementFun::TurnLeft();
//         MovementFun::TurnLeft45();
//         MovementFun::TurnLeft45();
//         MovementFun::TurnLeft45();
//         MovementFun::Forward();
//         backwardTimer.m_ResetSetting();
//       }
//     }


//     if (CheckSensorVal(Edge_U_BL) && CheckSensorVal(Edge_U_BR)) {
//       Serial.println("------------ BOTH BACKWARD SENSORS HIT ------------ ");
//       MovementFun::Forward();
//       straightLineTracker++;
//       madeAnyDecisions = true;

//       // if the machine has been moving back and forth in a straight line for 2 time (4 sensor hits)
//       // Turn to right to make the robot explore the ground
//       if (straightLineTracker >= 4) {
//         Serial.println("------------ STOPPING ROBOT FROM MOVING IN STRAIGHT LINE (TURNING RIGHT) ------------ ");
//         MovementFun::TurnLeft45();
//         straightLineTracker = 0;
//       }
//     } else if (CheckSensorVal(Edge_U_BL)) {
//       Serial.println("------------ BACKWARD LEFT SENSOR HIT ------------ ");
//       MovementFun::TurnRight45();
//       straightLineTracker = 0;
//       MovementFun::Forward();
//       madeAnyDecisions = true;

//     } else if (CheckSensorVal(Edge_U_BR)) {
//       Serial.println("------------ BACKWARD RIGHT SENSOR HIT ------------ ");
//       MovementFun::TurnLeft45();
//       straightLineTracker = 0;
//       MovementFun::Forward();
//       madeAnyDecisions = true;
//     }
//   }
//   return madeAnyDecisions;
// }
MovementDir findir;
inline bool ObjectAvoidanceLogic() {
  
  bool madeAnyDecisions = false;
  SensorFun::Read_ObjectAvoidance_Sensors();

  // -------- ------- --------
  // -------- Forward --------
  // -------- ------- --------

  if ((Obj_IR_FL == 0) && (Obj_IR_FR == 0)){
    Serial.print("[Front] Corner Sensor: Both");
    madeAnyDecisions = true;
    MovementFun::Backward();
  }
  else if (Obj_IR_FL == 0) {
    Serial.print("[Front] Corner Sensor: Turn Right");
    madeAnyDecisions = true;
    changeLedColor(CRGB::Green);
    MovementFun::TurnRight45();
    MovementFun::Forward();
  }
  else if (Obj_IR_FR == 0) {
    Serial.print("[Front] Corner Sensor: Turn Left");
    madeAnyDecisions = true;
    changeLedColor(CRGB::Red);
    MovementFun::TurnLeft45();
    MovementFun::Forward();
  }
  else if (Obj_Sharp_Front < 10) {
    Serial.print("[Front] Corner Sensor: Sharp");
    madeAnyDecisions = true;
    changeLedColor(CRGB::Blue);
    MovementFun::TurnRight45();
    MovementFun::Forward();
  }
  // -------- -------- --------
  // -------- Backward --------
  // -------- -------- --------
  if (direction == MovementDir::backward) {
    // changeLedColor(CRGB::Yellow);  
    if ((Obj_U_Back < 10) || ((Edge_U_BL <= 8) && (Edge_U_BR <= 8)) ) {
      Serial.print("[Backward] Corner Sensor: Both");
      changeLedColor(CRGB::Purple);  
      MovementFun::Forward();
      madeAnyDecisions = true;
    }
    else if (Edge_U_BL <= 10)
    {
      Serial.print("[Backward] Corner Sensor: Turn Left");
      changeLedColor(CRGB::Purple);
      MovementFun::TurnLeft45();
      MovementFun::Forward();
      madeAnyDecisions = true;
    }
    else if (Edge_U_BR <= 10)
    {
      Serial.print("[Backward] Corner Sensor: Turn Right");
      changeLedColor(CRGB::Purple);  
      MovementFun::TurnRight45();
      MovementFun::Forward();
      madeAnyDecisions = true;
    }
  }
  return madeAnyDecisions;
}

unsigned long timerLimit = 1000;
// Timer perpendicularTimer;
Timer keepSearchingTimer;

MovementDir sweepDirection;

bool isLineClose = false;
int sweepPhase = 0;

inline bool LineTrackingLogic() {

  bool madeAnyDecisions = false;
  SensorFun::Read_LineTracking_Sensors();
  if ( Line_IR_Middle && Line_IR_Right && Line_IR_Left && !isLineClose ) {
    Serial.println("Perpendicular ");
    MovementFun::Stop();
    // changeLedColor(CRGB::Blue);  
    delay(300);
    MovementFun::TurnRight45(-10);
    sweepDirection = MovementDir::turnR;
    isLineClose = true;
    madeAnyDecisions = true;
  }
  // else if ( Line_IR_Middle && Line_IR_Right ) {
  //   changeLedColor(CRGB::Green);  
  //   MovementFun::TurnRight(-10);
  //   MovementFun::Forward();
  //   madeAnyDecisions = true;
  // } 
  // else if ( Line_IR_Middle && Line_IR_Left ) {
  //   changeLedColor(CRGB::Green);
  //   MovementFun::TurnLeft(-10);
  //   MovementFun::Forward();
  //   madeAnyDecisions = true;
  // }

  // Line following logic
  if (Line_IR_Left || Line_IR_Middle || Line_IR_Right) {
    Serial.println("found Line:");
    madeAnyDecisions = true;
    isLineClose = true;
    keepSearchingTimer.m_ResetSetting();
    // On the line
    if (Line_IR_Middle == HIGH) {
      Serial.print("Forward");
      // changeLedColor(CRGB::White);  
      sweepDirection = MovementDir::forward;
      MovementFun::Forward();
      
      sweepPhase = 0;
    }
    // Turn right
    else if (Line_IR_Right == HIGH) {
      Serial.print("Right");
      // changeLedColor(CRGB::Purple);
      sweepDirection = MovementDir::turnR;
      MovementFun::TurnRight(10);
      sweepPhase = 0;
    }
    // Turn left
    else if (Line_IR_Left == HIGH) {
      Serial.print("Left");
      // changeLedColor(CRGB::Purple);
      sweepDirection = MovementDir::turnL;
      MovementFun::TurnLeft();
      sweepPhase = 0;
    }
  }

  else if ((isLineClose) && (sweepDirection != MovementDir::forward)) {
    Serial.print("SEARCHING:");
    // Timer Function
    keepSearchingTimer.m_Start();
    
    if(sweepPhase == 0){  
      // Serial.println("  Phase 1");      
      // changeLedColor(CRGB::Red, 100);
      if (!keepSearchingTimer.m_TimerDone(timerLimit)) {
        madeAnyDecisions = true;
        if (sweepDirection == MovementDir::turnL)   MovementFun::TurnLeft();
        else  MovementFun::TurnRight(10);
      }
      else{
        sweepPhase = 1;
        keepSearchingTimer.m_ResetSetting();
      }
    }
    if(sweepPhase == 1){
      // changeLedColor(CRGB::Green, 100);
      if (!keepSearchingTimer.m_TimerDone(2.0*timerLimit)) {
        madeAnyDecisions = true;
        if (sweepDirection == MovementDir::turnL)   MovementFun::TurnRight(10);
        else  MovementFun::TurnLeft();
      }
      else{
        sweepPhase = 2;
        isLineClose = false;
        Serial.println("Done");
      }
    } 
  } 
  
  else {
    Serial.println("nothing");
    // changeLedColor(CRGB::Black, 0);
    isLineClose = false;
    keepSearchingTimer.m_ResetSetting();
  }


  return madeAnyDecisions;
}
// inline bool FlameTrackingLogic(){ // new
//   bool madeAnyDecisions = false;
//   SensorFun::Read_Flame_Sensors();
//   if (F_Flame_IR >= 21){
//     Serial.print("Flame Detected");
//     MovementFun::Forward();
//     madeAnyDecisions = true;
//     if (F_Flame_IR <= 7){
//      MovementFun::Stop();
//    }
//     if (F_Flame_IR == 0){
//       Serial.print("Flame NOT Detected");
//       MovementFun::Forward();
//     }
//   }
//   return madeAnyDecisions;
// }

bool closeEnough = false;
bool fanOn = false;

inline bool FlameTrackingLogic() {  // new
  bool madeAnyDecisions = false;
  fanOn = false;
  
  SensorFun::Read_Flame_Sensors();
  SensorFun::Read_ObjectAvoidance_Sensors();
  // Serial.print(Obj_Sharp_Front);
  
  if ((RM_Flame_IR == 0 || LM_Flame_IR == 0 || F_Flame_IR == 0) && Obj_Sharp_Front <= 10) {
    MovementFun::Stop();
    madeAnyDecisions = true;
    Serial.print("stopped \n");
    fanOn = true;
  }

  else if (F_Flame_IR == 0) {
    Serial.print("Flame Detected \n");
    MovementFun::Forward();
    madeAnyDecisions = true;
    // fanOn = true;
  }

  // else if (R_Flame_IR == 0) {
  //   Serial.print("Flame Detected on Right \n");
  //   MovementFun::TurnRight();
  //   madeAnyDecisions = true;
  //   // fanOn = true;
  // }

  else if ((R_Flame_IR == 0) || (RM_Flame_IR == 0)) {
    Serial.print("Flame Detected on Right \n");
    MovementFun::TurnRight();
    madeAnyDecisions = true;
    // fanOn = true;
  }

  // Check for L_Flame_IR and turn left45
  else if ((L_Flame_IR == 0) || (LM_Flame_IR == 0)) {
    Serial.print("Flame Detected on Left \n");
    MovementFun::TurnLeft();
    madeAnyDecisions = true;
    // fanOn = true;
  }
  return madeAnyDecisions;
}



// ---------------------------------------------------------
//                      Base Functions
// ---------------------------------------------------------

void setup() {
  Serial.begin(9600);

  // Initialize motor pins as outputs
  MovementFun::Init();

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS); // Initialize FastLED
  FastLED.setBrightness(BRIGHTNESS); // Set the initial brightness
  changeLedColor(CRGB::Black, 0);  // Set to black with brightness 0

  // -----------------------------------

  SensorFun::Init();

  Serial.println("-------------");
  Serial.println("Starting Program");
  Serial.println("-------------");


  MovementFun::Forward();
  digitalWrite(Fan_relay, HIGH);
  //EdgeAvoidanceLogic();
}

bool flameDetectionHappened, edgeAvoidanceHappened, objectAvoidanceHappened;
MovementDir rememberDir;

Timer lineTrackingBufferTimer;
unsigned long lineTrackingBufferTimerLimit = 500.0;
bool lineTrackingBufferActive = false;

void temp(){

  // edgeAvoidanceHappened = EdgeAvoidanceLogic();
  // if(edgeAvoidanceHappened){
  //   // changeLedColor(CRGB::Yellow); // Change to Yellow
  // }


  
  rememberDir = direction;

  objectAvoidanceHappened = ObjectAvoidanceLogic();
  if(objectAvoidanceHappened){
    // changeLedColor(CRGB::HotPink); // Change to HotPink
  }

  if ((rememberDir != direction) && (rememberDir == MovementDir::turnL || rememberDir == MovementDir::turnR) && (direction == MovementDir::turnR || direction == MovementDir::turnL)) {
    MovementFun::Backward();
  }

  // ------- Line Detection Buffer--------
  
  if (edgeAvoidanceHappened || objectAvoidanceHappened) {
    // start the timer for Buffer that 
    lineTrackingBufferActive = true;
    lineTrackingBufferTimer.m_ResetSetting();
    // keepSearchingTimer.m_ResetSetting();
    // isLineClose=false;
  }
  else {
    // is Buffer Suppose to wait
    if (lineTrackingBufferActive) {
      changeLedColor(CRGB::Orange);
      lineTrackingBufferTimer.m_Start();
      if (lineTrackingBufferTimer.m_TimerDone(lineTrackingBufferTimerLimit)) {
        lineTrackingBufferTimer.m_ResetSetting();
        lineTrackingBufferActive = false;
      }
    } 
    else {

      if (!LineTrackingLogic()) {
        if(direction == MovementDir::stop){
          MovementFun::Forward();
        }
        // if ((direction == MovementDir::turnR) || (direction == MovementDir::turnL)) {
        //   MovementFun::Forward();
        // }
      }
      else{
        // changeLedColor(CRGB::Blue); // Change to Red
      }
      // if(direction != MovementDir::backward){
      //   MovementFun::Forward();
      // }
    }
  }
}


Timer object_edgeAvoidanceBufferTimer;
unsigned long object_edgeAvoidanceBufferTimerLimit = 1500.0;
bool object_edgeAvoidanceBufferActive = false;

void loop() {
    // Serial.println(Obj_Sharp_Front);

  // SensorFun::Read_EdgeAvoidance_BackSensors();
  // SensorFun::Read_EdgeAvoidance_FrontSensors();
  SensorFun::Read_ObjectAvoidance_Sensors();
  //SensorFun::Read_LineTracking_Sensors();
  SensorFun::Print(false, false, false, false);

  // Change to false to only print sensor output and disable output

  if (true) {
    // changeLedColor(CRGB::Black, 0);  // Set to black with brightness 0
    if (fanOn)
    {
      digitalWrite(Fan_relay, LOW);
      delay(4000);
      
    }
    else {
      digitalWrite(Fan_relay, HIGH);
      fanOn = false;
    }
    if (FlameTrackingLogic()) {
      // start the timer for Buffer that 
      // changeLedColor(CRGB::Red); // Change to Red
      object_edgeAvoidanceBufferActive = true;
      object_edgeAvoidanceBufferTimer.m_ResetSetting();
    }
    else {
      // is Buffer Suppose to wait
      if (object_edgeAvoidanceBufferActive) {
        object_edgeAvoidanceBufferTimer.m_Start();
        if (object_edgeAvoidanceBufferTimer.m_TimerDone(object_edgeAvoidanceBufferTimerLimit)) {
          object_edgeAvoidanceBufferTimer.m_ResetSetting();
          object_edgeAvoidanceBufferActive = false;
        }
      } 
      else {
        temp();
      }
    }
    
  } 
  else {
    MovementFun::Stop();
  }
}

//line tracking
//IR sensor
//add fan
