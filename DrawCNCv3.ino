
/*
 * DrawCNC v3 - Drawing robot configured for Nano CNC Shield V4.0+A4988
 */

#include <EEPROM.h>
#include <Servo.h>

const char appVersion[] = "3.2d";      // Version identification

// H/W pin assignments
#define MOTOR_ENABLE_PIN        8       // Enable steppers
#define X_AXIS_DIR_PIN          5       // X-Axis control
#define X_AXIS_STEP_PIN         2
#define Y_AXIS_DIR_PIN          6       // Y-Axis control
#define Y_AXIS_STEP_PIN         3
#define X_AXIS_LIMIT_PIN        9       // Axis limit switches
#define Y_AXIS_LIMIT_PIN        10
#define PEN_CONTROL_PIN         12      // Pen servo signal pin
#define BTN_A_PIN               11      // General-purpose case buttons
#define BTN_B_PIN               13


typedef struct {
  int dirPin;       // Direction control pin
  int stepPin;      // Stepping pin
  int posDir;       // Value for dirPin which increases value along axis
  int negDir;       // Value for dirPin which decreases value along axis
} MotorConfig_t;

// X & Y-axis dimensions
#define STEPS_PER_MM    25.1
#define X_MAX           530     // mm
#define Y_MAX           480     // mm

typedef struct {
  char label[10];          // identifying label
  MotorConfig_t motor;     // motor config
  long maxLoc;             // max. step position (i.e. range is 0 to maxLoc)
  float stepsPerMM;        // steps required to move 1 mm
  int limitPin;            // Limit switch pin
} AxisConfig_t;

AxisConfig_t xAxisConfig = { "X-axis", { X_AXIS_DIR_PIN, X_AXIS_STEP_PIN, LOW, HIGH },
                              X_MAX * STEPS_PER_MM, STEPS_PER_MM, X_AXIS_LIMIT_PIN };
AxisConfig_t yAxisConfig = { "Y-axis", { Y_AXIS_DIR_PIN, Y_AXIS_STEP_PIN, HIGH, LOW },
                              Y_MAX * STEPS_PER_MM, STEPS_PER_MM, Y_AXIS_LIMIT_PIN };


/******************** AXIS Class *********************/

const long HOMING_ZERO_OFFSET = 3L;        // Distance from home limit to axis "zero" (in mm)
const unsigned int HOMING_RATE = 600;      // Fixed step delay during HOMING (uSec/step)

typedef enum {
   ST_RESET = 0,
   ST_HOMING,
   ST_ZEROING,
   ST_READY
} AxisState_t;

const char *axisStateLabel[] = { "RESET", "HOMING", "ZEROING", "READY" };

class Axis {
  AxisConfig_t *config;        // configuration data
  AxisState_t axisState;
  long curLoc;                 // current position (in steps) (-1 == UNKNOWN)
  long targetLoc;              // target position (in steps)
  int stepState;               // current stepping state (LOW/HIGH)

public:
  Axis(AxisConfig_t *cfg) {
    config = cfg;
    axisState = ST_RESET;
    curLoc = targetLoc = 0;
    stepState = LOW;

    pinMode(cfg->motor.dirPin, OUTPUT);
    pinMode(cfg->motor.stepPin, OUTPUT);
    pinMode(cfg->limitPin, INPUT_PULLUP);

    digitalWrite(cfg->motor.dirPin, cfg->motor.posDir);
    digitalWrite(cfg->motor.stepPin, stepState);
  }

  float getStepsPerMM() {
    return config->stepsPerMM;
  }

  AxisState_t getState() {
    return axisState;
  }

  long deltaDist() {
    return (targetLoc - curLoc);
  }

  boolean isReady() {
    switch (axisState) {
      
    case ST_HOMING:
      if (! atLimit()) {
        step();
      } else {
        curLoc = 0;
        initMove(HOMING_ZERO_OFFSET, true);
        axisState = ST_ZEROING;
      }
      break;
      
    case ST_ZEROING:
      if (curLoc < targetLoc) {
        step();
      } else {
        curLoc = targetLoc = 0;
        axisState = ST_READY;
      }
      break;
      
    case ST_READY:
      return true;
    }

    return false;
  }

  void initMove(float mm, boolean absMove) {
    long steps = mm * config->stepsPerMM;

    if (absMove) {
      targetLoc = steps;              // Absolute move
    } else {
      targetLoc = curLoc + steps;     // Relative move
    }

    // Check parms. are within limits
    targetLoc = constrain(targetLoc, 0, config->maxLoc);

    // Set movement direction
    if (targetLoc > curLoc) {
      digitalWrite(config->motor.dirPin, config->motor.posDir);
    } else {
      digitalWrite(config->motor.dirPin, config->motor.negDir);
    }
  }

  boolean step() {
    
    if (stepState == LOW) {   // Move only on the rising edge
      stepState = HIGH;
      if (curLoc < targetLoc) {
        curLoc++;
      } else {
        curLoc--;
      }
    } else {
      stepState = LOW;
    }

    // Do the step!
    digitalWrite(config->motor.stepPin, stepState);

    // Did we actually move?
    return (stepState == HIGH);
  }

  initHoming() {
    axisState = ST_HOMING;
    digitalWrite(config->motor.dirPin, config->motor.negDir);   // Set direction towards limit switch
  }

  boolean isIdle() {
    return (curLoc == targetLoc);
  }

  boolean atLimit() {
    return (digitalRead(config->limitPin) == LOW);
  }
  
  void dumpInfo() {
    Serial.print(config->label);
    Serial.print(F(": state = "));    
    Serial.print(axisStateLabel[axisState]);
    Serial.print(F(", curLoc = "));
    Serial.print(curLoc);
    Serial.print(F(", targetLoc = "));
    Serial.println(targetLoc);
  }

};

Axis xAxis(&xAxisConfig);
Axis yAxis(&yAxisConfig);


/******************** TOOL Class *********************/

typedef struct {
  char label[10];           // identifying label
  int controlPin;           // servo control pin
  int downAngle;            // "Down position" angle (0 - 180 degrees)
  int upAngle;              // "Up position" angle (o - 180 degrees)
} ToolConfig_t;

ToolConfig_t toolConfig = { "Pen", PEN_CONTROL_PIN, 62, 120};

class Tool {
  ToolConfig_t *config;       // configuration data
  Servo toolServo;            // Tool servo
  int curPos;                 // current position (up/down angle)

public:
  Tool(ToolConfig_t *cfg) {
    config = cfg;
    curPos = cfg->downAngle;
  }

  void enable() {
    toolServo.attach(config->controlPin);
    toolServo.write(config->upAngle);
  }

  void disable() {
    toolServo.write(config->upAngle);
    toolServo.detach();
  }

  void moveUp() {
    curPos = config->upAngle;
    toolServo.write(curPos);
    delay(300);
  }

  void moveDown() {
    curPos = config->downAngle;
    toolServo.write(curPos);
    delay(300);
  }
  
  void dumpInfo() {
    Serial.print(config->label);
    Serial.print(F(": CurPos = "));
    Serial.println(toolServo.read());
    //Serial.println((curPos == config->upAngle) ? "UP" : "DOWN");
  }
};

Tool pen(&toolConfig);


/******************** DRAWBOT Class *********************/

typedef struct {
  unsigned int maxStep;     // max. # of steps
  unsigned int stepRate;    // step rate to max. # of steps (velocity)
} AccelConfig_t;

// Acceleration/deceleration curve
AccelConfig_t accelTable[] = {
    { 10, 1800 },
    { 30, 600 },
    { 100, 285 },
    { 150, 185 },
    { 300, 125 },
};

#define ACCEL_TBL_SIZE  (sizeof(accelTable) / sizeof(AccelConfig_t))

class DrawBot {
  boolean absMode;        // Measurement mode: TRUE = absolute, FALSE = relative
  boolean busy;           // Busy doing work?
  boolean motorsEnabled;  // Motors enabled?
  unsigned int moveDelay; // Time delay between moves (uSecs/step)
  long deltaX, deltaY;    // line-drawing X/Y distances
  int dirX, dirY;         // line drawing X/Y directions
  long totMoves;          // Total # of moves in current line (for acceleration algorithm)
  long curMove;           // Current move in current line (for acceleration algorithm)
  long remainder;         // For Bresenham's drawing algorithm
  unsigned long prevStepTime;
  
public:
  DrawBot() {
    absMode = false;      // Default is relative mode
    busy = false;
    moveDelay = HOMING_RATE;  // Setup for HOMING
  }

  void setAbsMode(boolean val) {
    absMode = val;
  }

  boolean getAbsMode() {
    return (absMode);
  }

  void setBusy() {
    busy = true;
  }

  void setMoveDelay(float val) {
    moveDelay = 60000000 / (val * xAxis.getStepsPerMM());   // Convert mm/minute to uSecs/step
  }
  
  void ready() {
    if (busy) {
      Serial.println(">");
      busy = false;
    }
  }
  
  void motorsOn() {
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTOR_ENABLE_PIN, LOW);
    pen.enable();
    motorsEnabled = true;
  }
  
  void motorsOff() {
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTOR_ENABLE_PIN, HIGH);
    pen.disable();
    motorsEnabled = false;
  }  
  
  void dumpInfo() {
    Serial.print(F("BOT: vers. = "));
    Serial.print(appVersion);
    Serial.print(F(", absMode = "));
    Serial.print(absMode);
    Serial.print(F(", motorsEnabled = "));
    Serial.print(motorsEnabled);
    Serial.print(F(", moveDelay = "));
    Serial.println(moveDelay);
  }

  void initMove() {
    deltaX = xAxis.deltaDist();
    deltaY = yAxis.deltaDist();

    dirX = (deltaX > 0) ? 1 : -1;
    dirY = (deltaY > 0) ? 1 : -1;

    deltaX = abs(deltaX);
    deltaY = abs(deltaY);
    
    totMoves = deltaX + deltaY;
    curMove = 0;
    moveDelay = accelTable[0].stepRate;   // was commented out...

    remainder = 0;
    prevStepTime = 0;
    busy = true;
  }

  initHoming() {
    moveDelay = HOMING_RATE;
    busy = true;
  }
  
  void updatePos() {
    unsigned long curTime = micros();

    // If the current move delay has expired...
    if ((curTime - prevStepTime) >= moveDelay) {

      // ...and if both axis are ready...
      if (xAxis.isReady() && yAxis.isReady()) {
          
        // ...and if at least one axis has moves remaining...
        if (! xAxis.isIdle() || ! yAxis.isIdle()) {

          // Run Bresenham's algorithm
          if (deltaX > deltaY) {
            if (xAxis.step()) {
              remainder += deltaY;
              curMove++;
            }
            if (remainder >= deltaX) {
              if (yAxis.step()) {
                remainder -= deltaX;
                curMove++;
              }
            }
          } else {
            if (yAxis.step()) {
              remainder += deltaX;
              curMove++;
            }
            if (remainder >= deltaY) {
              if (xAxis.step()) {
                remainder -= deltaY;
                curMove++;
              }
            }
          }
  
          // Accelerate/decelerate depending upon location on line
          if (curMove <= totMoves / 2) {
            moveDelay = accelerate(curMove);
          } else {
            moveDelay = accelerate(totMoves - curMove);
          }
        } else {

          // Since no movement required, signal idle
          ready();
        }
      }
      
      // Reset delay timer
      prevStepTime = curTime;
    }
  }

  unsigned int accelerate(long pos) {
    for (int i = 0; i < ACCEL_TBL_SIZE; i++) {
      if (pos < accelTable[i].maxStep) {
        return (accelTable[i].stepRate);
      }
    }
    return (accelTable[ACCEL_TBL_SIZE - 1].stepRate);
  }
};

DrawBot bot;


/******************** Command Parameter Processing *********************/

class CmdParms {
  float xLoc;         // X-Axis position (in mm)
  boolean xValid;
  float yLoc;         // Y-Axis position (in mm)
  boolean yValid;
  float zLoc;         // Z-Axis position
  boolean zValid;
  float rate;         // movement rate (in mm/minute)
  boolean rateValid;
  int parmCount;      // # of parameters present

public:
  boolean xIsValid() {
    return xValid;
  }

  float getXVal() {
    return xLoc;
  }

  boolean yIsValid() {
    return yValid;
  }

  float getYVal() {
    return yLoc;
  }

  boolean zIsValid() {
    return (zValid);
  }

  float getZVal() {
    return zLoc;
  }

  boolean rateIsValid() {
    return rateValid;
  }

  float getRateVal() {
    return rate;
  }
  
  int parse(boolean withVals) {
    char *tok;
    float val;
    xValid = yValid = zValid = rateValid = false;

    parmCount = 0;
    while ((tok = strtok(NULL, " "))) {
      if (withVals) {
        val = atof(tok + 1);
      }
      int id = tok[0];
  
      switch(toupper(id)) {
      case 'X':
        if (withVals) {
          xLoc = val;
        }
        xValid = true;
        parmCount++;
        break;
        
      case 'Y':
        if (withVals) {
          yLoc = val;
        }
        yValid = true;
        parmCount++;
        break;
         
      case 'Z':
        if (withVals) {
          zLoc = val;
        }
        zValid = true;
        parmCount++;
        break;
          
      case 'F':
        if (withVals) {
          rate = val;
        }
        rateValid = true;
        parmCount++;
        break;
      }
    }

    return (parmCount);
  }
};

CmdParms cmdParms;


/******************** Command Callback Functions (used by Command Processor) *********************/


// CMD_MOVETO() - Initialize a tool move
boolean cmd_moveTo() {
  boolean busy = false;
  
  cmdParms.parse(true);
  
  // Initialize X-Axis for move
  if (cmdParms.xIsValid()) {
    xAxis.initMove(cmdParms.getXVal(), bot.getAbsMode());
    busy = true;
  }

  // Initialize Y-Axis for move
  if (cmdParms.yIsValid()) {
    yAxis.initMove(cmdParms.getYVal(), bot.getAbsMode());
    busy = true;
  }
  
  // Move Z-Axis (pen) immediately
  if (cmdParms.zIsValid()) {
    if (cmdParms.getZVal() > 0) {
      pen.moveUp();
    } else {
      pen.moveDown();
    }
  }

  // Initialize move rate
  if (cmdParms.rateIsValid()) {
    bot.setMoveDelay(cmdParms.getRateVal());
  }

  // Initialize bot for move
  bot.initMove();

  return (busy);
}


// CMD_MOVEHOME() - Home one or more axis
boolean cmd_moveHome() {
  int pc = cmdParms.parse(false);

  // Initialize X-Axis for move
  if (cmdParms.xIsValid() || pc == 0) {
    xAxis.initHoming();
  }

  // Initialize Y-Axis for move
  if (cmdParms.yIsValid() || pc == 0) {
    yAxis.initHoming();
  }
  
  // Always raise pen!
  pen.moveUp();

  // Initialize bot for HOMING
  bot.initHoming();

  // Always busy
  return (true);
}


// CMD_MOTORSON() - Energize axis motors
boolean cmd_motorsOn() {
  bot.motorsOn();
  return (false);   // not busy
}


// CMD_MOTORSOFF() - De-energize axis motors
boolean cmd_motorsOff() {
  bot.motorsOff();
  return (false);   // not busy
}


// CMD_ABSMODE() - Set absolute movement mode
boolean cmd_absMode() {

  // Absolute mode allowed only if both axis are zeroed (READY)
  if (xAxis.getState() == ST_READY && yAxis.getState() == ST_READY) {
    bot.setAbsMode(true);
  }
  return (false);   // not busy
}


// CMD_RELMODE() - Set relative movement mode
boolean cmd_relMode() {
  bot.setAbsMode(false);
  return (false);   // not busy
}


// CMD_DUMPINFO() - Dump internal state data to serial port
boolean cmd_dumpInfo() {
  bot.dumpInfo();
  xAxis.dumpInfo();
  yAxis.dumpInfo();
  pen.dumpInfo();
  return (false);   // not busy
}


/******************** Command Processor *********************/

typedef struct {
  char *cmd;          // Command keyword
  boolean (*func)();     // Callback function
  boolean immediate;  // Immediately ready for next command?
} CommandTable;

CommandTable cmdTable[] = {
  { (char *)"G0", &cmd_moveTo, false },
  { (char *)"G00", &cmd_moveTo, false },
  { (char *)"G1", &cmd_moveTo, false },
  { (char *)"G01", &cmd_moveTo, false },
  { (char *)"G28", &cmd_moveHome, false },
  { (char *)"G90", &cmd_absMode, true },
  { (char *)"G91", &cmd_relMode, true },
  { (char *)"M17", &cmd_motorsOn, true },
  { (char *)"M18", &cmd_motorsOff, true },
  { (char *)"?", &cmd_dumpInfo, true },
};

#define BUFSIZE   65

void processCmd() {
  char cmdBuf[BUFSIZE];
  int cmdCnt;
  char *tok;
  boolean busy = false;

  bot.setBusy();
  
  // Read command to newline and tokenize
  cmdCnt = Serial.readBytesUntil('\n', cmdBuf, BUFSIZE-1);
  cmdBuf[cmdCnt] = '\0';
  tok = strtok(cmdBuf, " ");

  // Lookup command in command table
  for (int i = 0; i < sizeof(cmdTable) / sizeof(CommandTable); i++) {
    if (strcmp(tok, cmdTable[i].cmd) == 0) {

      // Found match - execute command function
      busy = (*cmdTable[i].func)();

      // Clear busy status if command completed immediately
      if (! busy) {
        bot.ready();
      }

      // Return now
      return;
    }
  }

  // No command match - ready for next
  bot.ready();
}


/******************* BUTTON MANAGER Configuration ************/

typedef struct {
  char *id;                 // Button ID
  int pin;                  // Digital pin attached to button
  char *cmdstr;             // Command string to execute
} ButtonConfig_t;

ButtonConfig_t buttonConfigList[] = { { (char *)"A", BTN_A_PIN, "" },
                                      { (char *)"B", BTN_B_PIN, "" },
                                    };

#define BUTTON_TABLE_SIZE   sizeof(buttonConfigList) / sizeof(ButtonConfig_t)


/******************* BUTTON MANAGER Class ************/

class ButtonManager {
  ButtonConfig_t *btnList;
  int btnCount;
  
public:
  ButtonManager(ButtonConfig_t *cfg) {
    btnList = cfg;

    // Set all pins for input
    for (int i = 0; i < BUTTON_TABLE_SIZE; i++) {
      pinMode(cfg[i].pin, INPUT_PULLUP);
    }
  }

  int pressed() {
    for (int i = 0; i < BUTTON_TABLE_SIZE; i++) {
      if (digitalRead(btnList[i].pin) == LOW) {
         return (i);
      }
    }
    return (-1);
  }

  char *getID(int btn) {
     return (btnList[btn].id);
  }
};

ButtonManager btnMgr(buttonConfigList);


/******************** SETUP() *********************/
void setup() {
  
  Serial.begin(9600);
  while (!Serial) { ; }

  // Not ready yet
  bot.setBusy();

  // Disable motors
  bot.motorsOff();

  Serial.println(F("RESET"));

  // Signal ready for commands
  bot.ready();
}


/******************** LOOP() *********************/
void loop() {
   int i;

  // Move, as necessary
  bot.updatePos();

  // if command present, process it
  if (Serial.available() > 0) {
    processCmd();
  }

  // If command button pressed, acknowledge it
  if ((i = btnMgr.pressed()) >= 0) {
    Serial.println(btnMgr.getID(i));
  }
}
