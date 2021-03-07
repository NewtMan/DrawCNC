/*
 * DrawCNC v3 - Drawing robot configured for Nano CNC Shield V4.0+A4988
 */

#include <EEPROM.h>
#include <Servo.h>

const char appVersion[] = "4.0";		// Version identification

#define P_NO_VALUE				(-1)	// No value provided for command parameter
#define INVALID_DIR				(999)	// "Invalid" direction value

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

// X & Y-axis maximum dimensions
#define STEPS_PER_MM    25.1
#define X_MAX           530     // mm
#define Y_MAX           480     // mm

/******************* AXIS Configuration **************/
typedef struct {
	char label[10];          // identifying label
	MotorConfig_t motor;     // motor config
	long maxLoc;             // max. step position (i.e. range is 0 to maxLoc)
	float stepsPerMM;        // steps required to move 1 mm
	int limitPin;            // Limit switch pin
} AxisConfig_t;

AxisConfig_t xAxisConfig = { "X-axis", { X_AXIS_DIR_PIN, X_AXIS_STEP_PIN, LOW, HIGH },
                              (long)(X_MAX * STEPS_PER_MM), STEPS_PER_MM, X_AXIS_LIMIT_PIN };
AxisConfig_t yAxisConfig = { "Y-axis", { Y_AXIS_DIR_PIN, Y_AXIS_STEP_PIN, HIGH, LOW },
                              (long)(Y_MAX * STEPS_PER_MM), STEPS_PER_MM, Y_AXIS_LIMIT_PIN };


/******************** AXIS Class *********************/

const long HOMING_ZERO_OFFSET = 3L;        // Distance from home limit to axis "zero" (in mm)
const unsigned int HOMING_RATE = 500;      // Fixed step delay during HOMING (uSec/step)

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

	AxisState_t getState() {
		return axisState;
	}

	long getLoc() {
		return curLoc;
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
				initMove(HOMING_ZERO_OFFSET * config->stepsPerMM, true);
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

	void initMove(long steps, boolean absMove) {

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

	void initHoming() {
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

	bool isDown() {
		return (curPos == config->downAngle);
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
	long totMoves;          // Total # of moves in current line (for acceleration algorithm)
	long curMove;           // Current move in current line (for acceleration algorithm)
	long remainder;         // For Bresenham's drawing algorithm
	unsigned long prevStepTime;
  
public:
	DrawBot() {
		absMode = false;      		// Default is relative mode
		busy = false;				// Initialize to "idle"
		moveDelay = HOMING_RATE;	// Initialize for HOMING
	}

	void setAbsMode(boolean val) {
		absMode = val;
	}

	boolean getAbsMode() {
		return (absMode);
	}

	bool isBusy() {
		return (busy);
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

		deltaX = abs(deltaX);
		deltaY = abs(deltaY);
    
		totMoves = deltaX + deltaY;
		curMove = 0;
		moveDelay = accelTable[0].stepRate;

		remainder = 0;
		prevStepTime = 0;
		busy = true;
	}

	void initHoming() {
		moveDelay = HOMING_RATE;
		busy = true;
	}
  
	bool updatePos() {
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

					// Since no movement required, bot is idle - signal "done" if necessary
					if (busy) {
						busy = false;
						return (true);		// transitioning from busy to idle
					}
				}
			}
      
			// Reset delay timer
			prevStepTime = curTime;
		}

		// No "busy/idle" transition
		return (false);
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


/******************** Parsed Command Structure *****************/
typedef struct {
	char code[4];			// Command code
	int xLoc;				// X-axis parameter
	bool xValid;			// X parameter valid?
	int yLoc;				// Y-axis parameter
	bool yValid;			// y parameter valid?
	int zLoc;				// Z-axis parameter
	bool zValid;			// Z parameter valid?
	int dirTo;				// angle of line from prev. point to this one (move command, only)
	unsigned int stepsTo;	// Total steps in line from prev. point to this one (move command, only)
} ParsedCommand_t;


/******************** Command Callback Functions (used by Command Executive) *********************/

// CMD_MOVETO() - Initialize a tool move
boolean cmd_moveTo(ParsedCommand_t *cmd) {
	boolean busy = false;
  
	// Initialize X-Axis for move
	if (cmd->xValid) {
		xAxis.initMove(cmd->xLoc, bot.getAbsMode());
		busy = true;
	}

	// Initialize Y-Axis for move
	if (cmd->yValid) {
		yAxis.initMove(cmd->yLoc, bot.getAbsMode());
		busy = true;
	}
  
	// Move Z-Axis (pen) immediately
	if (cmd->zValid) {
		if (cmd->zLoc > 0) {
			pen.moveUp();
		} else {
			pen.moveDown();
		}
	}

	// Initialize bot for move
	bot.initMove();

	return (busy);
}


// CMD_MOVEHOME() - Home one or more axis
boolean cmd_moveHome(ParsedCommand_t *cmd) {

	// Initialize X-Axis for move
	if (cmd->xValid || (! cmd->xValid && ! cmd->yValid)) {
		xAxis.initHoming();
	}

	// Initialize Y-Axis for move
	if (cmd->yValid || (! cmd->xValid && ! cmd->yValid)) {
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
boolean cmd_motorsOn(ParsedCommand_t *cmd) {
	bot.motorsOn();
	return (false);   // not busy
}


// CMD_MOTORSOFF() - De-energize axis motors
boolean cmd_motorsOff(ParsedCommand_t *cmd) {
	bot.motorsOff();
	return (false);   // not busy
}


// CMD_ABSMODE() - Set absolute movement mode
boolean cmd_absMode(ParsedCommand_t *cmd) {

	// Absolute mode allowed only if both axis are zeroed (READY)
	if (xAxis.getState() == ST_READY && yAxis.getState() == ST_READY) {
		bot.setAbsMode(true);
	}
	return (false);   // not busy
}


// CMD_RELMODE() - Set relative movement mode
boolean cmd_relMode(ParsedCommand_t *cmd) {
	bot.setAbsMode(false);
	return (false);   // not busy
}


// CMD_DUMPINFO() - Dump internal state data to serial port
boolean cmd_dumpInfo(ParsedCommand_t *cmd) {
	bot.dumpInfo();
	xAxis.dumpInfo();
	yAxis.dumpInfo();
	pen.dumpInfo();
	return (false);   // not busy
}


/******************** Command Table *********************/

typedef struct {
	char *cmd;          // Command keyword
	boolean (*func)(ParsedCommand_t *);     // Callback function
} CommandTable;

CommandTable cmdTable[] = {
	{ (char *)"G0", &cmd_moveTo},
	{ (char *)"G00", &cmd_moveTo},
	{ (char *)"G1", &cmd_moveTo},
	{ (char *)"G01", &cmd_moveTo},
	{ (char *)"G28", &cmd_moveHome},
	{ (char *)"G90", &cmd_absMode},
	{ (char *)"G91", &cmd_relMode},
	{ (char *)"M17", &cmd_motorsOn},
	{ (char *)"M18", &cmd_motorsOff},
	{ (char *)"?", &cmd_dumpInfo},
};


#define CMD_QUEUE_SIZE	10

/******************** COMMAND QUEUE Class *************************/
class CmdQueue {
	int head, tail, count, nidx;
	ParsedCommand_t queue[CMD_QUEUE_SIZE];

public:
	CmdQueue() {
		head = tail = count = 0;
	}

	bool enqueue(ParsedCommand_t *cmd) {
		if (count < CMD_QUEUE_SIZE) {
			queue[tail++] = *cmd;
			if (tail >= CMD_QUEUE_SIZE) {
				tail = 0;
			}
			count++;
			return true;
		} else {
			return false;
		}
	}

	ParsedCommand_t *dequeue() {
		ParsedCommand_t *ptr = queue + head;
		if (count > 0) {
			if (++head >= CMD_QUEUE_SIZE) {
				head = 0;
			}
			count--;
			return ptr;
		} else {
			return (ParsedCommand_t *)NULL;
		}
	}

	ParsedCommand_t *first() {
		if (count > 0) {
			nidx = head;
			return (queue + nidx);
		}
		return (ParsedCommand_t *)NULL;
	}

	ParsedCommand_t *next() {
		if (count > 0) {
			if (++nidx >= CMD_QUEUE_SIZE) {
				nidx = 0;
			}
			if (nidx != tail) {
				return (queue + nidx);
			}
		}
		return (ParsedCommand_t *)NULL;
	}


	int getCount() {
		return count;
	}

	bool isFull() {
		return (count >= CMD_QUEUE_SIZE);
	}
};

CmdQueue cmdQ;

/******************** COMMAND READER Class ************************/
#define BUFSIZE		65

class CmdReader {
	ParsedCommand_t cmdIn;
	bool waiting;

public:
	CmdReader() {
		waiting = false;
	}

	void ready() {
		if (! waiting) {
			Serial.print(cmdQ.getCount());
			Serial.println(">");
			waiting = true;
		}
	}

	bool checkForInput() {
		char cmdBuf[BUFSIZE];
		char *tok;
		int p_id;
		int p_val;
		cmdIn.xValid = cmdIn.yValid = cmdIn.zValid = false;

		// if command present and there's space in the command queue...
		if (Serial.available() > 0 && ! cmdQ.isFull()) {

			// No longer waiting for input
			waiting = false;
  
			// Read command to newline and tokenize
			int c = Serial.readBytesUntil('\n', cmdBuf, BUFSIZE-1);
			cmdBuf[c] = '\0';
			tok = strtok(cmdBuf, " ");

			// Lookup command in command table
			for (int i = 0; i < sizeof(cmdTable) / sizeof(CommandTable); i++) {
				if (strcmp(tok, cmdTable[i].cmd) == 0) {

					// Found valid command - store command code & parse parameters
					strcpy(cmdIn.code, tok);
					while ((tok = strtok(NULL, " "))) {
						p_id = tok[0];

						// If there's a parm value, get it
						if (strlen(tok) > 1) {
							p_val = atof(tok + 1);
						} else {
							p_val = P_NO_VALUE;
						}

							// Store parameter
							switch(toupper(p_id)) {
							case 'X':
								cmdIn.xLoc = xAxisConfig.stepsPerMM * p_val;
								cmdIn.xValid = true;
								break;
				
							case 'Y':
								cmdIn.yLoc = yAxisConfig.stepsPerMM * p_val;
								cmdIn.yValid = true;
							break;

							case 'Z':
								cmdIn.zLoc = p_val;
								cmdIn.zValid = true;
								break;
						}
					}

					// "Clear" the line analysis fields
					cmdIn.stepsTo = 0;
					cmdIn.dirTo = INVALID_DIR;

					// Store parsed command in command buffer
					cmdQ.enqueue(&cmdIn);

					// Report new command received
					return (true);
				}
			}
		}

		// Report no new command
		return (false);
    }
};

CmdReader cmdReader;


/******************** COMMAND EXECUTIVE Class ************************/
class CmdExecutive {

	void calcLine(ParsedCommand_t *cmd, int x1, int y1) {
		cmd->stepsTo = (abs(cmd->xLoc - x1) + abs(cmd->yLoc - y1)) * STEPS_PER_MM;
		if (x1 == cmd->xLoc) {
			cmd->dirTo = 90;
		} else {
			cmd->dirTo = round(atan((float)(cmd->yLoc - y1) / (float)(cmd->xLoc - x1)) * 4068.0 / 71.0);
		}
	}

public:
	CmdExecutive() {
	}

	bool nextCommand() {
		ParsedCommand_t *cmd;
		bool busy = false;

		// While commands are queued and we're not "busy"...
		while (! busy && cmdQ.getCount() > 0) {

			// Peek at next command
			cmd = cmdQ.first();

			// Lookup command in command table
			for (int i = 0; i < sizeof(cmdTable) / sizeof(CommandTable); i++) {
				if (strcmp(cmd->code, cmdTable[i].cmd) == 0) {

					// Execute command function
					busy = (*cmdTable[i].func)(cmd);

					// if an "immediate" command, pop it off queue so we can move on
					if (! busy)	{
						(void) cmdQ.dequeue();
					}
					break;		// terminate lookup
				}
			}
		}

		return (busy);
	}

	void analyzeLines() {
		ParsedCommand_t *last, *cmd;
		bool penDown = pen.isDown();

		// No need for analysis if pen is up
		if (pen.isDown()) {

			// Calculate line from current bot location to current end point
			last = cmdQ.first();
			calcLine(last, xAxis.getLoc(), yAxis.getLoc());
			Serial.print("A:");
			Serial.print(last->dirTo);

			while ((cmd = cmdQ.next()) != (ParsedCommand_t *)NULL) {

				// Pen up command breaks the line sequence
				if (cmd->zValid && cmd->zLoc == 1) {
					break;

				// Otherwise, calculate next line
				} else if (cmd->xValid && cmd->yValid) {
					calcLine(cmd, last->xLoc, last->yLoc);
					Serial.print(",");
					Serial.print(last->dirTo);
				}
				last = cmd;
			}
			Serial.println("");
		}
	}
};

CmdExecutive cmdExec;


/******************* BUTTON MANAGER Configuration ************/

typedef struct {
	char *id;                 // Button ID
	int pin;                  // Digital pin attached to button
	bool isHigh;              // Latched state
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
			cfg[i].isHigh = true;
			pinMode(cfg[i].pin, INPUT_PULLUP);
		}
	}

	int pressed() {
		for (int i = 0; i < BUTTON_TABLE_SIZE; i++) {
			if (btnList[i].isHigh) {
				if (digitalRead(btnList[i].pin) == LOW) {
					delay(1);
					if (digitalRead(btnList[i].pin) == LOW) {
						btnList[i].isHigh = false;
						return (i);
					}
				}
			} else if (digitalRead(btnList[i].pin) == HIGH) {
				delay(1);
				if (digitalRead(btnList[i].pin) == HIGH) {
					btnList[i].isHigh = true;
				}
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
  
	Serial.begin(19200);
	while (!Serial) { ; }

	// Disable motors
	bot.motorsOff();

	Serial.println(F("RESET"));

	// Signal ready for commands
	cmdReader.ready();
}


/******************** LOOP() *********************/
void loop() {
	int i;

	// Move, as necessary
	if (bot.updatePos()) {

		// If Bot is now idle, dequeue command
		(void) cmdQ.dequeue();
	}

	// Look for & parse command input
	if (cmdReader.checkForInput()) {

		// New command received - run line analysis
		cmdExec.analyzeLines();
	}

	// If machine is idle, execute next command(s) (if any)
	if (! bot.isBusy()) {
		(void) cmdExec.nextCommand();
	}

	// If command buffer not full, ready for more commands
	if (! cmdQ.isFull()) {
		cmdReader.ready();
	}

	// If command button pressed, acknowledge it
	if ((i = btnMgr.pressed()) >= 0) {
		Serial.println(btnMgr.getID(i));
	}
}
