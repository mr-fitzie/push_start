/**
 * Push Button Start 
 * 
 * Simple circuit that implements a push button start system for an automobile
 * The system has 4 distinct states
 * * DISABLED - Vehicle is powered off and cannot be turned on
 * * OFF - Vehicle is powered off
 * * ACC - Vehicle is in accessory mode
 * * START - Vehicle is starting
 * * RUN - Vehicle is running
 * 
 * The system uses 4 relays or FETs to drive the existing vehicle start circuits
 * When the system is:
 * * DISABLED the relays are deenergized/OFF and cannot be enabled
 * * OFF all relays are deenergized/OFF
 * * ACC the RELAY_ACC is energized/ON
 * * START the RELAY_START and RELAY_IGN are energized/ON
 * * RUN the RELAY_IGN is energized/ON
 * 
 * If the KILL_SWITCH circuit is grounded then the whole system is DISABLED and the relays are
 * immediately deenergized and the button no longer can transition states. If the circuit is
 * opened the system transitons to the OFF state.
 * 
 * When the kill switch is closed the system is enabeled and has the basic flow of:
 * OFF -> ACC -> (OFF | START) -> RUN -> (OFF | ACC)
 * * To transition from OFF -> ACC depress and release the push button
 * * To transition from ACC -> OFF depress and release the push button under HOLD_MS (1sec)
 * * To transition from ACC -> (START) -> RUN depress and hold the push button in until 
 * * vehicle is started and then release the button. The initial depress starts a timer that
 * * once held for over HOLD_MS (1sec) puts the system in START mode and then the release
 * * of the button once the vehicle is started transitions to RUN mode. Note, if the 
 * * SAFE_SWITCH is grounded the this transition will not be conducted.
 * * To transition from RUN -> OFF depress and release the push button.
 * * To transition from RUN -> ACC depress and hold the push button longer than HOLD_MS 
 * * (1sec) then release.
 * 
 * Because the timing to kick over the motor can be different for various vehicles and you
 * do not want not run the starter too long which may grind the flywheel/flexplate teeth the
 * I decided to keep the amount of time the starter is engaged to the actual press, just like
 * turning a key. The system could be modified to do it based on time in the future.
 * 
 * @author Chris Fitzpatrick
 * @version 1.0 02/11/2017
 * @license MIT License
 */

// Pin Definitions

// LEDs
const int LED_BUTTON   = 9;  // Push button LED
const int LED_ON_BOARD = 13; // Onboard will not fade because there is no PWM attached,
                             // but can at least give indication of state when not  
                             // using 12v input for testing/debug
// Switches
const int PUSH_BUTTON = 2;   // Momentary push button
const int KILL_SWITCH = 3;   // This is any input to enable/disable the system, could be NFC 
                             // trigger/physical switch, if grounded system is disabled.
const int SAFE_SWITCH = 4;   // Brake/Clutch/Neutral safety switch, if grounded cannot start.
                             
// Relays
const int RELAY_ACC   = 5;
const int RELAY_IGN   = 6;
const int RELAY_START = 7;
const int RELAY_4     = 8;   // Not Used

// System state
enum sysState {
  DISABLED,
  OFF,
  ACC,
  START,
  RUN
};

// Globals
const int FLASH_DELAY = 10;    // How many ticks to pass through before turning on/off LED
                               // when in ACC state. Increasing this number makes the blink
                               // slower. Since the main loop sleeps 30ms a value of 10 here
                               // waits 300ms in ON then 300ms in OFF.
                                                          
const int HOLD_MS = 1000;      // Milliseconds to hold button to transition from ACC->START 
                               // or from RUN->ACC                             
                 
unsigned int tick = 0;         // Count cycles for LED flash mode (unsigned so when it rolls
                               // over it restarts at 0
int brightness = 0;            // How bright the LED is (usable range 0-255)
int fadeAmount = 5;            // How many points to fade the LED by when in OFF state  

int previousButtonState = LOW; // Stores the previous button state so we can see if it changes
long pressTime;                // When the button was pressed 

sysState runState = OFF;       // Stores current system run state

/**
 * Setup runs once when system is reset, Setup pins and default states
 */
void setup() {
  // Initialize serial communication at 9600 bits per second
  Serial.begin(9600);
  
  // Setup Pins
  pinMode(LED_BUTTON, OUTPUT);
  pinMode(LED_ON_BOARD, OUTPUT);
  
  pinMode(PUSH_BUTTON, INPUT_PULLUP);
  pinMode(KILL_SWITCH, INPUT_PULLUP);
  pinMode(SAFE_SWITCH, INPUT_PULLUP); 
  
  pinMode(RELAY_ACC, OUTPUT);
  pinMode(RELAY_IGN, OUTPUT);
  pinMode(RELAY_START, OUTPUT);

  // Disable relays at start
  // HIGH == Denergized/OFF, LOW = Energized/ON
  digitalWrite(RELAY_ACC, HIGH);
  digitalWrite(RELAY_IGN, HIGH);
  digitalWrite(RELAY_START, HIGH);

  Serial.println("+------ Push Button Start System v1.0 ------+");
  Serial.println("| MIT License                               |");
  Serial.println("| Copyright (c) 2016-2017 Chris Fitzpatrick |");
  Serial.println("| https://github.com/synthtc/push_start     |");
  Serial.println("+-------------------------------------------+");
  printState(runState);
}

/**
 * Main Run Loop
 */
void loop() {
  // Set the brightness of LEDs:
  analogWrite(LED_BUTTON, brightness);
  analogWrite(LED_ON_BOARD, brightness);
  
  // System is enabled, process as normal
  if (digitalRead(KILL_SWITCH) == HIGH) { // HIGH = System Enabled LOW = Disabled (grounded)
    // If we were disabled move to OFF
    if (runState == DISABLED) {
      runState = OFF;
      printState(runState);
    }
    // Read in the button state to see if it has been pressed
    int buttonState = digitalRead(PUSH_BUTTON);
    if (buttonState != previousButtonState) {
      // Transition on LOW (released) state for most states
      if (buttonState == LOW) { // Released
        Serial.println("Released");
        switch (runState){
          case DISABLED: // Disabled -> Off
            // This state should never be hit, but is included for completeness.
            runState = OFF;
            break;
          case OFF: // Off -> Acc
            runState = ACC;
            break;
          case ACC: // Acc -> Off (Acc -> Start handled below)
            runState = OFF;
            break;
          case START: // Start -> Run
            runState = RUN;
            break;
          case RUN: // Run -> Off or ACC
            //Serial.println(millis() - pressTime);
            // Transition back to ACC if button was held longer than HOLD_MS otherwise
            // go to OFF
            if (millis() - pressTime > HOLD_MS) {
              runState = ACC;  
            } else {
              runState = OFF;
            }
            break;
        }
        printState(runState);
        setRelayState();
      } else { // Pressed
        pressTime = millis();
        Serial.println("Pressed");
      } 
    } else if (buttonState == HIGH) { // Pressed
      // Special case for START
      // When we release the push button in the ACC state we want to transition to
      // START if held longer than HOLD_MS and the SAFE_SWITCH is open
      if (runState == ACC) {
        bool safe = digitalRead(SAFE_SWITCH) == HIGH; // HIGH = Safe LOW = Disabled (grounded)
        if (safe && millis() - pressTime > HOLD_MS) {
          runState = START;
          printState(runState);
          setRelayState();
        } else {
          Serial.println("WARNING: Unsafe to Start");
        }
      }
    }
    // Store button state for next cycle
    previousButtonState = buttonState;
  } else {
    if (runState != DISABLED) {
      printState(DISABLED);
    }
    runState = DISABLED;
    setRelayState();
  }
  // Set brightness for next cycle
  setLedBrigtness();
  // Wait for 30 milliseconds
  delay(30);
}

/**
 * Sets the relays depending on the current system run state
 * 
 * DISABLED - All Off
 * OFF      - All Off
 * ACC      - ACC On
 * START    - ACC On IGN On
 * RUN      - IGN On
 * 
 * @return none
 */
void setRelayState() {
  switch (runState){
    case DISABLED: 
    case OFF:
      digitalWrite(RELAY_ACC, HIGH);
      digitalWrite(RELAY_IGN, HIGH);
      digitalWrite(RELAY_START, HIGH);
      break;
    case ACC:
      digitalWrite(RELAY_ACC, LOW);
      digitalWrite(RELAY_IGN, HIGH);
      digitalWrite(RELAY_START, HIGH);
      break;
    case START:
      digitalWrite(RELAY_ACC, HIGH);
      digitalWrite(RELAY_IGN, LOW);
      digitalWrite(RELAY_START, LOW);
      break;
    case RUN:
      digitalWrite(RELAY_ACC, HIGH);
      digitalWrite(RELAY_IGN, LOW);
      digitalWrite(RELAY_START, HIGH);
      break;
  }
}

/**
 * Sets the LED brightness depending on the current system run state
 * 
 * DISABLED - Off
 * OFF      - Fade In/Out
 * ACC      - Flashing
 * START    - Rapid Flashing
 * RUN      - On Solid
 * 
 * @return none
 */
void setLedBrigtness() {
  switch (runState) {
    case DISABLED: // Solid Off
      brightness = 0;
      break;
    case OFF: // Fade / Breath
      // Change the brightness for next time through the loop
      brightness = brightness + fadeAmount;

      // The following two checks will eliminate a quick flash if we
      // enter this loop from one of the other states with a number
      // above or below the max.
      if (brightness + fadeAmount > 255) {
        brightness = 255;
      }
      if (brightness + fadeAmount < 0) {
        brightness = 0;
      }
      
      // Reverse the direction of the fading at the ends of the fade
      if (brightness <= 0 || brightness >= 255) {
        fadeAmount = -fadeAmount;
      }
      break;
     case ACC: // Flashing
      // For every <FLASH_DELAY> ticks turn the LED on or off
      if (tick % FLASH_DELAY == 0) {
        if (brightness > 0) {
          brightness = 0;
        } else {
          brightness = 255;
        }
      }
      tick++;
      break;
     case START: // Rapid Flashing
      if (brightness > 0) {
        brightness = 0;
      } else {
        brightness = 255;
      }
      break;
     case RUN: // Solid On
      brightness = 255;
      break;
  }
}

/**
 * Prints the current system state
 * 
 * @param  state Current system state
 * @return none
 */
void printState(sysState state) {
  String strState;
  switch (state) {
     case DISABLED:  
        strState = "DISABLED";
        break;
     case OFF:
        strState = "OFF";
        break;
     case ACC:
        strState = "ACCESSORY";
        break;
     case START:
        strState = "STARTING";
        break;
     case RUN:
        strState = "RUNNING";
        break;
  }
  Serial.println("System State: " + strState);
}

