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
 * If the kill switch circuit is open then the whole system is disabled and the relays are
 * immediately deenergized and the button no longer can transition states.
 * 
 * When the kill switch is closed the system is enabeled and has the basic flow of:
 * ACC -> (START) -> RUN -> OFF
 * * To transition from OFF -> ACC depress and release the push button
 * * To transition from ACC -> (START) -> RUN depress and hold the push button in until 
 * * vehicle is started and then release the button. The initial press puts it in START  
 * * mode and the release of the button once the vehicle is started transitions to RUN mode. 
 * * To transition from RUN -> OFF depress and release the push button.
 * 
 * Because the timing to kick over the motor can be different for various vehicles and you
 * do not want not run the starter too long which may grind the flywheel/flexplate teeth the
 * I decided to keep the amount of time the starter is engaged to the actual press, just like
 * turning a key. The system could be modified to do it based on time in the future.
 * 
 * Future enhancements:
 * * Secondary Killswitch (prox sensor? nfc/rfid?)
 * * Brake Input?
 * * OFF->RUN?
 * * RUN->ACC?
 * 
 * @author Chris Fitzpatrick
 * @version 0.6 01/01/2017
 * @license MIT License
 */

// Pin Definitions

// LEDs
const int LED_BUTTON = 9;    // Push button LED
const int LED_ON_BOARD = 13; // Onboard will not fade because there is no PWM attached,
                             // but can at least give indication of state when not  
                             // using 12v input for testing/debug
// Switches
const int PUSH_BUTTON = 2;   // Momentary push button
const int KILL_SWITCH = 3;   // This is any input to enable/disable the system, could be NFC 
                             // trigger/physical swtich
// Relays
const int RELAY_ACC = 5;
const int RELAY_IGN = 6;
const int RELAY_START = 7;
const int RELAY_4 = 8;        // Not Used

// System state
enum sysState {
  DISABLED,
  OFF,
  ACC,
  START,
  RUN
};

// Globals
const int FLASH_DELAY = 10;     // How many ticks to pass through before turning on/off LED
                                // when in ACC state. Increasing this number makes the blink
                                // slower. Since the main loop sleeps 30ms a value of 10 here
                                // waits 300ms in ON then 300ms in OFF.
                 
unsigned int tick = 0;          // Count cycles for LED flash mode (unsigned so when it rolls
                                // over it restarts at 0
int brightness = 0;             // How bright the LED is (usable range 0-255)
int fadeAmount = 5;             // How many points to fade the LED by when in OFF state      

int previousButtonState = HIGH; // Stores the previous button state so we can see if it changes
sysState runState = OFF;        // Stores current system run state

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
  
  pinMode(RELAY_ACC, OUTPUT);
  pinMode(RELAY_IGN, OUTPUT);
  pinMode(RELAY_START, OUTPUT);

  // Disable relays at start
  // HIGH == Denergized/OFF, LOW = Energized/ON
  digitalWrite(RELAY_ACC, HIGH);
  digitalWrite(RELAY_IGN, HIGH);
  digitalWrite(RELAY_START, HIGH);
}

/**
 * Main Run Loop
 */
void loop() {
  // Set the brightness of LEDs:
  analogWrite(LED_BUTTON, brightness);
  analogWrite(LED_ON_BOARD, brightness);

  // System is enabled, process as normal
  if (digitalRead(KILL_SWITCH) == HIGH) { // HIGH = System Enabled, LOW = Disabled
    // If we were disabled move to OFF
    if (runState == DISABLED) {
      runState = OFF;
    }
    // Read in the button state to see if it has been pressed
    int buttonState = digitalRead(PUSH_BUTTON);
    if (buttonState != previousButtonState) {
      // Only transition on HIGH state so we do not bounce
      if (buttonState == HIGH) {
        switch (runState){
          case DISABLED: // Disabled -> Off
            // This state should never be hit, but is included for completeness.
            runState = OFF;
            break;
          case OFF: // Off -> Acc
            runState = ACC;
            digitalWrite(RELAY_ACC, LOW);
            digitalWrite(RELAY_IGN, HIGH);
            digitalWrite(RELAY_START, HIGH);
            break;
          case ACC: // Acc -> Start
            runState = START;
            digitalWrite(RELAY_ACC, HIGH);
            digitalWrite(RELAY_IGN, LOW);
            digitalWrite(RELAY_START, LOW);
            break;
          case START: // Start -> Run
            // This state should never really be hit since we bump from START->RUN when
            // we release the push button switch during the START sequence
            runState = RUN;
            digitalWrite(RELAY_ACC, HIGH);
            digitalWrite(RELAY_IGN, LOW);
            digitalWrite(RELAY_START, HIGH);
            break;
          case RUN: // Run -> Off
            runState = OFF;
            digitalWrite(RELAY_ACC, HIGH);
            digitalWrite(RELAY_IGN, HIGH);
            digitalWrite(RELAY_START, HIGH);
            break;
        }
        
        Serial.println("System State:");
        Serial.println(runState);
      } else {
        // Special case for START
        // When we release the push button in the START state we want to immediately
        // transition to RUN
        if (runState == START) {
          // Turn off starter solenoid and move to RUN
          digitalWrite(RELAY_START, HIGH);
          runState = RUN;
        }
      }
    }
    // Store button state for next cycle
    previousButtonState = buttonState;
  } else {
    runState = DISABLED;
    digitalWrite(RELAY_ACC, HIGH);
    digitalWrite(RELAY_IGN, HIGH);
    digitalWrite(RELAY_START, HIGH);
  }
  // Set brightness for next cycle
  setLedBrigtness();
  // Wait for 30 milliseconds
  delay(30);
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
