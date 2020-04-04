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
 * NFC
 * Can be used as a substitute for a wired KILL_SWITCH. The code below uses the PN532 library
 * from Elechouse (https://github.com/elechouse/PN532). To enable the starting system the system
 * will check the UID of the item close to the NFC antenna and make sure its in the approved 
 * list defined in the NFC_UIDS array. As long as the token is by the antenna the system will
 * stay active, if the token is removed from the antenna as soon as the system transitions to OFF
 * the token will be required to reenable the system. To get the UID of your tokens use the
 * PN532 readMifare example or the commented out debugging code that outputs the tokens UID.
 * 
 * 
 * @author Chris Fitzpatrick
 * @version 2.0 03/24/2020
 * @license MIT License
 */

// Defines
#define RELAY_ON        HIGH    // Relay Energized State (HIGH or LOW)
#define RELAY_OFF       LOW     // Relay Deenegized State (Opposite of RELAY_ON)

#define PUSH_BTN_PRESS  HIGH    // Push Button Pressed State (HIGH or LOW)

#define SAFE_SWITCH_ENB HIGH    // Safety Switch enabled

#define NFC_KILL_SWITCH 1       // 1=Enable NFC Kill Switch, 0=Enable Physical KILL_SWITCH

#if NFC_KILL_SWITCH
  // Use the PN532 Module in I2C mode, see PN532 examples for using in HSU or SPI modes
  #include <Wire.h>
  #include <PN532_I2C.h>
  #include <PN532.h>
  PN532_I2C pn532i2c(Wire);
  PN532 nfc(pn532i2c);

  byte NFC_UIDS[][7] = {
       {0x69, 0xAF, 0xEC, 0xC2, 0x00, 0x00}, // Fob
       {0x79, 0x5A, 0x13, 0xB1, 0x00, 0x00}  // Card
  };
#endif

// Pin Definitions

// LEDs
const int LED_BUTTON   = 9;  // Push button LED
const int LED_ON_BOARD = 13; // Onboard may not fade if there is no PWM attached,
                             // but can at least give indication of state when not  
                             // using 12v input for testing/debug
// Switches
const int PUSH_BUTTON = 6;   // Momentary push button
const int KILL_SWITCH = 3;   // This is any input to enable/disable the system, could be NFC 
                             // trigger/physical switch, if grounded system is disabled.
const int SAFE_SWITCH = 5;   // Brake/Clutch/Neutral safety switch, if grounded cannot start.
                             
// Relays
const int RELAY_ACC   = 18;
const int RELAY_IGN   = 21;
const int RELAY_START = 19;  
const int RELAY_4     = 8;   // Not Used

// Speaker
const int SPEAKER     = 22;

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
                                                          
const int HOLD_MS = 1000;       // Milliseconds to hold button to transition from ACC->START 
                                // or from RUN->ACC                             
                 
unsigned int tick = 0;          // Count cycles for LED flash mode (unsigned so when it rolls
                                // over it restarts at 0
int brightness = 0;             // How bright the LED is (usable range 0-255)
int fadeAmount = 5;             // How many points to fade the LED by when in OFF state  

int previousButtonState = !PUSH_BTN_PRESS;  // Stores the previous button state so we can see if it changes
long pressTime;                 // When the button was pressed 

sysState runState = DISABLED;   // Stores current system run state

/**
 * Setup runs once when system is reset, Setup pins and default states
 */
void setup() {
  // Initialize serial communication at 9600 bits per second
  Serial.begin(9600);
  // Leonardo/Micro delay, only uncomment for debugging will prevent running without Serial
  //while(!Serial) { delay(10); }

#if NFC_KILL_SWITCH
  // Setup NFC
  nfc.begin();
  unsigned long versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }
  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  
  // configure board to read RFID tags
  nfc.SAMConfig();
#endif

  // Setup Pins
  pinMode(LED_BUTTON, OUTPUT);
  pinMode(LED_ON_BOARD, OUTPUT);
  
  pinMode(PUSH_BUTTON, INPUT);
  pinMode(KILL_SWITCH, INPUT_PULLUP);
  pinMode(SAFE_SWITCH, INPUT_PULLUP); 
  
  pinMode(RELAY_ACC, OUTPUT);
  pinMode(RELAY_IGN, OUTPUT);
  pinMode(RELAY_START, OUTPUT);

  pinMode(SPEAKER, OUTPUT);
  
  // Disable relays at start
  // RELAY_OFF == Denergized, RELAY_ON = Energized
  digitalWrite(RELAY_ACC, RELAY_OFF);
  digitalWrite(RELAY_IGN, RELAY_OFF);
  digitalWrite(RELAY_START, RELAY_OFF);

  Serial.println("+------ Push Button Start System v2.0 ------+");
  Serial.println("| MIT License                               |");
  Serial.println("| Copyright (c) 2016-2020 Chris Fitzpatrick |");
  Serial.println("| https://github.com/mr-fitzie/push_start   |");
  Serial.println("+-------------------------------------------+");
  printState(runState);
  playDisabledChime();
}

/**
 * Main Run Loop
 */
void loop() {
  // Set the brightness of LEDs:
  analogWrite(LED_BUTTON, brightness);
  analogWrite(LED_ON_BOARD, brightness);
  
  // System is enabled, process as normal
  if (systemEnabled()) { 
    // If we were disabled move to OFF
    if (runState == DISABLED) {
      runState = OFF;
      printState(runState);
      playEnabledChime(); 
    }
    
    // Read in the button state to see if it has been pressed
    int buttonState = digitalRead(PUSH_BUTTON);
    if (buttonState != previousButtonState) {
      // Transition on released state for most states
      if (buttonState != PUSH_BTN_PRESS) { // Released
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
            // Transition back to ACC if button was held longer than HOLD_MS otherwise
            // go to OFF.
            runState = OFF;
            if (millis() - pressTime > HOLD_MS) {
              // In order for the engine to turn off to we need to transition to OFF, 
              //wait and then to ACC  
              setRelayState();
              delay(500);
              runState = ACC;
            }
            break;
        }
        printState(runState);
        setRelayState();
      } else { // Pressed
        pressTime = millis();
        Serial.println("Pressed");
      } 
    } else if (buttonState == PUSH_BTN_PRESS) { // Pressed
      // Special case for START
      // When we release the push button in the ACC state we want to transition to
      // START if held longer than HOLD_MS and the SAFE_SWITCH is open
      if (runState == ACC) {
        bool safe = digitalRead(SAFE_SWITCH) != SAFE_SWITCH_ENB; // ENB = Unsafe to start
        if (safe) {
          if(millis() - pressTime > HOLD_MS) {
            runState = START;
            printState(runState);
            setRelayState();
          }
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
      playDisabledChime();
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
      digitalWrite(RELAY_ACC, RELAY_OFF);
      digitalWrite(RELAY_IGN, RELAY_OFF);
      digitalWrite(RELAY_START, RELAY_OFF);
      break;
    case ACC:
      digitalWrite(RELAY_ACC, RELAY_ON);
      digitalWrite(RELAY_IGN, RELAY_OFF);
      digitalWrite(RELAY_START, RELAY_OFF);
      break;
    case START:
      digitalWrite(RELAY_ACC, RELAY_OFF);
      digitalWrite(RELAY_IGN, RELAY_ON);
      digitalWrite(RELAY_START, RELAY_ON);
      break;
    case RUN:
      digitalWrite(RELAY_ACC, RELAY_OFF);
      digitalWrite(RELAY_IGN, RELAY_ON);
      digitalWrite(RELAY_START, RELAY_OFF);
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
      else if (brightness + fadeAmount < 0) {
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

/**
 * Play Disabled Chime
 * 
 * @return none
 */
void playDisabledChime() {
  tone(SPEAKER, 2000, 250);
  delay(250);
  tone(SPEAKER, 4000, 250);
}

/**
 * Play Enabled Chime
 * 
 * @return none
 */
void playEnabledChime() {
  tone(SPEAKER, 4000, 250);
  delay(250);
  tone(SPEAKER, 2000, 250);  
}

/**
 * Checks to see if the system is enabled which will allow the vehicle 
 * to be started
 * 
 * @return bool true if system is enabled, false if disabled
 */
bool systemEnabled() {
#if NFC_KILL_SWITCH
  bool enabled = false;  
  bool success = false;
  byte uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  byte uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

  // If the system has already been enabled from NFC and started keep it enabled
  // until the car is turned OFF
  if (runState != DISABLED && runState != OFF) {
    return true;
  }

  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 25);  
  //Serial.print("UID Value: ");
  //nfc.PrintHex(uid, uidLength);
  //Serial.println("");
  bool found = false;
  for (byte i = 0; i < (sizeof(NFC_UIDS) / sizeof(NFC_UIDS[0])); i++) {
    //Serial.print("Compare to known UID: ");
    //nfc.PrintHex(NFC_UIDS[i], sizeof(NFC_UIDS[i]));
    if (checkUid(NFC_UIDS[i], uid, sizeof(NFC_UIDS[i]))) {
      //Serial.println("UID Match");
      found = true;
      break;
    }
  }
  return found;
#else
  return digitalRead(KILL_SWITCH) == HIGH; // HIGH = System Enabled LOW = Disabled (grounded)
#endif
}

/**
 * Compares known UID to UID to see if they match
 * 
 * @return bool true if UID is a match, false if not
 */
bool checkUid(byte knownUid[], byte uid[], byte uidLength) {
  for (byte i = 0; i < uidLength; i++) {
    if (knownUid[i] != uid[i]) {
      return false;
    }
  }
  return true;
}
