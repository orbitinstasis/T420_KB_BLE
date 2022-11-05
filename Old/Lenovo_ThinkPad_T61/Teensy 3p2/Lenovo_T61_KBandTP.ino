/* Copyright 2018 Frank Adams
   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at
       http://www.apache.org/licenses/LICENSE-2.0
   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
// This software controls a Lenovo ThinkPad T61 Laptop Keyboard and PS/2 Trackpoint using a Teensy 3.2 on
// a daughterboard with a 44 pin FPC connector. The keyboard part number is 42T3177.
// This routine uses the Teensyduino "Micro-Manager Method" to send Normal and Modifier
// keys over USB. Only the volume control multi-media keys are supported by this routine.
// Description of Teensyduino keyboard functions is at www.pjrc.com/teensy/td_keyboard.html
// The ps/2 code uses the USB PJRC Mouse functions at www.pjrc.com/teensy/td_mouse.html
// The ps/2 code has a watchdog timer so the code can't hang if a clock edge is missed.
// In the Arduino IDE, select Tools, Teensy 3.2. Also under Tools, select Keyboard+Mouse+Joystick
//
// Revision History
// Rev 1.0 - Nov 23, 2018 - Original Release
// Rev 1.1 - Dec 2, 2018 - Replaced ps/2 trackpoint code from playground arduino with my own code
// Rev 1.2 - July 16, 2019 - Check if slots are full when detecting a key press
// Rev 1.3 - 18 August, 2021 - using interrupt driven trackpoint code Ben Kazemi
// Rev 1.4 - 18 August, 2021 - added sleep,prev,next,play,stop,numlock to media matrix. Moved \ and gui
// Rev 1.5 - 18 August, 2021 - added conditioning to TP reset
// Rev 1.6 - 19 August, 2021 - removed -Fn, SYNC, Num lock, Scroll lock IOs, moved Caps lock to pin 13
//                             added Fn functionality 

/**
  pin 23 was -Fn
  pin 27 was SYNC
  pin 28 was caps lock led
  pin 29 was num lock led
  pin 30 was scroll lock
*/
#include "trackpoint.h"

//#define MODIFIERKEY_FN 0x8f   // give Fn key a HID code
// Trackpoint signals
#define TP_DATA 18   // ps/2 data to trackpoint
#define TP_CLK 19    // ps/2 clock to trackpoint
#define TP_RESET 0   // active high trackpoint reset at power up
// Keyboard LEDs  jlj
#define CAPS_LED 13  // The LED on the Teensy is programmed to blink 
// Keyboard Fn key (aka HOTKEY)
#define HOTKEY 14       // Fn key plus side


// Set the keyboard row & column size
const byte rows_max = 16; // sets the number of rows in the matrix
const byte cols_max = 8; // sets the number of columns in the matrix

static const char buttonStates[] = { MOUSE_LEFT, MOUSE_RIGHT, MOUSE_MIDDLE };

TrackPoint trackpoint(TP_CLK, TP_DATA, TP_RESET);

//
// Load the normal key matrix with the Teensyduino key names described at www.pjrc.com/teensy/td_keyboard.html
// A zero indicates no normal key at that location.
//
int normal[rows_max][cols_max] = {
  {KEY_TILDE, KEY_1, KEY_Q, KEY_TAB, KEY_A, KEY_ESC, KEY_Z, 0},
  {KEY_F1, KEY_2, KEY_W, KEY_CAPS_LOCK, KEY_S, KEY_BACKSLASH, KEY_X, 0}, //moved backslash here
  {KEY_F2, KEY_3, KEY_E, KEY_F3, KEY_D, KEY_F4, KEY_C, 0},
  {KEY_5, KEY_4, KEY_R, KEY_T, KEY_F, KEY_G, KEY_V, KEY_B},
  {KEY_6, KEY_7, KEY_U, KEY_Y, KEY_J, KEY_H, KEY_M, KEY_N},
  {KEY_EQUAL, KEY_8, KEY_I, KEY_RIGHT_BRACE, KEY_K, KEY_F6, KEY_COMMA, 0},
  {KEY_F8, KEY_9, KEY_O, KEY_F7, KEY_L, 0, KEY_PERIOD, 0},
  {KEY_MINUS, KEY_0, KEY_P, KEY_LEFT_BRACE, KEY_SEMICOLON, KEY_QUOTE, 0, KEY_SLASH},
  {KEY_F9, KEY_F10, 0, KEY_BACKSPACE, 0, KEY_F5, KEY_ENTER, KEY_SPACE},
  {KEY_INSERT, KEY_F12, 0, 0, 0, 0, 0, KEY_RIGHT},
  {KEY_DELETE, KEY_F11, 0, 0, 0, 0, 0, KEY_DOWN},
  {KEY_PAGE_UP, KEY_PAGE_DOWN, 0, 0, KEY_MENU, 0, 0, 0},
  {KEY_HOME, KEY_END, 0, 0, 0, KEY_UP, KEY_PAUSE, KEY_LEFT},
  {0, KEY_PRINTSCREEN, KEY_SCROLL_LOCK, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0}
};
// Load the modifier key matrix with key names at the correct row-column location.
// A zero indicates no modifier key at that location.
int modifier[rows_max][cols_max] = {
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, MODIFIERKEY_GUI, 0, 0, 0, 0}, //moved GUI here
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, MODIFIERKEY_LEFT_ALT, 0, MODIFIERKEY_RIGHT_ALT},
  {0, 0, 0, MODIFIERKEY_LEFT_SHIFT, 0, 0, MODIFIERKEY_RIGHT_SHIFT, 0},
  {MODIFIERKEY_LEFT_CTRL, 0, 0, 0, 0, 0, MODIFIERKEY_RIGHT_CTRL, 0}
};
// Load the media key matrix with key names at the correct row-column location.
// A zero indicates no media key at that location.
int media[rows_max][cols_max] = {
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, KEY_SYSTEM_SLEEP, 0, 0}, //added sleep at Fn-F4 position
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, KEY_MEDIA_NEXT_TRACK}, //added next-track at Fn-Right position
  {0, 0, KEY_MEDIA_VOLUME_INC, KEY_MEDIA_VOLUME_DEC, KEY_MEDIA_MUTE, 0, 0, KEY_MEDIA_PLAY_PAUSE}, //added play-pause at Fn-Down
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, KEY_MEDIA_STOP, 0, KEY_MEDIA_PREV_TRACK}, //added prev-track at Fn-Left position. Stop at Fn-Up position.
  {0, 0, KEY_NUM_LOCK, 0, 0, 0, 0, 0}, //added num-lock at Fn-Scroll lock position
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0}
};
// Initialize the old_key matrix with one's.
// 1 = key not pressed, 0 = key is pressed
boolean old_key[rows_max][cols_max] = {
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 1}
};
//
// Define the Teensy 3.2 I/O numbers
//
// Row FPC pin # 22,18,14,10,02,04,08,12,06,20,16,24,28,32,26,30
// Teensy I/O  # 20,33,24,25,31,32,07,06,26,04,05,03,02,01,21,22
int Row_IO[rows_max] = {20, 33, 24, 25, 31, 32, 7, 6, 26, 4, 5, 3, 2, 1, 21, 22}; // Teensy 3.2 I/O numbers for rows
//
// Column FPC pin # 05,13,09,07,11,03,15,17
// Teensy I/O     # 16,10,12,17,11,15,09,08
int Col_IO[cols_max] = {16, 10, 12, 17, 11, 15, 9, 8}; // Teensy 3.2 I/O numbers for columns
//
// Declare variables that will be used by functions

boolean slots_full = LOW; // Goes high when slots 1 thru 6 contain normal keys
// slot 1 thru slot 6 hold the normal key values to be sent over USB.
int slot1 = 0; //value of 0 means the slot is empty and can be used.
int slot2 = 0;
int slot3 = 0;
int slot4 = 0;
int slot5 = 0;
int slot6 = 0;
//
int mod_shift_l = 0; // These variables are sent over USB as modifier keys.
int mod_shift_r = 0; // Each is either set to 0 or MODIFIER_ ...
int mod_ctrl_l = 0;
int mod_ctrl_r = 0;
int mod_alt_l = 0;
int mod_alt_r = 0;
int mod_gui = 0;

//
// **************Functions common to keyboard and trackpoint**************************
//
// Function to set a pin to high impedance (acts like open drain output)
void go_z(int pin)
{
  pinMode(pin, INPUT);
  digitalWrite(pin, HIGH);
}
//
// Function to set a pin as an input with a pullup
void go_pu(int pin)
{
  pinMode(pin, INPUT_PULLUP);
  digitalWrite(pin, HIGH);
}
//
// Function to send a pin to a logic low
void go_0(int pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}
//
// Function to send a pin to a logic high
void go_1(int pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}
//
// *****************Functions for Trackpoint*****************************
void clockInterrupt()
{
  trackpoint.readData();
}
//
// *****************Functions for Keyboard*****************************
// Function to load the key name into the first available slot
void load_slot(int key) {
  if (!slot1)  {
    slot1 = key;
  }
  else if (!slot2) {
    slot2 = key;
  }
  else if (!slot3) {
    slot3 = key;
  }
  else if (!slot4) {
    slot4 = key;
  }
  else if (!slot5) {
    slot5 = key;
  }
  else if (!slot6) {
    slot6 = key;
  }
  if (!slot1 || !slot2 || !slot3 || !slot4 || !slot5 || !slot6)  {
    slots_full = LOW; // slots are not full
  }
  else {
    slots_full = HIGH; // slots are full
  }
}
//
// Function to clear the slot that contains the key name
void clear_slot(int key) {
  if (slot1 == key) {
    slot1 = 0;
  }
  else if (slot2 == key) {
    slot2 = 0;
  }
  else if (slot3 == key) {
    slot3 = 0;
  }
  else if (slot4 == key) {
    slot4 = 0;
  }
  else if (slot5 == key) {
    slot5 = 0;
  }
  else if (slot6 == key) {
    slot6 = 0;
  }
  if (!slot1 || !slot2 || !slot3 || !slot4 || !slot5 || !slot6)  {
    slots_full = LOW; // slots are not full
  }
  else {
    slots_full = HIGH; // slots are full
  }
}
//
// Function to load the modifier key name into the appropriate mod variable
void load_mod(int m_key) {
  if (m_key == MODIFIERKEY_LEFT_SHIFT)  {
    mod_shift_l = m_key;
  }
  else if (m_key == MODIFIERKEY_RIGHT_SHIFT)  {
    mod_shift_r = m_key;
  }
  else if (m_key == MODIFIERKEY_LEFT_CTRL)  {
    mod_ctrl_l = m_key;
  }
  else if (m_key == MODIFIERKEY_RIGHT_CTRL)  {
    mod_ctrl_r = m_key;
  }
  else if (m_key == MODIFIERKEY_LEFT_ALT)  {
    mod_alt_l = m_key;
  }
  else if (m_key == MODIFIERKEY_RIGHT_ALT)  {
    mod_alt_r = m_key;
  }
  else if (m_key == MODIFIERKEY_GUI)  {
    mod_gui = m_key;
  }
}
//
// Function to load 0 into the appropriate mod variable
void clear_mod(int m_key) {
  if (m_key == MODIFIERKEY_LEFT_SHIFT)  {
    mod_shift_l = 0;
  }
  else if (m_key == MODIFIERKEY_RIGHT_SHIFT)  {
    mod_shift_r = 0;
  }
  else if (m_key == MODIFIERKEY_LEFT_CTRL)  {
    mod_ctrl_l = 0;
  }
  else if (m_key == MODIFIERKEY_RIGHT_CTRL)  {
    mod_ctrl_r = 0;
  }
  else if (m_key == MODIFIERKEY_LEFT_ALT)  {
    mod_alt_l = 0;
  }
  else if (m_key == MODIFIERKEY_RIGHT_ALT)  {
    mod_alt_r = 0;
  }
  else if (m_key == MODIFIERKEY_GUI)  {
    mod_gui = 0;
  }
}
//
// Function to send the modifier keys over usb
void send_mod() {
  Keyboard.set_modifier(mod_shift_l | mod_shift_r | mod_ctrl_l | mod_ctrl_r | mod_alt_l | mod_alt_r | mod_gui);
  Keyboard.send_now();
}
//
// Function to send the normal keys in the 6 slots over usb
void send_normals() {
  Keyboard.set_key1(slot1);
  Keyboard.set_key2(slot2);
  Keyboard.set_key3(slot3);
  Keyboard.set_key4(slot4);
  Keyboard.set_key5(slot5);
  Keyboard.set_key6(slot6);
  Keyboard.send_now();
}
//
//************************************Setup*******************************************
void setup() {

  // ************trackpoint setup
  trackpoint.reset();
  trackpoint.setSensitivityFactor(170);
  trackpoint.enable();

  attachInterrupt(digitalPinToInterrupt(TP_CLK), clockInterrupt, FALLING);
  // ************keyboard setup
  for (int a = 0; a < cols_max; a++) {  // loop thru all column pins
    go_pu(Col_IO[a]); // set each column pin as an input with a pullup
  }
  //
  for (int b = 0; b < rows_max; b++) {  // loop thru all row pins
    go_z(Row_IO[b]); // set each row pin as a floating output
  }

  go_pu(HOTKEY);    // Pull up the Hotkey plus side for reading
}
//
// *******declare and initialize trackpoint variables

// **********declare and initialize keyboard variables
boolean Fn_pressed = HIGH; // Initialize Fn key to HIGH = "not pressed"
extern volatile uint8_t keyboard_leds; // 8 bits sent from Host to Teensy that give keyboard LED status.
char blink_count = 0; // Blink loop counter
boolean blinky = LOW; // Blink LED state
boolean sync_sig = LOW; // sync pulse to measure scan frequency
//
//*********************************Main Loop*******************************************
//
void loop() {
  // *************Keyboard Main**************
  //  // Read the Fn key (aka Hotkey) which is not part of the key matrix
  if (!digitalRead(HOTKEY)) {
    Fn_pressed = LOW; // Fn key is pressed (active low)
  }
  else  {
    Fn_pressed = HIGH; // Fn key is not pressed
  }
  //
  // Scan keyboard matrix with an outer loop that drives each row low and an inner loop that reads every column (with pull ups).
  // The routine looks at each key's present state (by reading the column input pin) and also the previous state from the last scan
  // that was 30msec ago. The status of a key that was just pressed or just released is sent over USB and the state is saved in the old_key matrix.
  // The keyboard keys will read as logic low if they are pressed (negative logic).
  // The old_key matrix also uses negative logic (low=pressed).
  //
  for (int x = 0; x < rows_max; x++) {   // loop thru the rows
    go_0(Row_IO[x]); // Activate Row (send it low)
    delayMicroseconds(10); // give the row time to go low and settle out
    for (int y = 0; y < cols_max; y++) {   // loop thru the columns
      // **********Modifier keys including the Fn special case
      if (modifier[x][y] != 0) {  // check if modifier key exists at this location in the array (a non-zero value)
        if (!digitalRead(Col_IO[y]) && (old_key[x][y])) {  // Read column to see if key is low (pressed) and was previously not pressed
          if (modifier[x][y] != Fn_pressed) {   // Exclude Fn modifier key
            load_mod(modifier[x][y]); // function reads which modifier key is pressed and loads it into the appropriate mod_... variable
            send_mod(); // function sends the state of all modifier keys over usb including the one that just got pressed
            old_key[x][y] = LOW; // Save state of key as "pressed"
          }
          else {
            Fn_pressed = LOW; // Fn status variable is active low
            old_key[x][y] = LOW; // old_key state is "pressed" (active low)
          }
        }
        else if (digitalRead(Col_IO[y]) && (!old_key[x][y])) {  //check if key is not pressed and was previously pressed
          if (modifier[x][y] != Fn_pressed) { // Exclude Fn modifier key
            clear_mod(modifier[x][y]); // function reads which modifier key was released and loads 0 into the appropriate mod_... variable
            send_mod(); // function sends all mod's over usb including the one that just released
            old_key[x][y] = HIGH; // Save state of key as "not pressed"
          }
          else {
            Fn_pressed = HIGH; // Fn is no longer active
            old_key[x][y] = HIGH; // old_key state is "not pressed"
          }
        }
      }
      // ***********end of modifier section

      // ***********Normal keys section and media keys in this section
      else if ((normal[x][y] != 0) || (media[x][y] != 0)) {  // check if normal or media key exists at this location in the array
        if (!digitalRead(Col_IO[y]) && (old_key[x][y]) && (!slots_full)) { // check if key pressed and not previously pressed and slots not full
          old_key[x][y] = LOW; // Save state of key as "pressed"
          if (Fn_pressed) {  // Fn_pressed is active low so it is not pressed and normal key needs to be sent
            load_slot(normal[x][y]); //update first available slot with normal key name
            send_normals(); // send all slots over USB including the key that just got pressed
          }
          else if (media[x][y] != 0) { // Fn is pressed so send media if a key exists in the matrix
            Keyboard.press(media[x][y]); // media key is sent using keyboard press function per PJRC
            delay(5); // delay 5 milliseconds before releasing to make sure it gets sent over USB
            Keyboard.release(media[x][y]); // send media key release
          }
        }
        else if (digitalRead(Col_IO[y]) && (!old_key[x][y])) { //check if key is not pressed, but was previously pressed
          old_key[x][y] = HIGH; // Save state of key as "not pressed"
          if (Fn_pressed) {  // Fn is not pressed
            clear_slot(normal[x][y]); //clear the slot that contains the normal key name
            send_normals(); // send all slots over USB including the key that was just released 
          }
        }
      }
      // **************end of normal section
      //
      // *************Volume key section. Note PJRC states that volume up, down, & mute should be sent with Keyboard.press function.
      else if (media[x][y] != 0) {  // check if any volume control key exists at this location in the array (a non-zero value)
        if (!digitalRead(Col_IO[y]) && (old_key[x][y])) { // check if key is pressed and was not previously pressed
          old_key[x][y] = LOW; // Save state of key as "pressed"
          Keyboard.press(media[x][y]); // send volume key press
        }
        else if (digitalRead(Col_IO[y]) && (!old_key[x][y])) { //check if key is not pressed, but was previously pressed
          old_key[x][y] = HIGH; // Save state of key as "not pressed"
          Keyboard.release(media[x][y]); // send volume key release
        }
      }
      // ***************end of volume section
    }
    go_z(Row_IO[x]); // De-activate Row (send it to hi-z)
  }
  //
  // **********keyboard scan complete
  //
  // ****************************Trackpoint Routine*********************************
  //
  if (trackpoint.reportReady())
  {
    const DataReport& report = trackpoint.getDataReport();

    for (uint8_t i = 0; i < sizeof(buttonStates); ++i)
    {
      if (report.state & (1 << i))
      {
        Mouse.press(buttonStates[i]);
      }
      else if (Mouse.isPressed(buttonStates[i]))
      {
        Mouse.release(buttonStates[i]);
      }
    }
    Mouse.move(report.x, -report.y);
  }
  // **************************************End of trackpoint routine***********************************
  //
  // *******keyboard LEDs
  // Turn on or off the LEDs for Num Lock, Caps Lock, and Scroll Lock based on bit 0, 1, and 2 from the keyboard_leds
  // variable controlled by the USB host computer
  //

  if (keyboard_leds & 1 << 1) { // mask off all bits but D1 and test if set
    go_0(CAPS_LED); // turn on the Caps Lock LED
  }
  else {
    go_1(CAPS_LED); // turn off the Caps Lock LED
  }
  // ****************End of main loop
}
