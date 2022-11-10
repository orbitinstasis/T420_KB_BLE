/* Copyright 2018 Frank Adams
   Copyright 2022
   Ben Kazemi
   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at
       http://www.apache.org/licenses/LICENSE-2.0
   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

  This software controls a Lenovo ThinkPad T61 Laptop Keyboard and PS/2 Trackpoint using a Teensy 3.2 on
  a daughterboard with a 44 pin FPC connector. The keyboard part number is 42T3177.
  This routine uses the Teensyduino "Micro-Manager Method" to send Normal and Modifier
  keys over USB. Only the volume control multi-media keys are supported by this routine.
  Description of Teensyduino keyboard functions is at www.pjrc.com/teensy/td_keyboard.html
  The ps/2 code uses the USB PJRC Mouse functions at www.pjrc.com/teensy/td_mouse.html
  The ps/2 code has a watchdog timer so the code can't hang if a clock edge is missed.
  In the Arduino IDE, select Tools, Teensy 3.2. Also under Tools, select Keyboard+Mouse+Joystick
  Revision History
  Rev 1.0 - Nov 23, 2018 - Original Release
  Rev 1.1 - Dec 2, 2018 - Replaced ps/2 trackpoint code from playground arduino with my own code
  Rev 1.2 - July 16, 2019 - Check if slots are full when detecting a key press
  Rev 1.3 - 18 August, 2021 - using interrupt driven trackpoint code Ben Kazemi
  Rev 1.4 - 18 August, 2021 - added sleep,prev,next,play,stop,numlock to media matrix. Moved \ and gui
  Rev 1.5 - 18 August, 2021 - added conditioning to TP reset
  Rev 1.6 - 19 August, 2021 - removed -Fn, SYNC, Num lock, Scroll lock IOs, moved Caps lock to pin 13
                              added Fn functionality
  Rev 2.0 - 21 August, 2021 - added bluetooth support over UART (need HW factory RST on repeat flashes)
                              refactored code by moving all definitions, macros and functions to helper.ino
                              added ATcommand mode from USB serial connection with the define set to true
                              finalised ugly BLE code with removed scroll lock, media keys and modifiers
                              fixed UK localisation on usb mode and ble mode
                              flipped fn and left ctrl
  Rev 2.1 - 05 November, 2022 -
                                 Added code to exclusivly send BLE or USB codes based solely on the states of physical USB port.

                                Enclosure has been designed and tested, some minor changes to be made but otherwise working well.

                                PCB files were incorrect previously, have uploaded correct files but need to add minor pin changes.

                                Attached Images

                                Whole keyboard is balaced in the centre and weighs over a kg.

                                BLE mouse move needs to be smoothed

                                need to add a way to unpair whatever is paired on the keyboard side, currently only possible by doing a factory reset or unpairing from the receiver

                                Power LED indicates keyboard power, currently the charging led is inside the keyboard and visible at an angle

  NOTE: You need to change the baud rate in Adafruit_BluefruitLE_UART::begin to the same baud rate here

  NOTE: Edited USB_DEX.h product name, man name len, man name

  TODO: ADD SOME DELAY WHEN NOT KEYING if i can reduce power consumtption during this delay
  find battery suitable


  pin 23 was -Fn
  pin 26 moved to 6
  pin 27 was SYNC is now FPC 2
  pin 28 was caps lock led
  pin 29 was num lock led is now existing 26 (FPC 6)
  pin 30 was scroll lock, is now what used to be 31 (FPC 2)



*/
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLE.h"
#include "trackpoint.h"


boolean BLE_EN  = 1;
boolean USB_EN  = 0;
//BLE defines
#define AT_COMMAND_MODE                 false
#define FACTORYRESET_ENABLE             0
#define MINIMUM_FIRMWARE_VERSION        "0.6.6"
#define BLUEFRUIT_HWSERIAL_NAME         Serial2
#define BUFSIZE                         256   // Size of the read buffer for incoming data
#define VERBOSE_MODE                    true  // If set to 'true' enables debug output
// Trackpoint signals
#define TP_DATA           18   // ps/2 data to trackpoint
#define TP_CLK            19    // ps/2 clock to trackpoint
#define TP_RESET          0   // active high trackpoint reset at power up
//#define CAPS_LED          13  // The LED on the Teensy is programmed to blink
#define HOTKEY            14       // Fn key plus side
#define TX                31
#define RX                26
#define BAUD              250000
#define MOUSE_MULTIPLIER  1

Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, -1, 23);
// Set the keyboard row & column size
const byte rows_max = 16; // sets the number of rows in the matrix
const byte cols_max = 8; // sets the number of columns in the matrix

// trackpoint variables
static const char buttonStates[] = { MOUSE_LEFT, MOUSE_RIGHT, MOUSE_MIDDLE };
bool USBbuttonClicked[] = { false, false, false };
static const char buttonStatesBLE[] = { 'L', 'R', 'M' };
TrackPoint trackpoint(TP_CLK, TP_DATA, TP_RESET);

// **********declare and initialize keyboard variables
boolean Fn_pressed = HIGH; // Initialize Fn key to HIGH = "not pressed"
extern volatile uint8_t keyboard_leds; // 8 bits sent from Host to Teensy that give keyboard LED status.
char blink_count = 0; // Blink loop counter
boolean blinky = LOW; // Blink LED state
boolean sync_sig = LOW; // sync pulse to measure scan frequency
// Load the normal key matrix with the Teensyduino key names described at www.pjrc.com/teensy/td_keyboard.html
// A zero indicates no normal key at that location.
int normal[rows_max][cols_max] = {
  {KEY_TILDE, KEY_1, KEY_Q, KEY_TAB, KEY_A, KEY_ESC, KEY_Z, 0},
  {KEY_F1, KEY_2, KEY_W, KEY_CAPS_LOCK, KEY_S, 0x64, KEY_X, 0}, //key for backslash, existing didn't work
  {KEY_F2, KEY_3, KEY_E, KEY_F3, KEY_D, KEY_F4, KEY_C, 0},
  {KEY_5, KEY_4, KEY_R, KEY_T, KEY_F, KEY_G, KEY_V, KEY_B},
  {KEY_6, KEY_7, KEY_U, KEY_Y, KEY_J, KEY_H, KEY_M, KEY_N},
  {KEY_EQUAL, KEY_8, KEY_I, KEY_RIGHT_BRACE, KEY_K, KEY_F6, KEY_COMMA, 0},
  {KEY_F8, KEY_9, KEY_O, KEY_F7, KEY_L, 0, KEY_PERIOD, 0},
  {KEY_MINUS, KEY_0, KEY_P, KEY_LEFT_BRACE, KEY_SEMICOLON, KEY_QUOTE, 0x32, KEY_SLASH}, // KEY_HASHTILDE for uk layout
  {KEY_F9, KEY_F10, 0, KEY_BACKSPACE, 0, KEY_F5, KEY_ENTER, KEY_SPACE},
  {KEY_INSERT, KEY_F12, 0, 0, 0, 0, 0, KEY_RIGHT},
  {KEY_DELETE, KEY_F11, 0, 0, 0, 0, 0, KEY_DOWN},
  {KEY_PAGE_UP, KEY_PAGE_DOWN, 0, 0, KEY_MENU, 0, 0, 0}, // removed KEY_MENU to make it "copy" in consumer  last two keys on this row are left and right page
  {KEY_HOME, KEY_END, 0, 0, 0, KEY_UP, KEY_PAUSE, KEY_LEFT},
  {0, KEY_PRINTSCREEN, KEY_NUM_LOCK, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0}
};
String normalBLE[rows_max][cols_max] = {
  {"35", "1E", "14", "2B", "04", "29", "1D", "00"},
  {"3A", "1F", "1A", "39", "16", "64", "1B", "00"}, //moved backslash here
  {"3B", "20", "08", "3C", "07", "3D", "06", "00"},
  {"22", "21", "15", "17", "09", "0A", "19", "05"},
  {"23", "24", "18", "1C", "0D", "0B", "10", "11"},
  {"2E", "25", "0C", "30", "0E", "3F", "36", "00"},
  {"41", "26", "12", "40", "0F", "00", "37", "00"},
  {"2D", "27", "13", "2F", "33", "34", "32", "38"},
  {"42", "43", "00", "2A", "00", "3E", "28", "2C"},
  {"49", "45", "00", "00", "00", "00", "00", "4F"},
  {"4C", "44", "80", "81", "7F", "00", "00", "51"},  //added media without fn
  {"4B", "4E", "00", "00", "65", "00", "00", "00"},
  {"4A", "4D", "00", "00", "00", "52", "48", "50"},
  {"00", "46", "53", "00", "00", "00", "00", "00"},
  {"00", "00", "00", "00", "00", "00", "00", "00"},
  {"00", "00", "00", "00", "00", "00", "00", "00"}
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
int modifierBLE[rows_max][cols_max] = {
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0x08, 0, 0, 0, 0}, //moved GUI here
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0x04, 0, 0x40},
  {0, 0, 0, 0x02, 0, 0, 0x20, 0},
  {0x01, 0, 0, 0, 0, 0, 0x10, 0}
};
// Load the media key matrix with key names at the correct row-column location.
// A zero indicates no media key at that location.
int media[rows_max][cols_max] = {
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0}, //added sleep at Fn-F4 position
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
  {0, 0, KEY_SCROLL_LOCK, 0, 0, 0, 0, 0}, //added num-lock at Fn-Scroll lock position
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0}
};

String mediaBLE[rows_max][cols_max] = {
  {"00", "00", "00", "00", "00", "00", "00", "00"},
  {"00", "00", "00", "00", "00", "00", "00", "00"},
  {"00", "00", "00", "00", "00", "00", "00", "00"}, //added sleep at Fn-F4 position
  {"00", "00", "00", "00", "00", "00", "00", "00"},
  {"00", "00", "00", "00", "00", "00", "00", "00"},
  {"00", "00", "00", "00", "00", "00", "00", "00"},
  {"00", "00", "00", "00", "00", "00", "00", "00"},
  {"00", "00", "00", "00", "00", "00", "00", "00"},
  {"00", "00", "00", "00", "00", "00", "00", "00"},
  {"00", "00", "00", "08", "00", "00", "00", "MEDIANEXT"}, //added next-track at Fn-Right position
  {"00", "00", "VOLUME+", "VOLUME-", "MUTE", "00", "00", "PLAYPAUSE"}, //added play-pause at Fn-Down
  {"00", "00", "00", "00", "00", "00", "00", "00"},
  {"BRIGHTNESS+", "BRIGHTNESS-", "00", "00", "00", "MEDIASTOP", "00", "MEDIAPREVIOUS"}, //added prev-track at Fn-Left position. Stop at Fn-Up position.
  {"00", "00", "00", "00", "00", "04", "00", "40"}, //added num-lock at Fn-Scroll lock position
  {"00", "00", "00", "02", "00", "00", "20", "00"},
  {"01", "00", "00", "00", "00", "00", "10", "00"}
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

// Define the Teensy 3.2 I/O numbers
// Row FPC pin # 22,18,14,10,02,04,08,12,06,20,16,24,28,32,26,30
// Teensy I/O  # 20,33,24,25,27,32,07,06,29,04,05,03,02,01,21,22
int Row_IO[rows_max] = {20, 33, 24, 25, 27, 32, 7, 6, 29, 4, 5, 3, 2, 1, 21, 22}; // Teensy 3.2 I/O numbers for rows

// Column FPC pin # 05,13,09,07,11,03,15,17
// Teensy I/O     # 16,10,12,17,11,15,09,08
int Col_IO[cols_max] = {16, 10, 12, 17, 11, 15, 9, 8}; // Teensy 3.2 I/O numbers for columns

// Declare variables that will be used by functions
boolean slots_full = LOW; // Goes high when slots 1 thru 6 contain normal keys
boolean slots_full_BLE = LOW;
// slot 1 thru slot 6 hold the normal key values to be sent over USB.
int slot1 = 0; //value of 0 means the slot is empty and can be used.
int slot2 = 0;
int slot3 = 0;
int slot4 = 0;
int slot5 = 0;
int slot6 = 0;

boolean isThinkvantagePressed = false;
boolean isBackPagePressed = false;
boolean isForwardPagePressed = false;
boolean isMicMutePressed = false;

String slot1BLE = "00"; //value of 0 means the slot is empty and can be used.
String slot2BLE = "00";
String slot3BLE = "00";
String slot4BLE = "00";
String slot5BLE = "00";
String slot6BLE = "00";

int modifierValueBLE = 0;

int mod_shift_l = 0; // These variables are sent over USB as modifier keys.
int mod_shift_r = 0; // Each is either set to 0 or MODIFIER_ ...
int mod_ctrl_l = 0;
int mod_ctrl_r = 0;
int mod_alt_l = 0;
int mod_alt_r = 0;
int mod_gui = 0;

boolean haveLoaded = false;
boolean haveUNloaded = false;

//
//************************************Setup*******************************************
void setup() {
  Serial.begin(BAUD);
  BLUEFRUIT_HWSERIAL_NAME.setTX(TX);
  BLUEFRUIT_HWSERIAL_NAME.setRX(RX);
  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  //  ble.println("AT+UARTFLOW=on");  ble.waitForOK();
  //  ble.println("AT+BAUDRATE=250000");  ble.waitForOK(); //=250000
  ble.println("AT+GAPDEVNAME=Orbs BLE Thinky");  ble.waitForOK(); //==Orbs Thinky
  //  ble.println("AT+HWMODELED=0");  ble.waitForOK();

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  /* Print Bluefruit information */
  ble.info();

  ble.println("AT+GAPGETCONN");  ble.waitForOK();
  ble.println("AT+UARTFLOW");  ble.waitForOK();
  //  ble.println("AT+BLEPOWERLEVEL=0");  ble.waitForOK();
  // This demo only available for firmware from 0.6.6
  if ( !ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    error(F("This sketch requires firmware version " MINIMUM_FIRMWARE_VERSION " or higher!"));
  }
  /* Enable HID Service (including Mouse) */
  Serial.println(F("Enable HID Service (including Mouse): "));
  if (! ble.sendCommandCheckOK(F( "AT+BleHIDEn=On"  ))) {
    error(F("Failed to enable HID (firmware >=0.6.6?)"));
  }
  /* Add or remove service requires a reset */
  Serial.println(F("Performing a SW reset (service changes require a reset): "));
  if (! ble.reset() ) {
    error(F("Could not reset??"));
  }

  Serial.println();

  // ************trackpoint setup
  trackpoint.reset();
  trackpoint.setSensitivityFactor(200);
  trackpoint.enable();

  attachInterrupt(digitalPinToInterrupt(TP_CLK), clockInterrupt, FALLING);
  // ************keyboard setup
  for (int a = 0; a < cols_max; a++) {  // loop thru all column pins
    go_pu(Col_IO[a]); // set each column pin as an input with a pullup
  }

  for (int b = 0; b < rows_max; b++) {  // loop thru all row pins
    go_z(Row_IO[b]); // set each row pin as a floating output
  }

  go_pu(HOTKEY);    // Pull up the Hotkey plus side for reading


}


elapsedMillis sinceLastConnCheck;
elapsedMillis  heldthinkvantageTime;;
//*********************************Main Loop*******************************************
void loop() {

  if (sinceLastConnCheck > 2500) {      // "sincePrint" auto-increases
    sinceLastConnCheck = 0;

    if (!bitRead(USB0_OTGSTAT, 5)) //  USB Connected
    {
      BLE_EN = false;
      USB_EN = true;

      //      Serial.print("Bluetooth disabled because USB connected");
      //      ble.println("Bluetooth disabled because USB connected");
    }
    else
    {
      BLE_EN = true;
      USB_EN = false;
      //      Serial.print("USB Disconnected, enabling BLE");
      //      ble.print("USB Disconnected, enabling BLE");
    }
  }
  if (AT_COMMAND_MODE)
  {
    // *************Deal with manual AT commands**************
    // Display command prompt
    Serial.print(F("AT > "));

    // Check for user input and echo it back if anything was found
    char command[BUFSIZE + 1];
    getUserInput(command, BUFSIZE);

    // Send command
    ble.println(command);

    // Check response status
    ble.waitForOK();
  }
  else
  {
    // *************Keyboard Main**************
    if (!digitalRead(HOTKEY)) // swap fn to ctrl
    {
      if (!haveLoaded) // is first press?
      {
        if (USB_EN)
        {
          load_mod(modifier[15][0]); // function reads which modifier key is pressed and loads it into the appropriate mod_... variable
          send_mod(); // function sends the state of all modifier keys over usb including the one that just got pressed
        }

        if (BLE_EN)
        {
          load_modBLE(modifierBLE[15][0]);
          sendKeysBLE();
        }
        haveLoaded = true;
        haveUNloaded = false;
      }
    }
    else
    {
      if (!haveUNloaded)
      {
        if (BLE_EN)
        {
          clear_modBLE(modifierBLE[15][0]);
          sendKeysBLE();
        }
        if (USB_EN)
        {
          clear_mod(modifier[15][0]); // function reads which modifier key was released and loads 0 into the appropriate mod_... variable
          send_mod(); // function sends all mod's over usb including the one that just released
        }
        haveUNloaded =  true;
        haveLoaded = false;
      }
    }

    // Scan keyboard matrix with an outer loop that drives each row low and an inner loop that reads every column (with pull ups).
    // The routine looks at each key's present state (by reading the column input pin) and also the previous state from the last scan
    // The status of a key that was just pressed or just released is sent over USB and the state is saved in the old_key matrix.
    // The keyboard keys will read as logic low if they are pressed (negative logic).
    // The old_key matrix also uses negative logic (low=pressed).





    for (int x = 0; x < rows_max; x++)
    { // loop thru the rows
      go_0(Row_IO[x]); // Activate Row (send it low)
      delayMicroseconds(10); // give the row time to go low and settle out
      for (int y = 0; y < cols_max; y++)
      { // loop thru the columns

        // **********Modifier keys including the Fn special case
        if (!isForwardPagePressed && (!digitalRead(Col_IO[y]) && ((x == 11) && (y == 7))))
        {
          if (USB_EN) {
            load_mod(KEY_LEFT_ALT );  // forward page
            send_mod();
            delay(50);
            load_slot(KEY_RIGHT);
            send_normals();
            delay(5);
            clear_slot(KEY_RIGHT);
            send_normals();
            delay(5);
            clear_mod(KEY_LEFT_ALT );
            send_mod();
            delay(5);
          }
          else if (BLE_EN)
          {
            load_modBLE(0x04);  // forward page
            sendKeysBLE();
            delay(50);
            load_slot_BLE("4F");
            sendKeysBLE();
            delay(5);
            clear_slot_BLE("4F");
            sendKeysBLE();
            delay(5);
            clear_modBLE(0x04);
            sendKeysBLE();
            delay(5);
          }
          isForwardPagePressed = true;
        }
        else if (isForwardPagePressed && ((x == 11) && (y == 7)) && digitalRead(Col_IO[y]))
        {
          isForwardPagePressed = false;
        }


        if (!isBackPagePressed && (!digitalRead(Col_IO[y]) && ((x == 11) && (y == 6))))
        {
          if (USB_EN) {
            load_mod(KEY_LEFT_ALT );  // back page
            send_mod();
            delay(50);
            load_slot(KEY_LEFT);
            send_normals();
            delay(5);
            clear_slot(KEY_LEFT);
            send_normals();
            delay(5);
            clear_mod(KEY_LEFT_ALT );
            send_mod();
            delay(5);
          }
          else if (BLE_EN)
          {
            load_modBLE(0x04);  // forward page
            sendKeysBLE();
            delay(50);
            load_slot_BLE("50");
            sendKeysBLE();
            delay(5);
            clear_slot_BLE("50");
            sendKeysBLE();
            delay(5);
            clear_modBLE(0x04);
            sendKeysBLE();
            delay(5);
          }
          isBackPagePressed = true;
        }
        else if (isBackPagePressed && ((x == 11) && (y == 6)) && digitalRead(Col_IO[y]))
        {
          isBackPagePressed = false;
        }


        if (!isThinkvantagePressed && (!digitalRead(Col_IO[y]) && ((x == 10) && (y == 5))))
        {
          heldthinkvantageTime = 0;
          isThinkvantagePressed = true;
        }
        else if (isThinkvantagePressed && ((x == 10) && (y == 5)) && digitalRead(Col_IO[y]))
        {
          isThinkvantagePressed = false;
          if (heldthinkvantageTime > 3000)
          {
            Serial.println("Disconnecting BLE and deleting bonds");
            ble.println("AT+GAPDISCONNECT");  ble.waitForOK();
            ble.println("AT+GAPDELBONDS");  ble.waitForOK();
          }
        }


        if (!isMicMutePressed && (!digitalRead(Col_IO[y]) && ((x == 10) && (y == 6))))
        {
          Serial.println("Mic Mute hit!");
          isMicMutePressed = true;
        }
        else if (isMicMutePressed && ((x == 10) && (y == 6)) && digitalRead(Col_IO[y]))
        {
          isMicMutePressed = false;
        }


        if (modifier[x][y] != 0)
        { // check if modifier key exists at this location in the array (a non-zero value)
          if (!digitalRead(Col_IO[y]) && (old_key[x][y]))
          { // Read column to see if key is low (pressed) and was previously not pressed
            if (BLE_EN)
            {
              if (((x == 15) && (y == 0))) // read the physical left ctrl as fn
              {
                Serial.println("fn pressed");
                Fn_pressed = LOW; //Save state of key as "pressed"
              }

              else
              {
                Serial.println("fn not pressed");
                load_modBLE(modifierBLE[x][y]);
                sendKeysBLE();
              }
            }
            if (USB_EN)
            {
              if (((x == 15) && (y == 0))) // read the physical left ctrl as fn
              {
                Fn_pressed = LOW; //Save state of key as "pressed"
              }
              else
              {
                load_mod(modifier[x][y]); // function reads which modifier key is pressed and loads it into the appropriate mod_... variable
                send_mod(); // function sends the state of all modifier keys over usb including the one that just got pressed
              }
            }
            old_key[x][y] = LOW; //  Fn key is not pressed
          }
          else if (digitalRead(Col_IO[y]) && (!old_key[x][y]))
          { //check if key is not pressed and was previously pressed
            if (BLE_EN)
            {
              if (((x == 15) && (y == 0)))
              {
                Fn_pressed = HIGH; // Fn key is not pressed (active low)
              }
              else
              {
                clear_modBLE(modifierBLE[x][y]);
                sendKeysBLE();
              }
            }
            if (USB_EN)
            {
              if (((x == 15) && (y == 0)))
              {
                Fn_pressed = HIGH; // Fn key is not pressed (active low)
              }
              else
              {
                clear_mod(modifier[x][y]); // function reads which modifier key was released and loads 0 into the appropriate mod_... variable
                send_mod(); // function sends all mod's over usb including the one that just released
              }
            }
            old_key[x][y] = HIGH; // Save state of key as "not pressed"
          }
        }
        // ***********end of modifier section

        // ***********Normal keys section and media keys in this section
        else if ((normal[x][y] != 0) || (media[x][y] != 0))
        { // check if normal or media key exists at this location in the array
          if (!digitalRead(Col_IO[y]) && (old_key[x][y]) && (!slots_full))
          { // check if key pressed and not previously pressed and slots not full
            old_key[x][y] = LOW; // Save state of key as "pressed"
            if (Fn_pressed)// Fn_pressed is active low so it is not pressed and normal key needs to be sent
            {
              if (BLE_EN)
              {
                if (media[x][y] == KEY_MEDIA_MUTE )
                {
                  send_control_HID(mediaBLE[x][y]);
                }
                else
                {
                  load_slot_BLE(normalBLE[x][y]);
                  sendKeysBLE();
                }
              }
              if (USB_EN)
              {
                if (media[x][y] == KEY_MEDIA_MUTE )
                {
                  Keyboard.press(media[x][y]); // media key is sent using keyboard press function per PJRC
                  delay(5); // delay 5 milliseconds before releasing to make sure it gets sent over USB
                  Keyboard.release(media[x][y]); // send media key release
                }
                else
                {
                  load_slot(normal[x][y]); //update first available slot with normal key name
                  send_normals(); // send all slots over USB including the key that just got pressed
                }
              }
            }
            else if (media[x][y] != 0 || mediaBLE[x][y] != "00")  // Fn is pressed so send media if a key exists in the matrix
            {
              if (BLE_EN)
              {
                if (mediaBLE[x][y] == "47") // special case for scroll lock
                {
                  load_slot_BLE(mediaBLE[x][y]);
                  sendKeysBLE();
                }
                else
                {
                  send_control_HID(mediaBLE[x][y]);
                }
              }
              if (USB_EN)
              {
                Keyboard.press(media[x][y]); // media key is sent using keyboard press function per PJRC
                delay(5); // delay 5 milliseconds before releasing to make sure it gets sent over USB
                Keyboard.release(media[x][y]); // send media key release
              }
            }
          }
          else if (!digitalRead(Col_IO[y]) && (media[x][y] == KEY_MEDIA_VOLUME_INC || media[x][y] == KEY_MEDIA_VOLUME_DEC))
          { // am holding vol buttons?
            if (BLE_EN)
            {
              send_control_HID(mediaBLE[x][y]);
              delay(50);
            }
            if (USB_EN)
            {

              Keyboard.press(media[x][y]); // media key is sent using keyboard press function per PJRC
              delay(5); // delay 5 milliseconds before releasing to make sure it gets sent over USB
              Keyboard.release(media[x][y]); // send media key release
              delay(50);
            }
          }
          else if (digitalRead(Col_IO[y]) && (!old_key[x][y]))
          { //check if key is not pressed, but was previously pressed
            old_key[x][y] = HIGH; // Save state of key as "not pressed"
            if (Fn_pressed)
            { // Fn is not pressed
              if (BLE_EN)
              {
                if (normalBLE[x][y] == "53") // special case for scroll lock
                  clear_slot_BLE("53");
                else
                  clear_slot_BLE(normalBLE[x][y]);
                sendKeysBLE();
              }
              if (USB_EN)
              {
                clear_slot(normal[x][y]); //clear the slot that contains the normal key name
                send_normals(); // send all slots over USB including the key that was just released
              }
            }
            else
            {
              if (BLE_EN)
              {
                if (mediaBLE[x][y] == "47") // special case for scroll lock
                  clear_slot_BLE(mediaBLE[x][y]);
                sendKeysBLE();
              }
            }
          }
        }
      }
      go_z(Row_IO[x]); // De-activate Row (send it to hi-z)
      // **************end of normal section
    }

    // **********keyboard scan complete
    //
    // ****************************Trackpoint Routine*********************************
    //

    if (trackpoint.reportReady())
    {
      noInterrupts();
      const DataReport& report = trackpoint.getDataReport();
      for (uint8_t i = 0; i < sizeof(buttonStatesBLE); ++i)
      {
        if (report.state & (1 << i))
        {
          USBbuttonClicked[i] = true;
          if (BLE_EN)
          {
            ble.print("AT+BleHidMouseButton=");
            ble.print(buttonStatesBLE[i]);
            ble.println(",press");
          }
          if (USB_EN)
          {
            Mouse.press(buttonStates[i]);
          }
        }
        else if (USBbuttonClicked[i])
        {
          USBbuttonClicked[i] = false;
          if (BLE_EN)
          {
            ble.print("AT+BleHidMouseButton=");
            ble.println("0");
          }
          if (USB_EN)
          {
            Mouse.release(buttonStates[i]);
          }
        }
      }
      if (USB_EN)
      {
        Mouse.move(report.x, -report.y);
      }
      if (BLE_EN)
      {
        char _x [10];
        char _y [10];
        itoa(report.x * MOUSE_MULTIPLIER, _x, 10);
        itoa(-report.y * MOUSE_MULTIPLIER, _y, 10);
        ble.print(F("AT+BleHidMouseMove="));
        ble.print(_x);
        ble.print(",") ;
        ble.println(_y) ;

      }
      interrupts();
    }

    // **************************************End of trackpoint routine***********************************

    // *******keyboard LEDs
    // Turn on or off the LEDs for Num Lock, Caps Lock, and Scroll Lock based on bit 0, 1, and 2 from the keyboard_leds
    // variable controlled by the USB host computer


    //    unsigned long currentMillis = millis();
    //
    //    if (currentMillis - previousMillis >= 250) {
    //      // save the last time you blinked the LED
    //      previousMillis = currentMillis;
    //
    //      // if the LED is off turn it on and vice-versa:
    //      if (ledState == LOW) {
    //        go_0(CAPS_LED);
    //      } else {
    //        go_1(CAPS_LED);
    //      }
    //      ledState = !ledState;
    // set the LED with the ledState of the variable:

    //  }

    //    if (keyboard_leds & 1 << 1) { // mask off all bits but D1 and test if set
    //      go_0(CAPS_LED); // turn on the Caps Lock LED
    //    }
    //    else {
    //      go_1(CAPS_LED); // turn off the Caps Lock LED
    //    }
  }
  // ****************End of main loop
}
