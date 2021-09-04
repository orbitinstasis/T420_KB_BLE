#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLE.h"
#include "trackpoint.h"



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
void load_slot(int key)
{
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

void load_slot_BLE(String key)
{
  if (slot1BLE == "00")  {
    slot1BLE = key;
  }
  else if (slot2BLE == "00") {
    slot2BLE = key;
  }
  else if (slot3BLE == "00") {
    slot3BLE = key;
  }
  else if (slot4BLE == "00") {
    slot4BLE = key;
  }
  else if (slot5BLE == "00") {
    slot5BLE = key;
  }
  else if (slot6BLE == "00") {
    slot6BLE = key;
  }
  //  if (slot1BLE == "00" || slot2BLE == "00" || slot3BLE == "00" || slot4BLE == "00" || slot5BLE == "00" || slot6BLE == "00")  {
  //    slots_full_BLE = LOW; // slots are not full
  //  }
  //  else {
  //    slots_full_BLE = HIGH; // slots are full
  //  }
}

void clear_slot_BLE(String key)
{
  if (slot1BLE == key) {
    slot1BLE = "00";
  }
  else if (slot2BLE == key) {
    slot2BLE = "00";
  }
  else if (slot3BLE == key) {
    slot3BLE = "00";
  }
  else if (slot4BLE == key) {
    slot4BLE = "00";
  }
  else if (slot5BLE == key) {
    slot5BLE = "00";
  }
  else if (slot6BLE == key) {
    slot6BLE = "00";
  }
  //  if (slot1BLE == "00" || slot2BLE == "00" || slot3BLE == "00" || slot4BLE == "00" || slot5BLE == "00" || slot6BLE == "00")  {
  //    slots_full_BLE = LOW; // slots are not full
  //  }
  //  else {
  //    slots_full_BLE = HIGH; // slots are full
  //  }
}

// Function to clear the slot that contains the key name
void clear_slot(int key)
{
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

// Function to load the modifier key name into the appropriate mod variable
void load_mod(int m_key)
{
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

// Function to load the modifier key name into the appropriate mod variable
void load_modBLE(int m_key)
{
  if (m_key == 0x02)  {
    mod_shift_l = m_key;
  }
  else if (m_key == 0x20)  {
    mod_shift_r = m_key;
  }
  else if (m_key == 0x01)  {
    mod_ctrl_l = m_key;
  }
  else if (m_key == 0x10)  {
    mod_ctrl_r = m_key;
  }
  else if (m_key == 0x04)  {
    mod_alt_l = m_key;
  }
  else if (m_key == 0x40)  {
    mod_alt_r = m_key;
  }
  else if (m_key == 0x08)  {
    mod_gui = m_key;
  }
  set_modifierBLE();
}

// Function to load 0 into the appropriate mod variable
void clear_modBLE(int m_key)
{
  if (m_key == 0x02)  {
    mod_shift_l = 0;
  }
  else if (m_key == 0x20)  {
    mod_shift_r = 0;
  }
  else if (m_key == 0x01)  {
    mod_ctrl_l = 0;
  }
  else if (m_key == 0x10)  {
    mod_ctrl_r = 0;
  }
  else if (m_key == 0x04)  {
    mod_alt_l = 0;
  }
  else if (m_key == 0x40)  {
    mod_alt_r = 0;
  }
  else if (m_key == 0x08)  {
    mod_gui = 0;
  }
  set_modifierBLE();
}

// Function to load 0 into the appropriate mod variable
void clear_mod(int m_key)
{
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

void sendKeysBLE()
{
  char _s [16];
  itoa(modifierValueBLE, _s, 16);
  String s = String(_s);
  if (s.toInt() < 10)
    s = "0" + s;
  ble.print("AT+BLEKEYBOARDCODE=");
  ble.print(s);
  ble.print("-00");
  ble.print("-" + slot1BLE);
  ble.print("-" + slot2BLE);
  ble.print("-" + slot3BLE);
  ble.print("-" + slot4BLE);
  ble.print("-" + slot5BLE);
  ble.print("-" + slot6BLE);
  ble.println("");
}

void set_modifierBLE()
{
  modifierValueBLE = mod_shift_l | mod_shift_r | mod_ctrl_l | mod_ctrl_r | mod_alt_l | mod_alt_r | mod_gui;
}

void send_control_HID(String s)
{
  ble.print("AT+BLEHIDCONTROLKEY=");
  ble.println(s);
}

// Function to send the modifier keys over usb
void send_mod()
{
  Keyboard.set_modifier(mod_shift_l | mod_shift_r | mod_ctrl_l | mod_ctrl_r | mod_alt_l | mod_alt_r | mod_gui);
  Keyboard.send_now();
}

//
// Function to send the normal keys in the 6 slots over usb
void send_normals()
{

  Keyboard.set_key1(slot1);
  Keyboard.set_key2(slot2);
  Keyboard.set_key3(slot3);
  Keyboard.set_key4(slot4);
  Keyboard.set_key5(slot5);
  Keyboard.set_key6(slot6);
  Keyboard.send_now();
}




// A small helper
void error(const __FlashStringHelper*err)
{
  Serial.println(err);
  while (1);
}


/**************************************************************************/
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/
void getUserInput(char buffer[], uint8_t maxSize)
{
  memset(buffer, 0, maxSize);
  while ( Serial.available() == 0 ) {
    delay(1);
  }

  uint8_t count = 0;

  do
  {
    count += Serial.readBytes(buffer + count, maxSize);
    delay(2);
  } while ( (count < maxSize) && !(Serial.available() == 0) );
}
