#include "trackpoint.h"

#define PAD_REMOTE_MODE 0x01 // only remote mode supported 
#define PAD_STREAM_MODE 0x02 // for reference only. Not used

#define PAD_STATUS_ENABLED  0x01
#define PAD_STATUS_DISABLED 0x02

#define PAD_SEQ_INFO      0x01 // information queries command sequence 
#define PAD_SEQ_SET_MODE  0x02 // set mode command sequence

#define PAD_MODE_ABSOLUTE 0x80 // bit7 1 to select Absolute mode
#define PAD_MODE_RATE_80  0x40 // bit6 1 to select high (80) packet rate
#define PAD_MODE_SLEEP    0x30 // bit3 1 to select sleep mode
#define PAD_MODE_DISGEST  0x20 // bit2 1 to disable detection of tap and drag gestures
#define PAD_MODE_PACKETSIZE 0x10 // not used in ps2 mode
#define PAD_MODE_WMODE    0x01  // 0 to select normal Absolute mode packets, or 1 to select
// enhanced Absolute packets that contain the “W” value
#define PAD_MODE_COMMON_00  0x01 // Always OK Relative mode
#define PAD_MODE_COMMON_04  0x04 // Version 4.x or later Relative mode with gestures disabled
#define PAD_MODE_COMMON_40  0x40 // Always OK Relative mode with high packet rate
#define PAD_MODE_COMMON_80  0x80 // capExtended = 0 Absolute mode
#define PAD_MODE_COMMON_81  0x81 // capExtended = 1 Absolute mode with W
#define PAD_MODE_COMMON_C0  0xC0 // capExtended = 0 Absolute mode with high packet rate
#define PAD_MODE_COMMON_C1  0xC1 // capExtended = 1 Absolute mode with W, high packet rate
#define PAD_MODE_COMMON_0C  0x0C // capSleep = 1 Low-power sleep mode

DataReport::DataReport():
  data(0), bitCount(-1), byte(0)
{
}

// errors are ignored
void TrackPoint::writeToRamLocation(uint8_t location, uint8_t value) {
  write(0xe2);
  read(); // ACK
  write(0x81);
  read(); // ACK
  write(location);
  read(); // ACK
  write(value);
  read(); // ACK
}//

void TrackPoint::setSensitivityFactor(uint8_t sensitivityFactor) {
  writeToRamLocation(0x4a, sensitivityFactor);
}

void DataReport::pushBit(uint8_t b)
{
  reportAvailable = 0;

  switch (++bitCount)
  {
    case 0: // start bit
      break;
    case 1: // data bit 0
    case 2: // data bit 1
    case 3: // data bit 2
    case 4: // data bit 3
    case 5: // data bit 4
    case 6: // data bit 5
    case 7: // data bit 6
    case 8: // data bit 7
      data >>= 1;
      if (b)
        data |= 0x80;
      break;
    case 9: // parity bit
      break;
    case 10: // stop bit
      switch (byte)
      {
        case 0:
          state = data;
          byte++;
          break;
        case 1:
          x = data;
          byte++;
          break;
        case 2:
          y = data;
          byte = 0;
          reportAvailable = 1;
          break;
      }
    default:
      data = 0;
      bitCount = -1;
  }
}


TrackPoint::TrackPoint(const uint8_t pinClock, const uint8_t pinData, const uint8_t pinReset):
  pinClock(pinClock), pinData(pinData), pinReset(pinReset)
{
}



void TrackPoint::reset() const
{
  pinMode(pinReset, OUTPUT);
  digitalWrite(pinReset, LOW);
  delay(100);
  pinMode(pinReset, OUTPUT);
  digitalWrite(pinReset, HIGH);
  delay(100);
  digitalWrite(pinReset, LOW);
  write(0xff); // sending reset command
  read(); // 0xFA ack

  delay(1000); // tp must run it's self diagnostic for 1 second
  read(); // 0xAA
  read(); // 0x00

  inhibitCommunication();
}

void TrackPoint::special_sequence(int sequence_type, uint8_t  param)
{

  // send initial sequence
  write(0xe8); read();
  write((param >> 6) & 0x03); read();
  write(0xe8); read();
  write((param >> 4) & 0x03); read();
  write(0xe8); read();
  write((param >> 2) & 0x03); read();
  write(0xe8); read();
  write(param & 0x03); read();
  // send command
  if (sequence_type == PAD_SEQ_INFO)
  {
    write(0xe9);
    read();
    // returns data
    Serial.println(read(), BIN);
    Serial.println(read(), BIN);
    Serial.println(read(), BIN);
  } else if (sequence_type == PAD_SEQ_SET_MODE)
  {
    write(0xf3);
    read();
    write(0x14);
    read();
    //no result
  }

}



/*        bit7    bit6    bit5    bit4    bit3    bit2    bit1   bit0
    ____________________________________________________________________
  byte1|                            InfoMinor
  byte2|                              0x047
  byte3|            infoModelCode       |           infoMajor
*/
void TrackPoint::identify(void)
{
  Serial.println("Identify: ");
  special_sequence(PAD_SEQ_INFO, 0x00);
}

/**
  returns 3 bytes device capabilities.
          bit7      bit6    bit5    bit4    bit3    bit2    bit1    bit0
    ________________________________________________________________________
  byte1|   cExended     -       -      -        -      -        -       -
  byte2|                                  0x047
  byte3|     -          -       -     cSleep cFourBtn  -   cMultiFIng   cPalmDet
  capExtended (bit 15)
  This bit is set if the extended capability bits are supported. The host can
  examine this bit to see whether the other 15 extended capability bits are
  present;
  capSleep (bit 4)
  For the PS/2 protocol, the capSleep bit is set if sleep mode is supported
  capFourButtons (bit 3)
  For the PS/2 protocol, this bit is set if the pad is a “MultiSwitch” pad which
  supports four mouse buttons labeled Left, Right, Up, and Down. In the
  PS/2 protocol, the Up and Down buttons are reported only during Absolute
  Mode with the Wmode bit set
  capMultiFinger (bit 1)
  This bit is set if multi-finger detection is supported. The pad is then able to
  count the number of simultaneous fingers on the pad and report the finger
  count via the W field of the Absolute packet. If this bit is 0, the pad does
  not support multi-finger detection; any finger contact will be assumed to be
  a single finger, with the position reported as the midpoint between all actual
  fingers, and, if capPalmDetect is set, with W reporting a (typically large)
  “width” for the assumed single finger.
  capPalmDetect (bit 0)
  This bit is set if “palm detection” is supported. In “W mode,” the TouchPad
  measures the apparent size or width of the finger and reports the width in
  the W field of the Absolute mode packet. The host can use this information
  to help distinguish between intentional finger contact and accidental palm
  or hand contact.
*/
void TrackPoint::read_capabilities(void)
{
  Serial.println("Read Capabilities: ");
  special_sequence(PAD_SEQ_INFO, 0x02);
}

/**
  returns the mode byte.
  bit(7): 1=>absolute mode; 0=>relative mode
  bit(6): 0=>40; 1=>80; packet per second
  bit(3): 1=>sleep; 0=>normal;
  bit(2): This bit is 0 to enable “tap” and “drag” gesture processing, or 1 to disable
      detection of tap and drag gestures. When this bit is 1, the Relative mode
      mouse packet reports the true physical button states, and the Absolute mode
      packet’s Gesture bit always reports as zero. The DisGest bit is implemented
      only for 40.x and later TouchPads (i.e., when infoMajor ≥ 4); for older pads,
      the bit is reserved.
  bit(0): 0=>normal Absolute mode packets; 1=>enhanced Absolute packets that contain the “W”
*/
uint8_t TrackPoint::read_modes(void)
{
  Serial.println("Read Mode (third byte): ");
  special_sequence(PAD_SEQ_INFO, 0x01);
}

/*
  Response is an ACK ($FA), followed by a 3-byte status
  packet consisting of the following data
        bit7    bit6    bit5    bit4    bit3    bit2    bit1    bit0
  ____________________________________________________________________
  byte1|   0     Remote  Enable  Scaling    0     Left   Middle  Right
  byte2|   0        0       0       0       0       0      Resolution
  byte3|                           Sample Rate
  Remote: 1 = Remote (polled) mode, 0 = Stream mode.
  Enable: 1 = Data reporting enabled, 0 = disabled. This bit only has effect in Stream mode.
  Scaling: 1 = Scaling is 2:1, 0 = scaling is 1:1. See commands $E6 and $E7 below.
  Left: 1 = Left button is currently pressed, 0 = released.
  Middle: 1 = Middle button is currently pressed, 0 = released.
  Right: 1 = Right button is currently pressed, 0 = released.
  Resolution: The current resolution setting, from 0 to 3 (1, 2, 4, and 8 counts per mm).
  Sample rate: The current sample rate setting, from 10 to 200.
*/
void TrackPoint::status_request(void)
{
  Serial.println("Status Request: ");
  write(0xe9);
  read();
  Serial.println(read(), BIN);
  Serial.println(read(), BIN);
  Serial.println(read(), BIN);
}

/**
  set mode for pad. See section 2.5 for details (or .h file)
*/
void TrackPoint::set_mode(uint8_t mode)
{
  special_sequence(PAD_SEQ_SET_MODE, mode);
  //  Serial.println(mode)
}

void TrackPoint::read_modelid(void)
{
  Serial.println("Read Model ID: ");
  special_sequence(PAD_SEQ_INFO, 0x03);
}

void TrackPoint::send_tp_arg(byte arg) {
  byte i;
  for (i = 0; i < 4; i++) {
    write(0xe8); read();
    write((arg >> (6 - 2 * i)) & 3); read();
  }
}

void TrackPoint::enable() const
{
  write(0xe8); //  Sending resolution command
  read(); // 0xFA
  write(0x03); // value of 0x03 = 8 counts/mm resolution (default is 4 counts/mm)
  read(); // 0xFA

  write(0xf3);    // change sample rate
  read(); // 0xFA
  write(100);    // 80sps is max in steam mode
  read(); // 0xFA

  write(0xE7);    // set scaling 2:1
  read(); // 0xFA

  write(0xf4);    // enable
  read(); // 0xFA

  pinMode(pinClock, INPUT);
  pinMode(pinData, INPUT);
}

void TrackPoint::readData() const
{
  if (digitalRead(pinClock) != HIGH)
  {
    dataReport.pushBit(digitalRead(pinData));
  }
}

const uint8_t TrackPoint::reportReady() const
{
  return dataReport.reportAvailable;
}

const DataReport& TrackPoint::getDataReport()
{
  dataReport.reportAvailable = 0;
  return dataReport;
}

void TrackPoint::write(const uint8_t data) const
{
  bool parity = true;

  inhibitCommunication();

  delayMicroseconds(100);

  // start bit (always 0)
  pinMode(pinData, OUTPUT);
  digitalWrite(pinData, LOW);

  delayMicroseconds(15);

  // ready for receiving clock pulses
  pinMode(pinClock, INPUT);

  // send data
  for (int i = 0; i < 8; ++i)
  {
    if (data & (1 << i))
    {
      parity = !parity;

      waitForClockAndSendBit(HIGH);
    }
    else
    {
      waitForClockAndSendBit(LOW);
    }
  }

  // send parity
  waitForClockAndSendBit(parity ? HIGH : LOW);

  // send stop bit
  waitForClockAndSendBit(HIGH);

  // waiting for line control bit
  pinMode(pinData, INPUT);

  waitForLow(pinData);
  waitForLow(pinClock);
  waitForHigh(pinClock);
  waitForHigh(pinData);

  inhibitCommunication();
}

uint8_t TrackPoint::read() const
{
  uint8_t result = 0;

  pinMode(pinData, INPUT);
  pinMode(pinClock, INPUT);

  // start bit (always 0)
  waitForLow(pinClock);
  // ignoring actual value
  waitForHigh(pinClock);

  for (int i = 0; i < 8; ++i)
  {
    waitForLow(pinClock);
    delayMicroseconds(15);
    result |= digitalRead(pinData) << i;
    waitForHigh(pinClock);
  }

  // parity
  waitForLow(pinClock);
  // ignoring actual value
  waitForHigh(pinClock);

  // stop bit
  waitForLow(pinClock);
  // ignoring actual value
  waitForHigh(pinClock);

  inhibitCommunication();

  return result;
}

inline void TrackPoint::waitForLow(const uint8_t pin) const
{
  while (digitalRead(pin) == HIGH);
}

inline void TrackPoint::waitForHigh(const uint8_t pin) const
{
  while (digitalRead(pin) == LOW);
}

inline void TrackPoint::inhibitCommunication() const
{
  pinMode(pinClock, OUTPUT);
  digitalWrite(pinClock, LOW);
}

inline void TrackPoint::waitForClockAndSendBit(const uint8_t val) const
{
  waitForLow(pinClock);
  delayMicroseconds(15);
  digitalWrite(pinData, val);
  waitForHigh(pinClock);
}
