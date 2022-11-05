#include "trackpoint.h"

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
}

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
    digitalWrite(pinReset, HIGH);

    delay(2500);

    digitalWrite(pinReset, LOW);

    read(); // 0xAA
    read(); // 0x00

    inhibitCommunication();
}

void TrackPoint::enable() const
{
    write(0xf4);
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
