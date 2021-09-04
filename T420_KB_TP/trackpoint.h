#pragma once
#include <Arduino.h>

struct DataReport
{
  DataReport();

  void pushBit(uint8_t b);

  uint8_t state;
  int8_t x;
  int8_t y;

  volatile uint8_t data;
  volatile int8_t bitCount;
  volatile uint8_t byte;
  volatile uint8_t reportAvailable;
};

class TrackPoint
{
  public:
    TrackPoint(const uint8_t pinClock, const uint8_t pinData, const uint8_t pinReset);
    void reset() const;
    void enable() const;
    void readData() const;
    void writeToRamLocation(uint8_t, uint8_t);
    void setSensitivityFactor(uint8_t);
    const uint8_t reportReady() const;
    const DataReport& getDataReport();
    uint8_t read() const;
    void send_tp_arg(byte);
    void identify(void);
    void special_sequence(int, uint8_t );
    void read_capabilities(void);
    void read_modelid(void);
    void set_mode(uint8_t);
    uint8_t read_modes(void);
    void status_request(void);

  private:
    void write(const uint8_t data) const;

    bool read_ack(void);
    inline void waitForLow(const uint8_t pin) const;
    inline void waitForHigh(const uint8_t pin) const;
    inline void inhibitCommunication() const;
    inline void waitForClockAndSendBit(const uint8_t val) const;

    uint8_t pinClock;
    uint8_t pinData;
    uint8_t pinReset;

    DataReport dataReport;
};
