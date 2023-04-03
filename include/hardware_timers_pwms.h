
#ifndef HARDWARE_TIMERS_PWMS_H
#define HARDWARE_TIMERS_PWMS_H

#include <Arduino.h>

#define TIMER_HZ 1000000L

#define HIGH_PULSE_US_DURATION 200

typedef enum {
    TIMER_TCC0, // WO[0] -> LeftStep
    TIMER_TCC1, // WO[0] -> RightStep
    TIMER_TCC2, // WO[0] -> HighCurrentPWM
    MAX_TIMER
} timer_num_t;

typedef void (*timerCallback)();

class HardwarePWM {
public:
    int pinAssign(int pin);
}; // class HardwarePWM

class HardwareTimer {
private:
    timer_num_t _timer_number;
    void* _HardTimer = NULL; // point to timer struct, (TcCount16*) for TCx or (Tcc*) for TCCx
    timerCallback* _pCallback = NULL; // pointer to the TCx_callback function
    unsigned long _period = 0; // any value but not 0
    unsigned long _hl_duration = HIGH_PULSE_US_DURATION;

    void attachInterrupt(timerCallback callback);
    timerCallback* getPCallback();
    void* getAddrPCallback();
    timerCallback getPtrCallback();
    void disableInterrupt();
    void enableInterrupt();
    void disableTimer();
    void enableTimer();
    void resetCounter();
    void setCounter(uint32_t value);
    void restartTimer();
    void setHighLevelDuration(unsigned long duration); // duration in us

public:
    void Init(timer_num_t timer_number, timerCallback callback);
    void setFrequency(float frequency); // frequency in hertz
    void setPeriod(unsigned long period); // period in us
    void setDutyCycle(unsigned int duty_cycle); // 0 to 100 % of configurated period
    void stopTimer();
    void startTimer();
    unsigned long getCounter();
}; // class HardwareTimer

#endif /* HARDWARE_TIMERS_PWMS_H */
