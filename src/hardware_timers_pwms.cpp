#include "hardware_timers_pwms.h"

// typedef HardwareTimer SAMDTimer;
bool _timerClockEnabled = false;

#define SAMD_TCC ((Tcc*)_HardTimer)

////////////////////////////////////////////////////////

void EnableTimerClock()
{
    if (!_timerClockEnabled) {

        /* This code seems killing the Serial port !!
        // Reset GCLK
        REG_GCLK_CTRL = GCLK_CTRL_SWRST;
        while (GCLK->STATUS.bit.SYNCBUSY);
        */
        REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(4);
        while (GCLK->STATUS.bit.SYNCBUSY)
            ;

        // Set GCLK4's prescaler in order to otbain 1 MHz
        REG_GCLK_GENDIV = GCLK_GENDIV_DIV(48) | GCLK_GENDIV_ID(4);
        while (GCLK->STATUS.bit.SYNCBUSY)
            ;

        // Connect GCLK4 to TCC0, TCC1, TCC2
        // !!! First disable it !?
        REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_TCC0_TCC1;
        // while (GCLK->STATUS.bit.SYNCBUSY)   // Not needed
        ;
        REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_TCC2_TC3;
        // while (GCLK->STATUS.bit.SYNCBUSY)   // Not needed
        ;
    }
    _timerClockEnabled = true;
}

timerCallback TCC0_callback = NULL;
void TCC0_Handler()
{
    // get timer struct
    Tcc* TCC = (Tcc*)TCC0;

    // If the compare register matching the timer count, trigger this interrupt
    if (TCC->INTFLAG.bit.MC0 == 1) {
        // A compare to cc0 caused the interrupt
        if (TCC0_callback != NULL)
            (*TCC0_callback)();
        TCC->INTFLAG.bit.MC0 = 1; // writing a one clears the flag ovf flag
    }

    if (TCC->INTFLAG.bit.OVF == 1) {
        //		if (TCC0_callback != NULL) (*TCC0_callback)();
        TCC->INTFLAG.bit.OVF = 1;
    }
}

timerCallback TCC1_callback = NULL;
void TCC1_Handler()
{
    // get timer struct
    Tcc* TCC = (Tcc*)TCC1;

    // If the compare register matching the timer count, trigger this interrupt
    if (TCC->INTFLAG.bit.MC0 == 1) {
        // A compare to cc0 caused the interrupt
        if (TCC1_callback != NULL)
            (*TCC1_callback)();
        TCC->INTFLAG.bit.MC0 = 1; // writing a one clears the flag ovf flag
    }

    if (TCC->INTFLAG.bit.OVF == 1) {
        //		if (TCC1_callback != NULL) (*TCC1_callback)();
        TCC->INTFLAG.bit.OVF = 1;
    }
}

void HardwareTimer::Init(timer_num_t timerNumber, timerCallback callback)
{
    _timer_number = timerNumber;

    EnableTimerClock();

    switch (_timer_number) {
    case TIMER_TCC0:
    case TIMER_TCC1:
    case TIMER_TCC2:
        if (_timer_number == TIMER_TCC0)
            _HardTimer = (Tcc*)TCC0;
        else if (_timer_number == TIMER_TCC1)
            _HardTimer = (Tcc*)TCC1;
        else
            _HardTimer = (Tcc*)TCC2;

        // From datasheet : The TCC should be disabled before the TCC is reset to avoid undefined behavior.
        disableTimer();
        // Reset TCC
        SAMD_TCC->CTRLA.reg |= TCC_CTRLA_SWRST;
        while (SAMD_TCC->SYNCBUSY.bit.SWRST == 1)
            ;

        // Set prescaler to 1 -> 1MHz
        SAMD_TCC->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV1;

        // Down-Counting in order to avoid count value outside 0..(old or new)TOP value (CC0)
        SAMD_TCC->CTRLBSET.reg = TCC_CTRLBSET_DIR;
        while (SAMD_TCC->SYNCBUSY.bit.CTRLB == 1)
            ;

        // Not in One-shot, but continuous mode
        SAMD_TCC->CTRLBCLR.reg = TCC_CTRLBCLR_ONESHOT;
        while (SAMD_TCC->SYNCBUSY.bit.CTRLB == 1)
            ;

        resetCounter();

        setPeriod(_period);

        SAMD_TCC->EVCTRL.reg |= TCC_EVCTRL_MCEO0;
        // Use Normal PWM operation mode
        SAMD_TCC->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM; // | TCC_WAVE_POL0;
        while (SAMD_TCC->SYNCBUSY.bit.WAVE == 1)
            ;

        // Invert output polarity
        SAMD_TCC->DRVCTRL.reg |= TCC_DRVCTRL_INVEN(1);

        // Enable the compare interrupt
        SAMD_TCC->INTENSET.reg = 0;
        // ToDo : Test difference between OVF and MC0
        SAMD_TCC->INTENSET.bit.OVF = 0;
        SAMD_TCC->INTENSET.bit.MC0 = 1;

        // Compare operation on channel 1 (CC1)
        //			SAMD_TCC->CTRLC.reg |= TCC_CTRLC_CPTEN1;
        //			while (SAMD_TCC->SYNCBUSY.bit.CTRLC == 1);

        setHighLevelDuration(HIGH_PULSE_US_DURATION);

        if (_timer_number == TIMER_TCC0) {
            _pCallback = &TCC0_callback;
            NVIC_EnableIRQ(TCC0_IRQn);
        }
        else if (_timer_number == TIMER_TCC1) {
            _pCallback = &TCC1_callback;
            NVIC_EnableIRQ(TCC1_IRQn);
        }
        else {
            // => TIMER_TCC2 =>
            // _pCallback = &TCC2_callback;
            // NVIC_EnableIRQ(TCC2_IRQn);
        }
        if (callback != NULL)
            *_pCallback = callback;
        //			maxCounter();
        enableTimer();
        stopTimer();

        break;
    default:
        // ToDo: Manage this error.
        break;
    }
};

void HardwareTimer::setFrequency(float frequency)
{
    _period = (unsigned long)(1000000.0f / frequency);
    setPeriod(_period);
}

void HardwareTimer::attachInterrupt(timerCallback callback)
{
    *_pCallback = callback;
}

timerCallback* HardwareTimer::getPCallback()
{
    return _pCallback;
}

void* HardwareTimer::getAddrPCallback()
{
    return &(_pCallback);
}

timerCallback HardwareTimer::getPtrCallback()
{
    return *_pCallback;
}

void HardwareTimer::disableInterrupt()
{
    if (_timer_number == TIMER_TCC0) {
        NVIC_DisableIRQ(TCC0_IRQn);
    }
    else if (_timer_number == TIMER_TCC1) {
        NVIC_DisableIRQ(TCC1_IRQn);
    }
}

void HardwareTimer::enableInterrupt()
{
    if (_timer_number == TIMER_TCC0) {
        NVIC_EnableIRQ(TCC0_IRQn);
    }
    else if (_timer_number == TIMER_TCC1) {
        NVIC_EnableIRQ(TCC1_IRQn);
    }
}

// Just stop clock source
void HardwareTimer::disableTimer()
{
    if ((_timer_number == TIMER_TCC0) || (_timer_number == TIMER_TCC1) || (_timer_number == TIMER_TCC2)) {
        SAMD_TCC->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
        while (SAMD_TCC->SYNCBUSY.bit.ENABLE == 1)
            ;
    }
}

// Just reconnect clock source
void HardwareTimer::enableTimer()
{
    if ((_timer_number == TIMER_TCC0) || (_timer_number == TIMER_TCC1) || (_timer_number == TIMER_TCC2)) {
        // Enable TCC
        SAMD_TCC->CTRLA.reg |= TCC_CTRLA_ENABLE;
        while (SAMD_TCC->SYNCBUSY.bit.ENABLE == 1)
            ;
    }
}

// Just clear the counter
void HardwareTimer::resetCounter()
{
    if ((_timer_number == TIMER_TCC0) || (_timer_number == TIMER_TCC1) || (_timer_number == TIMER_TCC2)) {
        // SAMD_TCC->COUNT.reg = 0xfffff0;
        SAMD_TCC->COUNT.reg = 0;
        while (SAMD_TCC->SYNCBUSY.bit.COUNT == 1)
            ;
    }
}

void HardwareTimer::setCounter(uint32_t value)
{
    if ((_timer_number == TIMER_TCC0) || (_timer_number == TIMER_TCC1) || (_timer_number == TIMER_TCC2)) {
        SAMD_TCC->COUNT.reg = value;
        while (SAMD_TCC->SYNCBUSY.bit.COUNT == 1)
            ;
    }
}

// Just read the counter
unsigned long HardwareTimer::getCounter()
{
    if ((_timer_number == TIMER_TCC0) || (_timer_number == TIMER_TCC1) || (_timer_number == TIMER_TCC2)) {
        // Read Synch
        SAMD_TCC->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;
        while (SAMD_TCC->SYNCBUSY.bit.CTRLB)
            ;
        return SAMD_TCC->COUNT.reg;
    }
    return 0;
}

// Just stop clock source and clear the count
void HardwareTimer::stopTimer()
{
    disableTimer();
    // Clear the counter
    resetCounter();
}

// Just clear the counter then reconnect clock source
void HardwareTimer::restartTimer()
{
    resetCounter();
    enableTimer();
}

// Just reconnect clock source
void HardwareTimer::startTimer()
{
    enableTimer();
}

// period in microseconds
void HardwareTimer::setPeriod(unsigned long period)
{
    _period = period;

    if ((_timer_number == TIMER_TCC0) || (_timer_number == TIMER_TCC1) || (_timer_number == TIMER_TCC2)) {
        // Use PER instead PERB because the last one will add 1 period more before updating
        SAMD_TCC->PER.reg = _period;
        while (SAMD_TCC->SYNCBUSY.bit.PER == 1)
            ;
    }
}

// duration in microseconds
void HardwareTimer::setHighLevelDuration(unsigned long duration)
{
    _hl_duration = duration;

    if ((_timer_number == TIMER_TCC0) || (_timer_number == TIMER_TCC1) || (_timer_number == TIMER_TCC2)) {
        // Use PER instead PERB because the last one will add 1 period more before updating
        SAMD_TCC->CC[0].reg = _hl_duration;
        while (SAMD_TCC->SYNCBUSY.bit.CC0 == 1)
            ;
    }
}

// duty cycle in percent (0..100)
void HardwareTimer::setDutyCycle(unsigned int duty_cycle)
{
    if (duty_cycle > 100)
        duty_cycle = 100;

    _hl_duration = _period * duty_cycle / 100;

    if ((_timer_number == TIMER_TCC0) || (_timer_number == TIMER_TCC1) || (_timer_number == TIMER_TCC2)) {
        // Use PER instead PERB because the last one will add 1 period more before updating
        SAMD_TCC->CC[0].reg = _hl_duration;
        while (SAMD_TCC->SYNCBUSY.bit.CC0 == 1)
            ;
    }
}

// Tables for looking up pin mappings etc. for different boards
typedef struct {
    const int arduinoPin; // Arduino pin number
    const unsigned int port; // Port of the SAMD21 pin
    const unsigned int samd21Pin; // SAMD21 pin
    const unsigned int timer; // Timer used for this pin
    const unsigned long int pMux; // Pin multiplexer for this pin
} PinLookup;

static const PinLookup pinTable[] = {
    // Table begin
    {-1, 0, 0, 0, 0},
    {-1, 0, 0, 0, 0},
    {-1, 0, 0, 0, 0},
    {-1, 0, 0, 0, 0},
    {-1, 0, 0, 0, 0},
    {-1, 0, 0, 0, 0},
    {6, PORTA, 4, 0, PORT_PMUX_PMUXE_E}, // LStep : TCC0-WO[0]
    {7, PORTA, 6, 1, PORT_PMUX_PMUXE_E}, // RStep : TCC1-WO[0]
    {-1, 0, 0, 0, 0},
    {-1, 0, 0, 0, 0},
    {-1, 0, 0, 0, 0},
    {11, PORTA, 16, 2, PORT_PMUX_PMUXE_E}, // HighCurrentPWM : TCC2-WO[0]
    {-1, 0, 0, 0, 0},
    {-1, 0, 0, 0, 0}
    // Table end
};
static const unsigned int pinTableSize = sizeof(pinTable) / sizeof(pinTable[0]);

int HardwarePWM::pinAssign(int pin)
{
    // Check if an acceptable pin is used
    unsigned int i;
    for (i = 0; i < pinTableSize; i++) {
        if (pinTable[i].arduinoPin == pin) {
            break;
        }
    }
    if (i >= pinTableSize || pin < 0) {
        return 0;
    }

    // Enable a SAMD21 pin as multiplexed and connect it to a pin using the port multiplexer
    PORT->Group[pinTable[pin].port].PINCFG[pinTable[pin].samd21Pin].bit.PMUXEN = 1;
    PORT->Group[pinTable[pin].port].PMUX[pinTable[pin].samd21Pin >> 1].reg |= pinTable[pin].pMux;
    return 1;
}
