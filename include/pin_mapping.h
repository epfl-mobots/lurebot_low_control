#ifndef PIN_MAPPING_H
#define PIN_MAPPING_H

// Definitions for GPIOs
#define ERROR_LED_PIN 13 // Led Pin
#define ERROR_LED_LIGHTUP_STATE HIGH // Set led to on/off

// Left motor controller GPIOs
#define LEFT_STEP 6
#define LEFT_DIR 5
#define LEFT_ENABLE 4
#define LEFT_DECAY 3
#define LEFT_HC 2

// Right motor controller GPIOs
#define RIGHT_STEP 7
#define RIGHT_DIR 10
#define RIGHT_ENABLE 9
#define RIGHT_DECAY 8
#define RIGHT_HC 11

// IR sensor GPIOs
#define PULSE_0 12
#define IR_0 A0
#define PULSE_1 0
#define IR_1 A1
#define PULSE_2 1
#define IR_2 A2

// Voltage measure
#define VSENS A3

// A6 and A7 lines
#define GPIO_A6 20
#define GPIO_A7 21

// Undervoltage check : C5 and C7 on Arduino Nano 33 IoT must be removed if this GPIO must be used as input (or interrupt)
// For the moment the track on this GPIO (bottom side) has been cutted in order to be able to check with an oscilloscope
// if this undervoltage condition races or not. Can be used later on this GPIO with Arduino modification or directly use as
// /Reset signal by J1 jumper closed.
#define UNDERVOLTAGE 33

#endif