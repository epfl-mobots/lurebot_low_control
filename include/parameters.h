
#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <lurebot_firmware_version.h>

// #define ENABLE_LOGGING
// #define ENABLE_TEMPERATURE_SENSORS

#define MAX_TEMP 95 // max temperature in Celsius

#define MAX_BUFFER_SIZE 20
#define DROPPED_MSG_BUFFER_SIZE 16
#define MOTOR_VEL_CHAR_SIZE 8
#define MOTOR_CVEL_CHAR_SIZE 4
#define IR_VAL_CHAR_SIZE 4
#define TEMP_BUFF_SIZE 2

// Sleep times in millisecond
#define CMD_SLEEP_MS 10
#define NUM_LOST_HEARTBEATS 10

#define ROBOT_ID "1"
#define LUREBOT_BLE_NAME "[" ROBOT_ID "] - LureBot"

#define LUREBOT_NAME_BUF_SIZE 20
#define FW_VERSION_BUF_SIZE 15

// Period in milisecond for speed controller
#define SPEED_CONTROL_PERIOD_MS 10
// To convert acceleration m/s2 in cm/s2 - Centimeter by meter
#define CM_BY_M 100
// To convert milisecond in second
#define MS_BY_S 1000
#define CYCLE_BY_S (MS_BY_S / SPEED_CONTROL_PERIOD_MS)
#define MAX_DV_IN_CM_BY_CYCLE (max_accel * CM_BY_M / CYCLE_BY_S)
#define MIN_ACCEL_TO_CONSIDER 0.1
#define MAX_ACCEL_TO_CONSIDER 1.75

// Stepper motor - Full step angle: 15 degre
// Full steps per revolution: 24
// Motor used in 1/4 step -> 96 ustep/revolution -> 96 period of timer generating the STEP signal

// Wheel diameter: d = 2 cm
// Perimeter: d * PI = 6.2832 cn
// Step/sec for 1cm/s: 96 / 6.2832

// As the timers are on 16 bits then limited to 65535 the minimal speed can't be smaller than
#define SPEED_MIN_CM_BY_S 0.5

#endif /* PARAMETERS_H */
