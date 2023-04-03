/**
 *
 * @brief LureBot Custom Firmware
 *  Targeted Arduino Circuit:
 *              -  Arduino Nano IoT
 *              -  LureBot Board
 * @author Vaios Papaspyros:
 *              personal email: b.papaspyros@gmail.com
 *              academic email: vaios.papaspyros@epfl.ch
 * @author Daniel Burnier:
 *              email: daniel.burnier@epfl.ch
 *
 **/

#if (defined(ARDUINO_SAMD_NANO_33_IOT))

#include <FreeRTOS_SAMD21.h>

#else
#warning ARDUINO_SAMD_NANO_33_IOT not defined in lurebot_low_control.cpp
#endif

#include <Arduino.h>
#include <ArduinoBLE.h>

#include <parameters.h>
#include <utility.h>
#include <pin_mapping.h>
#include <uuid_defs.h>

#include <hardware_timers_pwms.h>

#include <limits>

#ifdef ENABLE_TEMPERATURE_SENSORS
#include <SparkFun_TMP117.h>
#include <Wire.h>
TMP117 tmp117;
// #define L_TMP117_ADDR 0x48
#define TMP117_ADDR 0x49
#endif

// ir sensor poll flag
bool poll_ir = false;

// Left motor speed in cm/s
float l_target_speed = 0;
float l_current_speed = 0; // ! MUST never be changed outside periodControl
bool l_must_be_reset = false;
int32_t l_current_period = 0;

// Right motor speed in cm/s
float r_target_speed = 0;
float r_current_speed = 0; // ! MUST never be changed outside periodControl
bool r_must_be_reset = false;
int32_t r_current_period = 0;

// max acceleration in m/s^2
float max_accel = 1.75;

// count dropped packets
uint64_t dropped_msg_count = 0;
uint64_t total_msg_count = 0;

// current msg id
uint16_t internal_id = 0;

// heartbeat counter to make sure to disable the motors if communication has halted
int heartbeat = NUM_LOST_HEARTBEATS; // this translates to CMD_SLEEP_MS * NUM_LOST_HEARTBEATS cycles with no received message

// Hardware timers
HardwareTimer left_motor_timer;
HardwareTimer right_motor_timer;
HardwareTimer lowCurrentTimer;

// Hardware PWMs
HardwarePWM left_step;
HardwarePWM right_step;
HardwarePWM right_HC;

// BLE Services and Characeristics
BLEService motor_vel_srv(MOTOR_VEL_SRV_UUID);
BLECharacteristic desired_vel_char(DESIRED_VEL_CHAR_UUID, BLERead | BLEWriteWithoutResponse, MOTOR_VEL_CHAR_SIZE, true);
BLEByteCharacteristic return_current_vel_char(RETURN_CURRENT_VEL_CHAR_UUID, BLERead | BLEWriteWithoutResponse);
BLECharacteristic current_vel_char(CURRENT_VEL_CHAR_UUID, BLERead | BLENotify, MOTOR_CVEL_CHAR_SIZE, true);

BLEService accel_srv(ACCEL_SRV_UUID);
BLEByteCharacteristic max_accel_char(MAX_ACCEL_CHAR_UUID, BLERead | BLEWriteWithoutResponse);

BLEService duty_cycle_srv(DUTY_CYCLE_SRV_UUID);
BLEByteCharacteristic set_duty_cycle_char(SET_DUTY_CYCLE_CHAR_UUID, BLERead | BLEWriteWithoutResponse);

// BLEService ir_srv(IR_SRV_UUID);
// BLEByteCharacteristic set_ir_char(SET_IR_CHAR_UUID, BLERead | BLEWriteWithoutResponse);
// BLECharacteristic ir_val_char(IR_VAL_CHAR_UUID, BLERead | BLENotify, IR_VAL_CHAR_SIZE, true);

BLEService dropped_msgs_srv(DROPPED_MSGS_SRV_UUID);
BLEByteCharacteristic return_dropped_msgs_char(RETURN_DROPPED_MSGS_CHAR_UUID, BLERead | BLEWriteWithoutResponse);
BLECharacteristic dropped_msgs_count_char(DROPPED_MSGS_COUNT_CHAR_UUID, BLERead | BLENotify, DROPPED_MSG_BUFFER_SIZE, true);

BLEService info_srv(INFO_SRV_UUID);
BLEStringCharacteristic device_name_char(DEVICE_NAME_CHAR_UUID, BLERead, LUREBOT_NAME_BUF_SIZE);
BLEStringCharacteristic fw_version_char(FW_VERSION_CHAR_UUID, BLERead, FW_VERSION_BUF_SIZE);

BLEService heartbeat_srv(HEARTBEAT_SRV_UUID);
BLEByteCharacteristic heartbeat_char(HEARTBEAT_CHAR_UUID, BLERead | BLENotify);

BLEService temperature_srv(TEMPERATURE_SRV_UUID);
BLEByteCharacteristic return_current_temp_char(RETURN_CURRENT_TEMP_CHAR_UUID, BLERead | BLEWriteWithoutResponse);
BLECharacteristic temperature_char(TEMPERATURE_CHAR_UUID, BLERead | BLENotify, TEMP_BUFF_SIZE, true);

union MotorCmd {
    uint8_t bytes[MOTOR_VEL_CHAR_SIZE];
    uint16_t cmds[MOTOR_VEL_CHAR_SIZE / 2];
};

union DroppedMsg {
    uint8_t bytes[2 * DROPPED_MSG_BUFFER_SIZE];
    uint64_t counters[2];
};

union MotorSpeeds {
    uint8_t bytes[MOTOR_CVEL_CHAR_SIZE];
    uint16_t cmds[MOTOR_CVEL_CHAR_SIZE / 2];
};

union Temperature {
    uint8_t bytes[TEMP_BUFF_SIZE];
    uint16_t cmds[TEMP_BUFF_SIZE / 2];
};

/**
 * @brief Convert number to the correct signed range. This is very specific
 * to this problem and potentially dangerous for type narrowing
 * @param typename T: the old unsigned type
 * @return typename UT: the new type with sign
 **/
template <typename UT, typename T>
UT toSigned(T val)
{
    T mid = (std::numeric_limits<T>::max() - 1) / 2 + 1;
    UT v = (val >= mid) ? (UT)(val - mid) : (UT)val - (UT)mid;
    return v;
}

/**
 * @brief Convert number to the correct unsigned range. This is very specific
 * to this problem and potentially dangerous for type narrowing
 * @param typename T: the old signed type
 * @return typename UT: the new type without sign
 **/
template <typename UT, typename T>
UT toUnsigned(const T val)
{
    UT mid = (std::numeric_limits<UT>::max() - 1) / 2 + 1;
    return val + mid;
}

/**
 * @brief Step the left motor
 **/
void leftMotorTimerHandler()
{
    if (l_current_period >= 0)
        digitalWrite(LEFT_DIR, LOW);
    else
        digitalWrite(LEFT_DIR, HIGH);
}

/**
 * @brief Step the right motor
 **/
void rightMotorTimerHandler()
{
    if (r_current_period >= 0)
        digitalWrite(RIGHT_DIR, HIGH);
    else
        digitalWrite(RIGHT_DIR, LOW);
}

// Function to reset the board if necessary
void (*resetFunc)() = 0; // declare reset function @ address 0

// force inline functions to make up for speed
inline float round(float, const int) __attribute__((always_inline));

/**
 * @brief Round a floating point number to precision
 * @param float: value to be rounded
 * @return float: rounded value
 **/
float roundToClosestHalf(float value)
{
    return roundf(value * 2) / 2.;
}

/**
 * @brief Compute period for the stepper motor
 * @param float: linear speed in cm/s
 * @return uint32_t: delay in us
 **/
uint32_t computePeriod(float speed)
{
    if (abs(speed) < SPEED_MIN_CM_BY_S) {
        return 0;
    }
    else {
        return (uint32_t)roundToClosestHalf(65450.0 / speed);
    }
}

/**
 * @brief Period's controller that is responsible for managing the speeds by limiting
 * to the max acceleration
 * @param void*: Task parameters (not used)
 **/
static void periodControl(void*)
{
    // Current speeds
    float d_speed;

    bool l_is_stopped = true;
    bool l_is_disabled = true;
    bool r_is_stopped = true;
    bool r_is_disabled = true;

    // Local copies of motor speed in cm/s
    float l_local_target_speed = 0;
    float r_local_target_speed = 0;

    TickType_t Time;

    while (true) {
        // Take current time.
        Time = xTaskGetTickCount();

        // left motor
        if (!l_must_be_reset) {
            l_local_target_speed = l_target_speed;
            d_speed = l_local_target_speed - l_current_speed;

            if (d_speed != 0.0) {
                if (d_speed > MAX_DV_IN_CM_BY_CYCLE) {
                    l_current_speed += MAX_DV_IN_CM_BY_CYCLE;
                }
                else if (d_speed < -MAX_DV_IN_CM_BY_CYCLE) {
                    l_current_speed -= MAX_DV_IN_CM_BY_CYCLE;
                }
                else {
                    l_current_speed = l_local_target_speed;
                }

                if (l_current_speed != 0) {
                    l_current_period = computePeriod(l_current_speed);
                    left_motor_timer.setPeriod(abs(l_current_period));
                    if (l_is_stopped) {
                        digitalWrite(LEFT_ENABLE, HIGH);
                        l_is_disabled = false;
                        left_motor_timer.startTimer();
                        l_is_stopped = false;
                    }
                }
                else {
                    if (!l_is_stopped) {
                        left_motor_timer.stopTimer();
                        l_is_stopped = true;
                        // if (!l_is_disabled) {
                        //     digitalWrite(LEFT_ENABLE, LOW);
                        //     l_is_disabled = true;
                        // }
                    }
                }
            }
        }
        else {
            if (!l_is_stopped || l_must_be_reset) {
                left_motor_timer.stopTimer();
                l_current_speed = 0;
                l_is_stopped = true;
                if (!l_is_disabled || l_must_be_reset) {
                    digitalWrite(LEFT_ENABLE, LOW);
                    l_is_disabled = true;
                }
            }
            if (l_must_be_reset)
                l_must_be_reset = false;
        }

        // right motor
        if (!r_must_be_reset) {
            r_local_target_speed = r_target_speed;
            d_speed = r_local_target_speed - r_current_speed;

            if (d_speed != 0.0) {
                if (d_speed > MAX_DV_IN_CM_BY_CYCLE) {
                    r_current_speed += MAX_DV_IN_CM_BY_CYCLE;
                }
                else if (d_speed < -MAX_DV_IN_CM_BY_CYCLE) {
                    r_current_speed -= MAX_DV_IN_CM_BY_CYCLE;
                }
                else {
                    r_current_speed = r_local_target_speed;
                }

                if (r_current_speed != 0) {
                    r_current_period = computePeriod(r_current_speed);
                    right_motor_timer.setPeriod(abs(r_current_period));
                    if (r_is_stopped) {
                        digitalWrite(RIGHT_ENABLE, HIGH);
                        r_is_disabled = false;
                        right_motor_timer.startTimer();
                        r_is_stopped = false;
                    }
                }
                else {
                    if (!r_is_stopped) {
                        right_motor_timer.stopTimer();
                        r_is_stopped = true;
                        // if (!r_is_disabled) {
                        //     digitalWrite(RIGHT_ENABLE, LOW);
                        //     r_is_disabled = true;
                        // }
                    }
                }
            }
        }
        else {
            if (!r_is_stopped || r_must_be_reset) {
                right_motor_timer.stopTimer();
                r_current_speed = 0;
                r_is_stopped = true;
                if (!r_is_disabled || r_must_be_reset) {
                    digitalWrite(RIGHT_ENABLE, LOW);
                    r_is_disabled = true;
                }
            }
            if (r_must_be_reset)
                r_must_be_reset = false;
        }

        vTaskDelayUntil(&Time, SPEED_CONTROL_PERIOD_MS);
    }
}

void stop_motors(BLEDevice central)
{
    l_target_speed = 0.;
    r_target_speed = 0.;
    l_must_be_reset = true;
    r_must_be_reset = true;
}

void set_velocities(BLEDevice central, BLECharacteristic characteristic)
{
    ++total_msg_count;

    MotorCmd msg;
    memcpy(msg.bytes, characteristic.value(), sizeof(msg.bytes));
    float vl = toSigned<float>(msg.cmds[1]) / 10.;
    float vr = toSigned<float>(msg.cmds[2]) / 10.;
    float accel = msg.cmds[3] / 100.;
    if (accel > MIN_ACCEL_TO_CONSIDER) {
        max_accel = accel;
        max_accel = fmin(accel, MAX_ACCEL_TO_CONSIDER);
    }

    // LOG("Current velocities:", vl, vr);
    // LOG("Current acceleration:", accel);

    // make sure the velocities stay within a good range
    vl = min(vl, 100.);
    vr = min(vr, 100.);

    l_target_speed = vl;
    r_target_speed = vr;

    // check for mismatch of ids which indicates a dropped message
    if (internal_id != msg.cmds[0]) {
        LOG("Have id", internal_id, "but received", msg.cmds[0]);
        dropped_msg_count++; // This is naively assuming consecutive messages are not lost/mixed in reception.
    }
    internal_id = (msg.cmds[0] + 1) % 101;
    heartbeat = NUM_LOST_HEARTBEATS;

    if (return_dropped_msgs_char.value()) { // dropped msgs write
        DroppedMsg msg;
        msg.counters[0] = dropped_msg_count;
        msg.counters[1] = total_msg_count;
        dropped_msgs_count_char.writeValue(msg.bytes, sizeof(msg.bytes), false);
    }

    if (return_current_vel_char.value()) { // current velocities
        MotorSpeeds msg;
        msg.cmds[0] = toUnsigned<uint16_t>(l_current_speed * 10.);
        msg.cmds[1] = toUnsigned<uint16_t>(r_current_speed * 10.);
        current_vel_char.writeValue(msg.bytes, sizeof(msg.bytes), false);
        // LOG("Current velocities:", l_current_speed, r_current_speed);
    }
#ifdef ENABLE_TEMPERATURE_SENSORS
    if (return_current_temp_char.value()) { // current velocities
        Temperature msg;
        float temp = tmp117.readTempC();
        msg.cmds[0] = toUnsigned<uint16_t>(temp * 10.);
        temperature_char.writeValue(msg.bytes, sizeof(msg.bytes), false);
    }
#endif
}

void set_duty_cycle(BLEDevice central, BLECharacteristic characteristic)
{
    // make sure duty cycle variable does not go out of range
    uint8_t duty = min(*characteristic.value(), 100);
    duty = max(duty, 0);
    lowCurrentTimer.setDutyCycle(duty);
    LOG("Setting duty cycle to:", duty);
}

void set_max_accel(BLEDevice central, BLECharacteristic characteristic)
{
    max_accel = fmin(*characteristic.value() / 100., MAX_ACCEL_TO_CONSIDER); // this is in cm with 2 decimal points when sent this is why / 100
    LOG("Max accel updated to:", max_accel);
}

/**
 * @brief This task is acting as a server waiting to receive motor commands which converts
 * to rad/s for the steppers. The stepper motion is dictated by 2 separate tasks (not this).
 * @param void*: Task parameters (not used)
 **/
void motorCmdServer(void*)
{
    TickType_t Time;
    // LOG("Motor command server task: Running", __LINE__);

    BLE.setEventHandler(BLEDisconnected, stop_motors);

    desired_vel_char.setEventHandler(BLEWritten, set_velocities);
    set_duty_cycle_char.setEventHandler(BLEWritten, set_duty_cycle);
    max_accel_char.setEventHandler(BLEWritten, set_max_accel);

    while (true) { // central loop

        // Take current time.
        Time = xTaskGetTickCount();

        BLE.poll(3);

        --heartbeat; // no velocities received
        if (heartbeat == 0) {
            l_target_speed = 0.;
            r_target_speed = 0.;
            l_must_be_reset = true;
            r_must_be_reset = true;
            heartbeat = NUM_LOST_HEARTBEATS;
            heartbeat_char.writeValue((byte)0x01);

            // LOG("No heartbeat", internal_id);
        }

        vTaskDelayUntil(&Time, CMD_SLEEP_MS);
    }
}

/**
 * @brief Initialize the board and wifi module
 **/
void setup()
{
    // Orange led upon connection
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // Initialize Motors GPIOs
    left_step.pinAssign(LEFT_STEP);
    right_step.pinAssign(RIGHT_STEP);
    digitalWrite(LEFT_DIR, LOW);
    pinMode(LEFT_DIR, OUTPUT);
    digitalWrite(RIGHT_DIR, LOW);
    pinMode(RIGHT_DIR, OUTPUT);
    digitalWrite(LEFT_ENABLE, LOW);
    pinMode(LEFT_ENABLE, OUTPUT);
    digitalWrite(RIGHT_ENABLE, LOW);
    pinMode(RIGHT_ENABLE, OUTPUT);

    digitalWrite(LEFT_HC, HIGH);
    pinMode(LEFT_HC, OUTPUT);
    right_HC.pinAssign(RIGHT_HC);
    digitalWrite(LEFT_DECAY, HIGH);
    pinMode(LEFT_DECAY, OUTPUT);
    digitalWrite(RIGHT_DECAY, HIGH);
    pinMode(RIGHT_DECAY, OUTPUT);

    // pinMode(UNDERVOLTAGE, INPUT);

    pinMode(IR_0, INPUT);
    pinMode(IR_1, INPUT);
    pinMode(IR_2, INPUT);

    digitalWrite(PULSE_0, LOW);
    pinMode(PULSE_0, OUTPUT);
    digitalWrite(PULSE_1, LOW);
    pinMode(PULSE_1, OUTPUT);
    digitalWrite(PULSE_2, LOW);
    pinMode(PULSE_2, OUTPUT);

    // digitalWrite(GPIO_A6, LOW);
    // pinMode(GPIO_A6, OUTPUT);
    // digitalWrite(GPIO_A7, LOW);
    // pinMode(GPIO_A7, OUTPUT);

    pinMode(RIGHT_DECAY, OUTPUT);

// Wait for the serial port to be initialized
#ifdef ENABLE_LOGGING
    while (!Serial) {
        Serial.begin(115200);
        delay(1500);
        // digitalWrite(GPIO_A7, HIGH);
        // digitalWrite(GPIO_A7, LOW);
    }
#endif

#ifdef ENABLE_TEMPERATURE_SENSORS
    Wire.begin();
    Wire.setClock(400000); // Set clock speed to be the fastest for better communication (fast mode)

    // Initialize temperature sensors
    if (tmp117.begin(TMP117_ADDR)) {
        LOG("TMP117 sensor found!");
    }
    else {
        LOG("Failed to find Right TMP117 sensor");
    }
#endif

    // BLE initialization
    while (!BLE.begin()) {
        delay(1500);
        // digitalWrite(GPIO_A6, HIGH);
        // digitalWrite(GPIO_A6, LOW);
    }
    BLE.setLocalName(LUREBOT_BLE_NAME);
    BLE.setDeviceName(LUREBOT_BLE_NAME);
    LOG("BLE started with address", BLE.address());

    motor_vel_srv.addCharacteristic(desired_vel_char);
    motor_vel_srv.addCharacteristic(current_vel_char);
    motor_vel_srv.addCharacteristic(return_current_vel_char);
    BLE.addService(motor_vel_srv);

    accel_srv.addCharacteristic(max_accel_char);
    BLE.addService(accel_srv);

    duty_cycle_srv.addCharacteristic(set_duty_cycle_char);
    BLE.addService(duty_cycle_srv);

    // ir_srv.addCharacteristic(set_ir_char);
    // ir_srv.addCharacteristic(ir_val_char);
    // BLE.addService(ir_srv);

    dropped_msgs_srv.addCharacteristic(dropped_msgs_count_char);
    dropped_msgs_srv.addCharacteristic(return_dropped_msgs_char);
    BLE.addService(dropped_msgs_srv);

    info_srv.addCharacteristic(device_name_char);
    info_srv.addCharacteristic(fw_version_char);
    BLE.addService(info_srv);

    heartbeat_srv.addCharacteristic(heartbeat_char);
    BLE.addService(heartbeat_srv);

    temperature_srv.addCharacteristic(temperature_char);
    temperature_srv.addCharacteristic(return_current_temp_char);
    BLE.addService(temperature_srv);

    device_name_char.writeValue(LUREBOT_BLE_NAME);
    fw_version_char.writeValue(LUREBOT_FIRMWARE_VERSION);
    max_accel_char.writeValue((byte)0xaf);
    set_duty_cycle_char.writeValue((byte)0x21);
    return_dropped_msgs_char.writeValue((byte)0x00);
    dropped_msgs_count_char.writeValue((byte)0x00);
    desired_vel_char.writeValue((byte)0x0000);
    return_current_vel_char.writeValue((byte)0x00);
    // set_ir_char.writeValue((byte)0x00);

    // BLE.setAdvertisingInterval(160); // 160 * 0.625 ms = 100 ms (this is the default)
    BLE.setSupervisionTimeout(3200);
    // BLE.setTimeout();
    BLE.setConnectionInterval(0x0000, 0x000f); // intervals of 1.25
    BLE.advertise();

    // RTOS tasks
    vSetErrorLed(ERROR_LED_PIN, ERROR_LED_LIGHTUP_STATE);

    left_motor_timer.Init(TIMER_TCC0, leftMotorTimerHandler);
    // LOG("Left motor timer: Started A");

    right_motor_timer.Init(TIMER_TCC1, rightMotorTimerHandler);
    // LOG("Right motor timer: Started B");

    lowCurrentTimer.Init(TIMER_TCC2, NULL);
    lowCurrentTimer.setFrequency(10625);
    lowCurrentTimer.setDutyCycle(33);
    lowCurrentTimer.startTimer();
    // LOG("Current modulation: Started C");

    if (xTaskCreate(periodControl, "tsk_periodControl", 1024, NULL, tskIDLE_PRIORITY + 1, NULL) == pdPASS) {
        // LOG("Period control task: Started D");
    }
    else {
        // LOG("Period control task: Failed to start");
        // LOG("Reset the robot !!");
        resetFunc();
    }

    if (xTaskCreate(motorCmdServer, "tsk_motorCmdSrv", 1024, NULL, tskIDLE_PRIORITY + 2, NULL) == pdPASS) {
        // LOG("Motor command server task: Started E");
    }
    else {
        // LOG("Motor command server task: Failed to start");
        // LOG("Reset the robot !!");
        resetFunc();
    }

    vTaskStartScheduler();

    // LOG("Scheduler terminated");
    // LOG("Reset the robot !!");
    delay(10000);
    resetFunc();
}

/**
 * @brief Main process - for RTOS this should remain empty
 **/
void loop() {}