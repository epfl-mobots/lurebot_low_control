[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.7802053.svg)](https://doi.org/10.5281/zenodo.7802053)

# Instructions

This project uses the Arduino Nano 33 IoT and depends on:

- FreeRTOS_SAMD21 (2.3.0)
- ArduinoBLE (1.2.2)

This is the firmware for the LureBot. It implements:

1. The low level motor control and power management
2. A Bluetooth Low Energy (BLE) interface to accept commands and communicate information to clients
## Customizing parameters

Multiple parameters concerning the functionality of the robot and/or other parameters not related to the motion of the robot are located in the file `parameters.h`.

## UUID definitions (BLE) 

In the header file `uuid_defs.h` you may find all the definitions of the services and characteristics that are used. In short, there exist the following services and characteristics:

- **General information service** (`2a00970d-6592-4c13-bfc0-6eab4c021967`)
    - **Device name** (`6a7d1ab3-8e30-44a3-ab60-adf2247233ff`)
      - **Properties:** `Read`
      - **Type:** `string`
    - **Firmware version flashed** (`f2fac9b2-6646-402a-a387-8aace42aec50`)
      - **Properties:** `Read`
      - **Type:** `string`
- **Motor velocity service** (`577be3a5-d33d-4961-a2d0-eb0e48d6f0c8`)
  - **Target velocity** (`b8c18dc6-1b80-4833-bad3-7f3411cdb25f`)
      - **Properties:** `Read | WriteWithNoResponse`
      - **Type:** `uint8_t[6]`
        - | message id (2 bytes) | left motor (2 bytes) | right motor (2 bytes) |
      - **Comments:** Velocities are sent in an unsigned range 0-65536. Half of the values represent negative numbers the non-negative. It is expected that the received value is mm/s with 1 decimal point of precision (i.e., 30.2 cm/s ~> 302 mm/s).
  - **Current velocity** (`49a6d7a3-658a-4a3c-81e7-7a3360ecdaaf`)
    - **Properties:** `Read | Notify`
    - **Type:** `uint8_t[4]`
      - | left motor (2 bytes) | right motor (2 bytes) |
  - **Return current velocity** (`b8c18dc6-1b80-4833-bad3-7f3411cdb25f`)
    - **Properties:** `Read | WriteWithNoResponse`
    - **Type:** `uint8_t`
      - | True/False (1 byte) |
- **Max acceleration service** (`84f74144-6d58-4e96-98b0-c9eab0bcdc55`)
  - **Max acceleration** (`6e9942df-a2ec-477a-96da-79ec9b78ba9f`)
    - **Properties:** `Read | WriteWithNoResponse`
    - **Type:** `uint8_t[2]`
      - | max acceleration (2 bytes) |
    - **Comments:** Accelerations are to be sent in mm/s^2 (i.e., 1.75 m/s^2 ~> 175 mm/s^2).
- **Duty cycle service** (`f19b954f-ff61-423c-bf6a-587a791b14a7`)
  - **Duty cycle percentage** (`34a29f06-ef2f-433d-8749-3122b3259a82`)
    - **Properties:** `Read | WriteWithNoResponse`
    - **Type:** `uint8_t`
      - | duty cycle percentage (1 byte) |
    - **Comments:** Percentage values in range [0, 100].
- **IR service** (`not offered at the moment`)
- **Dropped messages service** (`fbbfb9b9-ce47-4ce8-9156-f1139560022e`)
  - **Return dropped messages** (`312abc15-73cf-4a55-9f8f-f822a66f2bf7`)
    - **Properties:** `Read | WriteWithNoResponse`
    - **Type:** `uint8_t`
      - | True/False (1 byte) |
  - **Dropped message counters** (`e354a4a8-3202-401d-9bb6-2f918c257338`)
    - **Properties:** `Read | WriteWithNoResponse`
    - **Type:** `uint8_t[16]`
      - | number of dropped packets (8 bytes) | number of total messages received (8 bytes) |
