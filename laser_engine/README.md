## Overview

This firmware is designed for a laser engine system, controlling and monitoring various aspects of laser operation, including temperature, voltage, current, and laser status. It interfaces with TCM (Thermo-Electric Cooler Module) controllers and a host device (e.g., PC) to manage the laser's performance and safety.

## Hardware Requirements

    Teensy 4.1 board
    RGB LED (connected to pins 36, 37, 38)
    Laser drivers (connected to pins 16-19, 31-33)
    Despeckler (connected to pin 33)
    Interlock key (connected to pin 9)
    TCM modules (connected via UART5)

## Software Dependencies

    FastLED library
    elapsedMillis library
    CRC32 library

## Functions Explanation
    getAdjustTemperature()

    This function is responsible for retrieving the adjust temperature values from the TCM (Thermo-Electric Cooler Module) controllers. These values are used to set the initial temperature setpoints for each channel.

    enableAllTCMs()

    This function is responsible for enabling all TCM modules. This ensures that the temperature control modules are active and ready to control the laser temperatures.

## Status Machine 

    The state transitions are based on the following conditions:

    WARMING_UP:
        If the temperature difference (tempDiff) is within 0.5 degrees of the setpoint, transition to CHECK_ACTIVE.
        If the temperature difference is greater than the error threshold (TEMP_ERROR_THRESHOLD), transition to CHECK_ERROR.

    CHECK_ACTIVE:
        If the temperature difference is greater than 0.5 degrees, transition back to WARMING_UP.
        If the temperature difference is within 0.5 degrees and the channel has been in this state for at least ACTIVE_DURATION milliseconds, transition to ACTIVE.

    ACTIVE:
        If the temperature difference is greater than the error threshold, transition to CHECK_ERROR.
        If the temperature difference is less than -0.5 degrees, transition to WARMING_UP.

    CHECK_ERROR:
        If the temperature difference is less than the error threshold, transition to ACTIVE if it is within 0.5 degrees, or to WARMING_UP if it is not.
        If the channel has been in this state for at least ERROR_DURATION milliseconds, transition to ERROR.

    ERROR:
        If the temperature difference is within 0.5 degrees of the setpoint, transition to CHECK_ACTIVE.

    SLEEP:
        If the laser status is 1 (active) and the cooldown time has passed, transition to WAKE_UP.

    WAKE_UP:
        Wait for the TCM module to enable the channel, then transition to WARMING_UP.

    PREPARE_SLEEP:
        Wait for the TCM module to disable the channel, then transition to SLEEP.

## License

This firmware is released under the MIT License.
Contact

For any questions or issues, please contact Kevin.Wang.
