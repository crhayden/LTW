/*************************************************************
 * Copyright (c) 2019 Apple Enterprises. This code is
 * proprietary and a trade secret of Apple Enterprises.
 *
 * $Workfile: Client.c $
 *
 * $Creator: Chris Apple $
 *
 * $Description: Handles high level USB HID communication (HID Reports). $
 *
 * $NoKeywords: $
 *************************************************************/

#include "stdint.h"
#include "kisPackets.h"
#include "Client.h"
#include "gpio.h"
#include "GPIOHandler.h"
#include "kisStepper.h"
#include "tim.h"
#include "version.h"
#include "Analog.h"
#include "resetHandler.h"
#include "loop.h"
#include "serialNumber.h"
#include <string.h>

// variables
uint16_t brightness_LD1 = 5500;//default variable for duty% setting, LD1
uint16_t brightness_LD2 = 5500;//default variable for duty% setting, LD2
bool shouldAllowReportingFromDevice = false;
bool isFirstReadFromHost = true;

/**
 * @brief Return a bitmap of merged status flags
 *
 * @return uint8_t Merged bitmap.
 */
uint8_t ClIENT_GetStatusFlags()
{
    uint8_t flags = 0;
    if (yStepper.isHomed())
    {
        flags += HOMED_Y_STEPPER;
    }

    if (yStepper.isEnabled())
    {
        flags += ENABLED_Y_STEPPER;
    }

    if (flipStepper.isHomed())
    {
        flags += HOMED_FLIP_STEPPER;
    }

    if (flipStepper.isEnabled())
    {
        flags += ENABLED_FLIP_STEPPER;
    }

    if (zStepper.isHomed())
    {
        flags += HOMED_Z_STEPPER;
    }

    if (zStepper.isEnabled())
    {
        flags += ENABLED_Z_STEPPER;
    }
    return (flags);
}

/**
 * @brief Construct status packet to send to usb host.
 *
 * @param responsePacketData Pointer to usb buffer to fill with packet data.
 */
void CLIENT_GetStatusReport(uint8_t *responsePacketData)
{
    uint8_t moving = (zStepper.inMotion() || flipStepper.inMotion() || yStepper.inMotion());
    PACKET_HEADER *head = (PACKET_HEADER *)responsePacketData;
    PACKET_STATUS *status = (PACKET_STATUS *)(responsePacketData + sizeof(PACKET_HEADER));
    head->null_byte = 1;                                          // windows needs this to be a 1 byte.
    head->length = sizeof(PACKET_HEADER) + sizeof(PACKET_STATUS); // not currently used, struct unpacking handles this
    head->message = STATUS;                                       // Message type
    head->timestamp = HAL_GetTick();                              // Milliseconds since boot
    status->gpioOutState = getGPIOOutputState();                  // Current output mask
    status->gpioInState = getGPIOInputState();                    // Current input mask TODO
    status->yStep = yStepper.getStep();               // Tipstop step position
    status->flipStep = flipStepper.getStep();                   // Laser step position
    status->zStep = zStepper.getStep();                     // Door step position
    status->inMotion = moving;                                    // Motion state of the motors
    status->mcuTemp = gAnalog.GetTemperature();                   // MCU junction temperature
    status->vbus = gAnalog.GetMillivoltsVbus();                   // Voltage on rail
    status->vref = gAnalog.GetMillivoltsVdd();                    // Internal reference voltage
    status->stateFlags = ClIENT_GetStatusFlags();                 // Bitwise status flags
    status->errorFlags = 0;                                       // Bitwise error flags TODO implement
    status->versionMinor = VERSION_MINOR;                         // Minor version of FW
    status->versionMajor = VERSION_MAJOR;                         // Major version of FW
    status->serialNumber = getSerialNumber();
    status->ld1_duty = brightness_LD1;							  // Usable range of 4500 - 8100....< 3000 is off, 8100 max
    status->ld2_duty = brightness_LD2;							  // Usable range of 4500 - 8100....< 3000 is off, 8100 max
}

/**
 * @brief Route an input command to the appropriate subsytem
 *
 * @param commandOpcode Type of command being handled. Matched with packet type
 * @param commandParameters Pointer to buffer just after message header.
 */
void CLIENT_ProcessCommand(uint32_t commandOpcode, const uint8_t *commandParameters)
{
    // If the power is failed, don't process any commands
    if (!isPowerFailed())
    {
        switch (commandOpcode)
        {
        case RUN_MOTOR:
        {
            PACKET_MOTOR *packet = (PACKET_MOTOR *)commandParameters;
            StepperMotor *motor = NULL;
            
            // Map motor to motor object
            switch (packet->motorid)
            {
            case FLIP_MOTOR:
                motor = &flipStepper;
                break;
            case Z_MOTOR:
                motor = &zStepper;
                break;
            case Y_MOTOR:
                motor = &yStepper;
                break;
            }

            switch (packet->motorCommand)
            {
            case RUN_TO_POSITION:
                motor->runToPosition(packet->motorPosition, packet->motorSpeed);
                break;

            case RUN_TO_HOME:
                motor->runToHome();
                break;

            case RUN_TO_LIMIT:
                motor->runToLimit();
                break;

            case ENABLE_MOTOR:
                motor->setEnable(true);
                break;

            case DISABLE_MOTOR:
                motor->setEnable(false);
                break;

            case ABORT:
                motor->abort();
                break;

            case SET_CURRENT:
                motor->setCurrent(packet->motorRunCurrent, packet->motorHoldCurrent);
                break;

            case SET_SERVO_TORQUE:
                motor->setServoTorque(packet->servoTorque);
                break;
            }
        }
        break;

        case RUN_GPIO:
        {
            if(!shouldAllowReportingFromDevice) {
            	shouldAllowReportingFromDevice = true;
            }
            PACKET_GPIO *packet = (PACKET_GPIO *)commandParameters;
            setGPIOState(packet->gpioMask);
        }
        break;

        case GENERAL_COMMAND:
        {
            PACKET_CMD *packet = (PACKET_CMD *)commandParameters;
            switch (packet->cmd)
            {
            case RESET_MCU:
                // TODO implement sw reset (trigger watchdog?)
                break;
            
            case JUMP_TO_BOOTLOADER:
                assertBootloaderJump();
                break;
            
            case SET_SERIAL_NUMBER:
                setSerialNumber(packet->optionalArgument);
                break;

            case RESET_MODEM:
                modemResetHandler.reset();
                break;

            case DUTY_LD1:
				brightness_LD1 = packet->optionalArgument;
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,brightness_LD1);
				HAL_Delay(10);
				break;

			case DUTY_LD2:
				brightness_LD2 = packet->optionalArgument;
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,brightness_LD2);
				HAL_Delay(10);
				break;

            default:
                break;
            }
        }
        break;

        default:
            break;
        }
    }
}
