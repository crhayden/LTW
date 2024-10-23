#ifndef __KIS_PACKETS_H__
#define __KIS_PACKETS_H__

#include <stdint.h>

#define PACKET_SIZE 64

#pragma pack(push,1)        // force single-byte packing of structure fields

// Set the message description to for the command handling
enum MSG_ID {
    STATUS = 1,
    RUN_MOTOR = 2,
    RUN_GPIO = 3,
    GENERAL_COMMAND = 4,
};

// Motor to be commanding the machine
enum MOTOR_ID {
    FLIP_MOTOR = 0,
    Y_MOTOR = 1,
    Z_MOTOR = 2,
};

// Command to request a motor action
enum MOTOR_COMMAND {
    RUN_TO_HOME = 0,
    RUN_TO_LIMIT = 1,
    RUN_TO_POSITION = 2,
    ABORT = 3,
    ENABLE_MOTOR = 4,
    DISABLE_MOTOR = 5,
    SET_CURRENT = 6,
    SET_SERVO_TORQUE = 7,
};

// Flags for the status
enum STATUS_FLAGS {
    HOMED_Y_STEPPER = 2,
    HOMED_FLIP_STEPPER = 4,
    HOMED_Z_STEPPER = 8,
    ENABLED_Y_STEPPER = 16,
    ENABLED_FLIP_STEPPER = 32,
    ENABLED_Z_STEPPER = 64,
};

// Flags for system errors
enum ERROR_FLAGS {
    STEPPERS_DRIVER_CONFIG_ERROR = 1,
    POWER_FAILURE = 2,
};

// Flags for gpio for the LEDs
enum OUTPUT_FLAGS {
    FE_RUN = 1,
    EM_STOP = 2,
    LD_1 = 4,
    LD_2 = 8,
    SOLENOID = 16,
};

enum INPUT_FLAGS {
    YDIAG0 = 1,
    YDIAG1 = 2,
    Y_FRONT = 4,
    Y_BACK = 8,
    FDIAG0 = 16,
    FDIAG1 = 32,
    OPTO_LT = 64,
    OPTO_RT = 128,
    INTER_LS = 256,
    HOME = 512,
    TAG_DOWN = 1024,
    LIMIT = 2048,
    TAG_UP= 4096,
    FE_WRN = 8192,
    FE_STAT = 16384,
};

// Hardware command for auxillary functins.
enum CMD {
    RESET_MCU = 0,
    RESET_STEPPERS = 2,
    RESET_USB = 10,
    JUMP_TO_BOOTLOADER = 11,
    SET_SERIAL_NUMBER = 12,
	RESET_MODEM = 13,
	DUTY_LD1 = 14,
	DUTY_LD2 = 15,
};

// Header packet attached to every message
typedef struct PACKET_HEADER {
    uint8_t null_byte;
    uint8_t message;
    uint8_t length;
    uint32_t timestamp;
} PACKET_HEADER;

// Packet describing a motor command
typedef struct PACKET_MOTOR {
    uint8_t motorid;
    uint8_t motorCommand;
    int32_t motorPosition;
    int32_t motorSpeed;
    uint8_t motorRunCurrent;
    uint8_t motorHoldCurrent;
    uint8_t servoTorque;
} PACKET_MOTOR;

// Packet describing the on-off state of GPIO ports
typedef struct PACKET_GPIO {
    uint16_t gpioMask;
} PACKET_GPIO;

// Packet starting an animation state
typedef struct PACKET_ANIMATION {
    uint8_t animationCmd;
    uint8_t repeat;
    uint8_t framerate;
} PACKET_ANIMATION;

// Packet outlining a hardware command
typedef struct PACKET_CMD {
    uint8_t cmd;
    uint32_t optionalArgument;
} PACKET_CMD;


// Packet describing the full state of the kis
typedef struct PACKET_STATUS {
    uint16_t gpioOutState;              // Bit mask of gpio outputs
    uint16_t gpioInState;               // Bit mask of gpio inputs
    int32_t yStep;               // Nominal step location of the tip stop
    int32_t tipStopEncoder;            // Actual location of the tip stop
    int32_t flipStep;                 // Nominal step location of the laser motor
    int32_t zStep;                  // Nominal step location of the door
    int32_t tzStep;                 // Nominal step location of the transponder door
    int32_t dispenserStep;             // Nominal step location of the dispenser
    uint8_t inMotion;                   // Is the system moving
    uint16_t temperature;               // Ambient temperature
    uint16_t mcuTemp;                   // MCU junction temperature
    uint16_t clampMotorCurrent;         // Current of the clamp motor
    uint16_t vbus;                      // Voltage on the main bus
    uint16_t vref;                      // Reference voltage inside the chip
    uint16_t stateFlags;                // Flags for current state of the system
    uint8_t errorFlags;                 // Flags for current errors in the system
    uint8_t versionMinor;				// Numerical version (minor)
    uint8_t versionMajor;               // Numerical version (major)
    uint32_t serialNumber;              // Serial number as unsigned int
    //uint32_t unused1;
    //uint32_t unused2;
    uint32_t ld1_duty;					//variable for duty% setting, LD1 --> Usable range of 4500 - 8100....< 3000 is off, 8100 max
    uint32_t ld2_duty;					//variable for duty% setting, LD2 --> Usable range of 4500 - 8100....< 3000 is off, 8100 max
} PACKET_STATUS;

#pragma pack(pop)

#endif
