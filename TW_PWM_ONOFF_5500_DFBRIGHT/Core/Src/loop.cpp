#include "loop.h"
#include "main.h"
#include "motor.h"
#include "tim.h"
#include "kisStepper.h"
#include "Analog.h"
#include "resetHandler.h"
#include "GPIOHandler.h"
#include "serialNumber.h"

#define VBUS_DROP_THRESHOLD 2300
#define VBUS_RISE_THRESHOLD 2350

static bool bootloadJump = false;
static bool powerFailure = true;

void setup()
{
//    usbResetHandler.initializeCold(VUSB_RESET__Pin, VUSB_RESET__GPIO_Port, 1500);
//    usbResetHandler.reset();
    modemResetHandler.initializeCold(MODEM_RESET__Pin, MODEM_RESET__GPIO_Port, 3000);

    configSteppers();
    flipStepper.initializeCold(
        FAXIS_EN__Pin,
        FAXIS_EN__GPIO_Port,
        FAXIS_STEP_Pin,
        FAXIS_STEP_GPIO_Port,
        FAXIS_DIR_Pin,
        FAXIS_DIR_GPIO_Port,
        TAG_UP__Pin,
        TAG_DOWN__GPIO_Port,
        -1,
        NULL,
        5000,
        3200,
        NULL,
        0,
        0,
        FAXIS_CS__Pin,
        FAXIS_CS__GPIO_Port);
    flipStepper.setCurrent(0x10, 0x03);
    HAL_Delay(10);
    yStepper.initializeCold(
        YAXIS_EN__Pin,
        YAXIS_EN__GPIO_Port,
        YAXIS_STEP_Pin,
        YAXIS_STEP_GPIO_Port,
        YAXIS_DIR_Pin,
        YAXIS_DIR_GPIO_Port,
        Y_BACK__Pin,
        Y_BACK__GPIO_Port,
        -1,
        NULL,
        5000,
        150,
        NULL,
        0,
        0,
        YAXIS_CS__Pin,
        YAXIS_CS__GPIO_Port);
    yStepper.setCurrent(0x10, 0x03);
    HAL_Delay(10);
    zStepper.initializeCold(
        ZAXIS_EN__Pin,
        ZAXIS_EN__GPIO_Port,
        ZAXIS_STEP_Pin,
        ZAXIS_STEP_GPIO_Port,
        ZAXIS_DIR_Pin,
        ZAXIS_DIR_GPIO_Port,
        HOME__Pin,
        HOME__GPIO_Port,
        -1,
        NULL,
        5000,
        500,
        NULL,
        0,
        0,
        ZAXIS_CS__Pin,
        ZAXIS_CS__GPIO_Port);
    zStepper.setCurrent(0x10, 0x03);
    zStepper.invertDirection();
    HAL_Delay(10);


    gAnalog.InitializeCold();

    // Start the motor timer
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        /* Starting Error */
        Error_Handler();
    }
}

void assertBootloaderJump()
{
    bootloadJump = true;
}

bool isPowerFailed()
{
    return powerFailure;
}

void checkPowerFailure()
{
    uint32_t voltage = gAnalog.GetMillivoltsVbus();
    if (voltage > VBUS_RISE_THRESHOLD && powerFailure)
    {
        // clear power failure on boot
        powerFailure = false;
    }
    else if (voltage < VBUS_DROP_THRESHOLD && !powerFailure)
    {
        // Set the semaphor
        powerFailure = true;
    }
}

bool loop()
{
    gAnalog.HandleIdleLoop();
    runResetHandlers();
    checkPowerFailure();
    return !bootloadJump;
}

void runMotors()
{
    runStepperMotors();
}
