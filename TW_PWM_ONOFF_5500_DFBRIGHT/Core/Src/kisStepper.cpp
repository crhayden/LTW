#include "kisStepper.h"
#include <math.h>
#include "gpio.h"
#include "stm32f4xx.h"
#include "spi.h"

#define STEPPER_CONTROL_TIMER 40
#define DEBOUNCE_THRESH 64
#define HOME_OFFSET 1000

// Type so that squaring does not overflow
const int64_t SECOND_US = 1000000;

// timestamp
static uint32_t microseconds = 0;

// Statically allocate motor objects
StepperMotor flipStepper;
StepperMotor yStepper;
StepperMotor zStepper;

StepperMotor::StepperMotor()
{
    // Empty init, use initializeCold
    state = MOTOR_STATE_UNINIT;
}

void StepperMotor::initializeCold(int enablePin, GPIO_TypeDef *enablePort, int stepPin, GPIO_TypeDef *stepPort, int dirPin, GPIO_TypeDef *dirPort, int homePin, GPIO_TypeDef *homePort, int limitPin, GPIO_TypeDef *limitPort, int32_t acc, int32_t hSpd, TIM_HandleTypeDef *encTim, float encRatio, uint8_t microsteps, int spiPin, GPIO_TypeDef *spiPort, int32_t homeOffset)
{
    // initialize GPIO
    ePin = enablePin;
    ePort = enablePort;
    sPin = stepPin;
    sPort = stepPort;
    dPin = dirPin;
    dPort = dirPort;
    hPin = homePin;
    hPort = homePort;
    lPin = limitPin;
    lPort = limitPort;
    selectPin = spiPin;
    selectPort = spiPort;
    // This is the allowable acceleration for this motor
    accel = acc;
    homeSpeed = hSpd;
    encoder = encTim;
    eRatio = encRatio;
    uSteps = microsteps;

    sState = false;
    state = MOTOR_STATE_COLD;
    step = 0;
    homed = false;
    invertDir = false;
    limitPosition = 0;
    servoTorque = 100;
    hOffset = homeOffset;

    if (encoder)
    {
        // Start the encoder reading
        HAL_TIM_Encoder_Start(encoder, TIM_CHANNEL_ALL);
    }
    setEnable(false);
}

void StepperMotor::setAcceleration(int32_t acc)
{
    // This changes the ramp acceleration. Probably bad to change while moving
    accel = acc;
}

void StepperMotor::setCurrent(uint8_t irun, uint8_t ihold)
{
    // {ADDR register + write offset, EMPTY, T_POWER_DOWN (0-16), IRUN % = (IRUN+1)/32, IHOLD % = (IHOLD+1)/32}
    uint8_t irunIholdSpiFrame[5] = {0x90, 0x00, 0x08, irun, ihold};
    HAL_GPIO_WritePin(selectPort, selectPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi4, irunIholdSpiFrame, 5, 100);
    HAL_GPIO_WritePin(selectPort, selectPin, GPIO_PIN_SET);
}

void StepperMotor::setServoTorque(uint8_t percent)
{
    if (percent > 100)
    {
        percent = 100;
    }
    servoTorque = percent;
}

void StepperMotor::invertDirection()
{
    invertDir = true;
}

void StepperMotor::setDirection(bool dir)
{
    // set the direction and save that state
    dState = dir;
    if (!invertDir)
    {
        HAL_GPIO_WritePin(dPort, dPin, (GPIO_PinState)dir);
    }
    else
    {
        HAL_GPIO_WritePin(dPort, dPin, (GPIO_PinState)!dir);
    }
}

void StepperMotor::setEnable(bool enable)
{
    if (enable)
    {
        HAL_GPIO_WritePin(ePort, ePin, GPIO_PIN_RESET);
        enabled = true;
    }
    else
    {
        HAL_GPIO_WritePin(ePort, ePin, GPIO_PIN_SET);
        enabled = false;
        homed = false;
    }
}

void StepperMotor::stepMotor()
{
    // toggle the step pin and count the steps
    HAL_GPIO_TogglePin(sPort, sPin);
    if (dState)
    {
        ++step;
    }
    else
    {
        --step;
    }
}

void StepperMotor::manageDebounce()
{
    // Manage pin debouncing. Just count continous presses, and use a threshold.
    // Only check the pins if they are delcared
    if (hPin != -1 && HAL_GPIO_ReadPin(hPort, hPin))
    {
        hDB_ = 0;
    	++hDB;
    }
    else
    {
        hDB = 0;
    	++hDB_;
    }

    // Not all motors have limit pins. Check for it first
    if (lPin != -1 && HAL_GPIO_ReadPin(lPort, lPin))
    {
		lDB_ = 0;
    	++lDB;
    }
    else
    {
        lDB = 0;
        ++lDB_;
    }
}

void StepperMotor::calculateMotion(int32_t distance, int32_t speed)
{
    // Calculate the acceleration decceleration profiles. is only valid for symmetric 00 profiles
    // This is all done with floats because preserving precision is easier. If moved to something without
    // an FPU, then it probably should be done as int math and minimized aggregate error.
    cruiseSpeed = speed;
    negativeMove = false;

    // Easier to keep all the numbers positive due to square roots. This flag keeps track of if
    // we are actually moving backwards.
    if (distance < 0)
    {
        negativeMove = true;
        distance = -distance;
    }

    // Time to get up to speed.
    float timeToAccel = (float)speed / (float)accel;
    // Minimum time at speed
    float minCruiseTime = 2 * timeToAccel;
    // Minimum distance at speed
    float minCruiseDistance = (minCruiseTime * (float)speed) / 2;

    if (minCruiseDistance > distance)
    {
        // If we would get to full distance before full speed, then calculate the
        // triangular velocity profile that fits the distance
        float accelDist = distance / 2;
        // d = (1/2)at^2
        // t = sqrt(2d/a)
        float accelTime = sqrt(2 * accelDist / (float)accel);
        accelTargetTime = SECOND_US * accelTime;
        decelTargetTime = accelTargetTime;
        moveTargetTime = 2 * decelTargetTime;

        accelTargetStep = accelDist;
        decelTargetStep = accelTargetStep;
        moveTargetStep = distance;

        cruiseSpeed = accelTime * accel;
    }
    else
    {
        // if we have the time to reach full speed, there are 3 segments, acceleration
        // cruise, and deceleration.
        float cruiseDist = (float)distance - minCruiseDistance;
        accelTargetTime = SECOND_US * timeToAccel;
        decelTargetTime = accelTargetTime + SECOND_US * (cruiseDist / (float)speed);
        moveTargetTime = decelTargetTime + accelTargetTime;

        accelTargetStep = minCruiseDistance / 2;
        decelTargetStep = cruiseDist + accelTargetStep;
        moveTargetStep = distance;
    }
}

int32_t StepperMotor::calculateLerp()
{
    // Calculate the wanted step given an acceleration profile. This is run every control loop, and on a
    // Less perforant mcu may be too expensive. One solution would be to precompute the timings and put
    // Them in a table, but that is harder math. This always interpolates from 0 to a positive number, and
    // then flips the sign if needed.
    int32_t out = 0;
    int64_t deltaT = microseconds - moveStartTime;
    if (deltaT < accelTargetTime)
    {
        // initial acceleration
        out = ((int64_t)accel * deltaT * deltaT) / (2 * SECOND_US * SECOND_US);
    }
    else if (deltaT < decelTargetTime)
    {
        // cruise interpolation
        int64_t cruiseTime = deltaT - accelTargetTime;
        out = accelTargetStep + ((int64_t)cruiseSpeed * cruiseTime) / SECOND_US;
    }
    else if (deltaT < moveTargetTime)
    {
        // deceleratation
        // divide in an order that preserves as much precision as possible
        int64_t decelTime = deltaT - decelTargetTime;
        int64_t decelDividend = -((int64_t)accel * decelTime * decelTime) / (2 * SECOND_US);
        out = decelTargetStep + (decelDividend + cruiseSpeed * decelTime) / SECOND_US;
    }
    else
    {
        // completed move
        out = moveTargetStep;
    }

    // if we are going backwards, reverse the number.
    if (negativeMove)
    {
        out *= -1;
    }

    return (out);
}

bool StepperMotor::inMotion()
{
    // Return whether we can give commands or not.
    if (state == MOTOR_STATE_IDLE || state == MOTOR_STATE_COLD || state == MOTOR_STATE_UNINIT)
    {
        return (false);
    }
    return (true);
}

void StepperMotor::manageActionQueue()
{
    switch (queuedState)
    {
    case MOTOR_STATE_HOMING:
        // Only start the homin process if we have a home sensor
        if (hPin != -1)
        {
            // Enable the motors upon homing
            setEnable(true);
            state = MOTOR_STATE_HOMING;
            stepPeriod = SECOND_US / homeSpeed;
        }
        break;

    case MOTOR_STATE_LIMITING:
        // Only start the limit process if we have a limit sensor
        if (lPin != -1)
        {
            state = MOTOR_STATE_LIMITING;
            stepPeriod = SECOND_US / homeSpeed;
        }
        break;

    case MOTOR_STATE_ABORT:
        // freeze servos in their current position
        targetStep = step;
        state = MOTOR_STATE_IDLE;
        break;

    case MOTOR_STATE_RUNNING:
        // Start a command to run the motor to a position with a profile.
        calculateMotion(queuedPosition - step, queuedSpeed);
        targetStep = queuedPosition;
        moveStartTime = microseconds;
        moveStartStep = step;
        state = MOTOR_STATE_RUNNING;
        break;

    default:
        break;
    }

    queuedState = MOTOR_STATE_UNINIT;
}

int32_t StepperMotor::getEncoder()
{
    if (encoder)
    {
        int32_t tempEncoder = (int32_t)encoder->Instance->CNT - HOME_OFFSET;
        return ((tempEncoder * uSteps) / eRatio);
    }
    return (0);
}

int32_t StepperMotor::clampStep(int32_t target)
{
    if (encoder)
    {
        int32_t torqueClamp = (servoTorque * uSteps) / 100;
        int32_t max = getEncoder() + torqueClamp;
        int32_t min = getEncoder() - torqueClamp;
        if (target > max)
        {
            return (max);
        }
        if (target < min)
        {
            return (min);
        }
    }
    return (target);
}

void StepperMotor::run()
{
    // Check if there is a new action to take and reconfigure the path planning
    manageActionQueue();
    // run on a very fast timer to handle motor commutation. Commutates differently based on
    // current state of the motor
    manageDebounce();

    int32_t nominalStep = step;

    // switch the commutation state based on what we are doing
    switch (state)
    {
    case MOTOR_STATE_HOMING:
        stepIntegrator += STEPPER_CONTROL_TIMER;
        // Move towards home until we see a debounced home switch
        if (hDB > DEBOUNCE_THRESH)
        {
            // Once found, perform a fine home to get a consistent reading
            state = MOTOR_STATE_FINEHOME;
            stepPeriod = 5 * (SECOND_US / homeSpeed);
        }
        else if (stepIntegrator >= stepPeriod)
        {
            stepIntegrator = 0;
            --nominalStep;
        }
        break;

    case MOTOR_STATE_LIMITING:
        stepIntegrator += STEPPER_CONTROL_TIMER;
        // Move towards the limit until we see a debounced limit switch
        if (lDB > DEBOUNCE_THRESH)
        {
            // Once found, perform a fine limit to get a consistent reading
            state = MOTOR_STATE_FINELIMIT;
            stepPeriod = 5 * (SECOND_US / homeSpeed);
        }
        else if (stepIntegrator >= stepPeriod)
        {
            stepIntegrator = 0;
            ++nominalStep;
        }
        break;

    case MOTOR_STATE_FINEHOME:
        stepIntegrator += STEPPER_CONTROL_TIMER;
        if (hDB_ > DEBOUNCE_THRESH)
        {
            // Move until we de-assert the switch. Then we are at a home position. This is done at
            // a very slow speed to drive up repeatability.
            step = 0;
            targetStep = 0;
            nominalStep = 0;
            homed = true;
            if (encoder)
            {
                encoder->Instance->CNT = HOME_OFFSET;
            }
            state = MOTOR_STATE_HOMEOFFSET;
        }
        else if (stepIntegrator >= stepPeriod)
        {
            stepIntegrator = 0;
            ++nominalStep;
        }
        break;

    case MOTOR_STATE_HOMEOFFSET:
        // Move the motor negative to guarantee we see the homing switch fired after
        // homing the motor.
        stepIntegrator += STEPPER_CONTROL_TIMER;
        if (step <= hOffset)
        {
            state = MOTOR_STATE_IDLE;
        }
        else if (stepIntegrator >= stepPeriod)
        {
            stepIntegrator = 0;
            --nominalStep;
        }
        break;

    case MOTOR_STATE_FINELIMIT:
        stepIntegrator += STEPPER_CONTROL_TIMER;
        if (lDB_ > DEBOUNCE_THRESH)
        {
            // Move until we de-assert the switch. Then we are at a limit position. This is done at
            // a very slow speed to drive up repeatability.
            limitPosition = step;
            state = MOTOR_STATE_LIMITOFFSET;
        }
        else if (stepIntegrator >= stepPeriod)
        {
            stepIntegrator = 0;
            --nominalStep;
        }
        break;

    case MOTOR_STATE_LIMITOFFSET:
        // Move the motor negative to guarantee we see the homing switch fired after
        // homing the motor.
        stepIntegrator += STEPPER_CONTROL_TIMER;
        if (step >= limitPosition - hOffset)
        {
            state = MOTOR_STATE_IDLE;
        }
        else if (stepIntegrator >= stepPeriod)
        {
            stepIntegrator = 0;
            ++nominalStep;
        }
        break;

    case MOTOR_STATE_RUNNING:
    {
        // Calculate the lerp on a motion profile to manage accelerations and get to high speeds
        nominalStep = calculateLerp() + moveStartStep;
        // if we are done moving, go back to idle.
        if (nominalStep == targetStep)
        {
            state = MOTOR_STATE_IDLE;
        }
        nominalStep = clampStep(nominalStep);
    }
    break;

    case MOTOR_STATE_IDLE:
    {
        if (homed && encoder)
        {
            nominalStep = clampStep(targetStep);
        }
    }
    break;

    default:
        break;
    }

    // handle step commutation
    if (nominalStep > step)
    {
        setDirection(true);
        stepMotor();
    }
    else if (nominalStep < step)
    {
        setDirection(false);
        stepMotor();
    }
}

void StepperMotor::runToHome()
{
    queuedState = MOTOR_STATE_HOMING;
}

void StepperMotor::runToLimit()
{
    queuedState = MOTOR_STATE_LIMITING;
}

void StepperMotor::abort()
{
    queuedState = MOTOR_STATE_ABORT;
}

void StepperMotor::runToPosition(int32_t position, int32_t speed)
{
    if (speed <= 0)
    {
        // No going backwards in time
        return;
    }
    queuedSpeed = speed;
    queuedPosition = position;
    queuedState = MOTOR_STATE_RUNNING;
}

void runStepperMotors()
{
    microseconds += STEPPER_CONTROL_TIMER;
    // run the interrupt that handles each stepper
    flipStepper.run();
    yStepper.run();
    zStepper.run();
}
