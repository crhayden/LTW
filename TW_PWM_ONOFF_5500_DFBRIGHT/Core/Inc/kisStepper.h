#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "stdint.h"
#include "stm32f4xx.h"
#include "stdbool.h"

enum MOTOR_STATE
{
    MOTOR_STATE_UNINIT,
    MOTOR_STATE_COLD,
    MOTOR_STATE_HOMING,
    MOTOR_STATE_FINEHOME,
    MOTOR_STATE_HOMEOFFSET,
    MOTOR_STATE_LIMITING,
    MOTOR_STATE_FINELIMIT,
    MOTOR_STATE_LIMITOFFSET,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_IDLE,
    MOTOR_STATE_ABORT,
};

class StepperMotor
{
public:
    StepperMotor();
    void initializeCold(int enablePin, GPIO_TypeDef *enablePort, int stepPin, GPIO_TypeDef *stepPort, int dirPin, GPIO_TypeDef *dirPort, int homePin, GPIO_TypeDef *homePort, int limitPin, GPIO_TypeDef *limitPort, int32_t acc, int32_t hSpd, TIM_HandleTypeDef *encTim, float encRatio, uint8_t microsteps, int spiPin, GPIO_TypeDef *spiPort, int32_t homeOffset = -20);
    void runToPosition(int32_t position, int32_t speed);    // Run the motor to a nominal position
    void runToHome();                                       // Run the motor to the home sensor
    void runToLimit();                                      // Run the motor to the limit sensor
    void abort();                                           // Cancel current motor commands
    void setCurrent(uint8_t irun, uint8_t ihold);           // Set the run and hold current for the stepper
    void setAcceleration(int32_t acc);                      // Set the nominal acceleration for the stepper
    void setServoTorque(uint8_t percent);                   // Set the servo torque if in servo mode
    void setEnable(bool enable);                            // Enable or disable the motor
    int32_t getStep(){ return(step); };                     // Get the nominal step postion of the motor
    int32_t getEncoder();                                   // Get the encoder position (in steps)
    void run();                                             // Run the motor control lool
    bool inMotion();                                        // Is the motor currently executing a command?
    bool isHomed(){ return(homed); };                       // Has the motor been homed since boot?
    bool isEnabled(){ return(enabled); };                   // Is the motor currently enabled
    void invertDirection();                                 // Reverse the motor direction for all commands

private:
    void stepMotor();
    void setDirection(bool dir);
    void manageDebounce();
    void manageActionQueue();
    void calculateMotion(int32_t distance, int32_t speed);
    int32_t clampStep(int32_t target);
    int32_t calculateLerp();
    int32_t accel;
    int32_t servoTorque;
    int32_t hOffset;
    int ePin;                                               // Enable Pin
    GPIO_TypeDef *ePort;                                    // Enable Port
    int sPin;                                               // Step Pin
    GPIO_TypeDef *sPort;                                    // Step Port
    volatile bool sState;                                   // Cached step state
    int dPin;                                               // Direction Pin
    GPIO_TypeDef *dPort;                                    // Direction Port
    volatile bool dState;                                   // Cached direction State
    int hPin;                                               // Home Pin (declare -1 if not present)
    GPIO_TypeDef *hPort;                                    // Home Port (declare NULL if not present)
    int lPin;                                               // Limit Pin (declare -1 if not present)
    GPIO_TypeDef *lPort;                                    // Limit Port (declare NULL if not present)
    int selectPin;                                          // SPI chip select pin
    GPIO_TypeDef *selectPort;                               // SPI chip select port
    TIM_HandleTypeDef *encoder;                             // Encoder timer. If provided stepper will act as a servo
    int state;                                              // Current motor operation state, see MOTOR_STATE
    volatile int32_t step;                                  // Current motor step
    volatile int32_t targetStep;                            // Step we are moving to
    volatile int32_t limitPosition;                         // Used when limiting to set final position
    float eRatio;                                           // Ratio of encoder tics to full steps
    int32_t uSteps;                                         // Number of microsteps per step (MUST BE SET EXTERNALLY)
    bool homed;                                             // Is the motor homed?
    bool enabled;                                           // Is the motor enabled?
    bool invertDir;                                         // Have we inverted the direction?

    
    // Used for running at a constant velocity during homing
    volatile int32_t stepIntegrator;
    volatile int32_t stepPeriod;
    volatile uint32_t hDB;
    volatile uint32_t lDB;
    volatile uint32_t hDB_;
    volatile uint32_t lDB_;
    // Used to queue an action to process during the interrupt
    volatile int queuedState;
    volatile int32_t queuedPosition;
    volatile int32_t queuedSpeed;

    // Ramp Control Constants
    unsigned long moveStartTime;
    unsigned long accelTargetTime;
    unsigned long decelTargetTime;
    unsigned long moveTargetTime;
    int32_t moveStartStep;
    int32_t accelTargetStep;
    int32_t decelTargetStep;
    int32_t moveTargetStep;
    int32_t cruiseSpeed;
    int32_t homeSpeed;
    bool negativeMove;
};

// Expose statically allocated motors
extern StepperMotor flipStepper;
extern StepperMotor yStepper;
extern StepperMotor zStepper;

void runStepperMotors();


#endif
