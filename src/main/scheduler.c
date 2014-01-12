#include <stdint.h>
#include "scheduler.h"

#define NUM_MOTORS 1

static MotorMovement motorMovement[NUM_MOTORS];

#ifdef TEST
extern int expectedMotorNumber;
#endif

static int moveMotor( int motorNumber );

int schedulerInit( void ) {
    int i;

    for( i = 0; i < NUM_MOTORS; ++i ) {
        motorMovement[i].motorStatus = Idle;
        motorMovement[i].steps = 0;
        motorMovement[i].fractionalStep = 0;
        motorMovement[i].maxSpeed = 0;
        motorMovement[i].speed = 0;
        motorMovement[i].acceleration = 0;
    }

    return 1;
}

int updateMotors( void ) {
    int i;
    int32_t oldFractionalStep;
    MotorMovement *currentMotor;

    for( i = 0; i < NUM_MOTORS; ++i ) {
        currentMotor = &motorMovement[i];
        switch( currentMotor->motorStatus ) {
            case Idle:
                continue;
            case Accelerating:
                currentMotor->speed += currentMotor->acceleration;
                if( currentMotor->speed >= currentMotor->maxSpeed ) {
                    currentMotor->speed = currentMotor->maxSpeed;
                    currentMotor->motorStatus = ConstantSpeed;
                }
                break;
            case Deaccelerating:
                break;
            case ConstantSpeed:
                break;
            default:
                return 0;
        }

        oldFractionalStep = currentMotor->fractionalStep;
        currentMotor->fractionalStep += currentMotor->speed;
        if( oldFractionalStep > 0 && currentMotor->speed > 0 && currentMotor->fractionalStep < 0 ) {
            if( !moveMotor( i ) ) {
                return 0;
            }
        } else if( oldFractionalStep < 0 && currentMotor->speed < 0 && currentMotor->fractionalStep > 0 ) {
            if( !moveMotor( i ) ) {
                return 0;
            }
        }
    }

    return 1;
}

static int moveMotor( int motorNumber ) {
#ifdef TEST
    if( expectedMotorNumber < 0 ) {
        return 0;
    } else {
        return motorNumber == expectedMotorNumber;
    }
#else
    if( motorNumber >= 0 && motorNumber < NUM_MOTORS ) {
        return 1;
    } else {
        return 0;
    }
#endif
}
