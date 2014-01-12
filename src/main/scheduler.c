#include <stdint.h>
#include "scheduler.h"

#define NUM_MOTORS 1
#define sign(x) ( x >= 0 )

static MotorMovement motorMovement[NUM_MOTORS];

#ifdef TEST
extern int expectedMotorNumber;
extern int expectedMotorDirection;
#endif

static int moveMotor( int, int );

int schedulerInit( void ) {
    int i;

    for( i = 0; i < NUM_MOTORS; ++i ) {
        motorMovement[i].motorStatus = Idle;
        motorMovement[i].steps = 0;
        motorMovement[i].stepsTaken = 0;
        motorMovement[i].fractionalStep = 0;
        motorMovement[i].maxSpeed = 0;
        motorMovement[i].speed = 0;
        motorMovement[i].acceleration = 0;
    }

    return 1;
}

int updateMotors( void ) {
    int i;
    char oldSign, speedSign;
    MotorMovement *currentMotor;

    for( i = 0; i < NUM_MOTORS; ++i ) {
        currentMotor = &motorMovement[i];
        switch( currentMotor->motorStatus ) {
            case Idle:
                continue;
            case Accelerating:
                currentMotor->speed += currentMotor->acceleration;
                if( currentMotor->speed >= currentMotor->maxSpeed ) {
                    currentMotor->motorStatus = ConstantSpeed;
                    currentMotor->speed = currentMotor->maxSpeed;
                }
                break;
            case Deaccelerating:
                break;
            case ConstantSpeed:
                break;
            default:
                return 0;
        }

        oldSign = sign( currentMotor->fractionalStep );
        speedSign = sign( currentMotor->speed );
        currentMotor->fractionalStep += currentMotor->speed;
        if( oldSign != sign( currentMotor->fractionalStep ) && oldSign == speedSign ) {
            if( !moveMotor( i, speedSign ) ) {
                return 0;
            }
            currentMotor->stepsTaken++;
            if( currentMotor->motorStatus == Accelerating && currentMotor->stepsTaken >= ( currentMotor->steps >> 1 ) ) {
                currentMotor->motorStatus = Deaccelerating;
            }
            if( currentMotor->stepsTaken == currentMotor->steps ) {
                currentMotor->motorStatus = Idle;
            }
        }
    }

    return 1;
}

static int moveMotor( int motorNumber, int forwardDirection ) {
#ifdef TEST
    if( expectedMotorNumber < 0 || expectedMotorDirection < 0 ) {
        return 0;
    } else {
        return motorNumber == expectedMotorNumber && forwardDirection == expectedMotorDirection;
    }
#else
    if( motorNumber >= 0 && motorNumber < NUM_MOTORS ) {
        if( forwardDirection ) {
            /* Move motor forward. */
            return 1;
        } else {
            /* Move motor backward. */
            return 1;
        }
    } else {
        return 0;
    }
#endif
}
