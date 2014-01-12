#include <stdint.h>
#include <stdlib.h>
#include "scheduler.h"

#define NUM_MOTORS 1
#define sign(x) ( x >= 0 )

static MotorMovement motorMovement[NUM_MOTORS];

#ifdef TEST
extern int expectedMotorNumber;
extern int expectedMotorDirection;
#endif

static void updateSpeed( MotorMovement * );
static void updateMotorStatus( MotorMovement * );
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
    MotorMovement *motor;

    for( i = 0; i < NUM_MOTORS; ++i ) {
        motor = &motorMovement[i];
        updateSpeed( motor );

        oldSign = sign( motor->fractionalStep );
        speedSign = sign( motor->speed );
        motor->fractionalStep += motor->speed;
        if( oldSign != sign( motor->fractionalStep ) && oldSign == speedSign ) {
            if( !moveMotor( i, speedSign ) ) {
                return 0;
            }
            motor->stepsTaken++;
        }

        updateMotorStatus( motor );
    }

    return 1;
}

static void updateSpeed( MotorMovement *motor ) {
    switch( motor->motorStatus ) {
        case Idle:
            break;
        case Accelerating:
            motor->speed += motor->acceleration;
            if( abs( motor->speed ) > abs( motor->maxSpeed ) ) {
                motor->speed = motor->maxSpeed;
            }
            break;
        case Deaccelerating:
            motor->speed -= motor->acceleration;
            if( sign( motor->speed ) != sign( motor->acceleration ) ) {
                motor->speed = 0;
            }
            break;
        case ConstantSpeed:
            break;
    }
}

static void updateMotorStatus( MotorMovement *motor ) {
    switch( motor->motorStatus ) {
        case Idle:
            break;
        case Accelerating:
            if( motor->stepsTaken >= ( motor->steps >> 1 ) ) {
                motor->motorStatus = Deaccelerating;
            } else if( motor->speed == motor->maxSpeed ) {
                motor->motorStatus = ConstantSpeed;
                motor->deaccelerationStart = motor->steps - motor->stepsTaken;
            }
            break;
        case Deaccelerating:
            if( motor->speed == 0 ) {
                motor->motorStatus = Idle;
            }
            break;
        case ConstantSpeed:
            if( motor->stepsTaken >= motor->deaccelerationStart ) {
                motor->motorStatus = Deaccelerating;
            }
            break;
    }

    if( motor->stepsTaken == motor->steps ) {
        motor->motorStatus = Idle;
    }
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
