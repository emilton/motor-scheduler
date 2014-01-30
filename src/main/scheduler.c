#include <stdint.h>
#include <stdlib.h>
#include "comm.h"
#include "scheduler.h"

#define sign(x) ( x >= 0 )

static MotorMovement motorMovement[NUM_MOTORS];

static int moveMotor( int, int );

int schedulerInit( void ) {
    int i;

    for( i = 0; i < NUM_MOTORS; ++i ) {
        motorMovement[i].steps = 0;
        motorMovement[i].fractionalStep = 0;
        motorMovement[i].speed = 0;
        motorMovement[i].acceleration = 0;
    }

    return 1;
}

int updateMotors( void ) {
    int i;
    MotorMovement *motor;
    int32_t oldFractionalStep;

    for( i = 0; i < NUM_MOTORS; ++i ) {
        motor = &motorMovement[i];
        if( motor->steps ) {
            motor->speed += motor->acceleration;

            oldFractionalStep = motor->fractionalStep;
            motor->fractionalStep += motor->speed;
            if( ( motor->fractionalStep ^ motor->speed ) & ( motor->fractionalStep ^ oldFractionalStep ) & 0x80000000 ) {
                if( !moveMotor( i, sign( motor->speed ) ) ) {
                    return 0;
                }
                motor->steps--;
            }
        }
    }

    return 1;
}

#ifndef TEST
static int moveMotor( int motorNumber, int forwardDirection ) {
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
}
#endif
