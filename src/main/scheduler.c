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
        motorMovement[i].fractionalSpeed = 0;
        motorMovement[i].acceleration = 0;
        motorMovement[i].fractionalAcceleration = 0;
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
