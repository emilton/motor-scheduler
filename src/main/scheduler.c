#include <stdint.h>
#include "scheduler.h"

#define NUM_MOTORS 1

static MotorMovement motorMovement[NUM_MOTORS];

static int moveMotor( int8_t motorNumber );

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

static int moveMotor( int8_t motorNumber ) {
#ifdef TEST
    static int8_t lastMotorNumber = INT8_MAX;

    if( motorNumber < 0 ) {
        if( lastMotorNumber >= 0 && -( motorNumber + 1 ) == lastMotorNumber ) {
            lastMotorNumber = INT8_MAX;
            return 1;
        } else {
            return 0;
        }
    } else if( lastMotorNumber != INT8_MAX ) {
        return 0;
    } else {
        lastMotorNumber = motorNumber;
        return 1;
    }
#else
    if( motorNumber >= 0 && motorNumber < NUM_MOTORS ) {
        return 1;
    } else {
        return 0;
    }
#endif
}
