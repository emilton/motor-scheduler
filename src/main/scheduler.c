#include <stdint.h>
#include <stdlib.h>
#include "comm.h"
#include "scheduler.h"

#define sign(x) ( x >= 0 )

static MotorMovement motorMovement[NUM_MOTORS];

#pragma CODE_SECTION( updateMotors, "ramfuncs" );
#pragma CODE_SECTION( applyCommand, "ramfuncs" );
#pragma CODE_SECTION( applyAcceleration, "ramfuncs" );
#pragma CODE_SECTION( applyConstantSpeed, "ramfuncs" );

static void applyAcceleration( Accelerating_t *accelerating );
static void applyConstantSpeed( ConstantSpeed_t *constantSpeed );

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

int applyCommand( Command_t *command ) {
    switch( command->commandType ) {
        case Accelerating:
            applyAcceleration( &command->command.accelerating );
            break;
        case ConstantSpeed:
            applyConstantSpeed( &command->command.constantSpeed );
            break;
        default:
            return 0;
    }

    return 1;
}

static void applyAcceleration( Accelerating_t *accelerating ) {
    int i;

    for( i = 0; i < NUM_MOTORS; i++ ) {
        motorMovement[i].steps = accelerating->steps[i];
        motorMovement[i].acceleration = accelerating->accelerations[i];
    }
}

static void applyConstantSpeed( ConstantSpeed_t *constantSpeed ) {
    int i;

    for( i = 0; i < NUM_MOTORS; i++ ) {
        motorMovement[i].steps = constantSpeed->steps[i];
        motorMovement[i].acceleration = 0;
        motorMovement[i].speed = constantSpeed->speeds[i];
    }
}
