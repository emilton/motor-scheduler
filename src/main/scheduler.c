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

static int applyAcceleration( Accelerating_t *accelerating );
static int applyConstantSpeed( ConstantSpeed_t *constantSpeed );

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

    for( i = 0; i < NUM_MOTORS; ++i ) {
        motor = &motorMovement[i];
        if( motor->steps ) {
            motor->speed += motor->acceleration;

            motor->fractionalStep += motor->speed;
#ifdef x86
            asm( "jno noOverflow" );
#else
            asm( " sb noOverflow cond nov" );
#endif
                if( !moveMotor( i ) ) {
                    return 0;
                }
                motor->steps--;
            asm( "noOverflow:" );
        }
    }

    return 1;
}

int applyCommand( Command_t *command ) {
    switch( command->commandType ) {
        case Accelerating:
            return applyAcceleration( &command->command.accelerating );
        case ConstantSpeed:
            return applyConstantSpeed( &command->command.constantSpeed );
        default:
            return 0;
    }
}

static int applyAcceleration( Accelerating_t *accelerating ) {
    int i;

    for( i = 0; i < NUM_MOTORS; i++ ) {
        if( ! setDirection( i, sign( motorMovement[i].acceleration ) ) ) {
            return 0;
        }
        motorMovement[i].steps = accelerating->steps[i];
        motorMovement[i].acceleration = accelerating->accelerations[i];
    }

    return 1;
}

static int applyConstantSpeed( ConstantSpeed_t *constantSpeed ) {
    int i;

    for( i = 0; i < NUM_MOTORS; i++ ) {
        motorMovement[i].steps = constantSpeed->steps[i];
        motorMovement[i].acceleration = 0;
        motorMovement[i].speed = constantSpeed->speeds[i];
    }

    return 1;
}
