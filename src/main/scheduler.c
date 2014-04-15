#include <stdint.h>
#include <stdlib.h>
#include "comm.h"
#include "scheduler.h"

static MotorMovement motorMovement[NUM_MOTORS];

#pragma CODE_SECTION( updateMotors, "ramfuncs" );
#pragma CODE_SECTION( applyCommand, "ramfuncs" );
#pragma CODE_SECTION( applyAcceleration, "ramfuncs" );
#pragma CODE_SECTION( applyConstantSpeed, "ramfuncs" );

static int applyAcceleration( Accelerating_t *accelerating );
static int applyConstantSpeed( ConstantSpeed_t *constantSpeed );
static int isHoming = 0;

int schedulerInit( void ) {
    memset( motorMovement, 0, sizeof(motorMovement) );
/*
    int i;
    for( i = 0; i < NUM_MOTORS; ++i ) {
        motorMovement[i].steps = 0;
        motorMovement[i].fractionalStep = 0;
        motorMovement[i].speed = 0; // (2^32)/50k/desired freq
        motorMovement[i].acceleration = 0;
    }
*/
    return 1;
}

int updateMotors( void ) {
    MotorMovement *motor;
    int hasSteps = 0;
    int i;

    for( i = 0; i < NUM_MOTORS; ++i ) {
        motor = &motorMovement[i];
        if( isHoming ){
        	if( checkHome(i) ){
        		motor->steps = 0;
        	}
        }
        if( motor->steps ) {
            hasSteps = 1;
            motor->speed += motor->acceleration;
		#ifndef x86
            asm( " sb clearVflag cond nov" );
            asm( "clearVflag" );
		#endif
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
    if( !hasSteps ) {
        getNewCommand();
    }
    return 1;
}

int applyCommand( Command_t *command ) {
    switch( command->commandType & 0x000000FF ) {
        case Accelerating:
            return applyAcceleration( &command->command.accelerating );
        case ConstantSpeed:
            return applyConstantSpeed( &command->command.constantSpeed );
        case WorkHead:
            return 1;
        case Home:
            return homeMachine( &command->command.home );
        default:
            return 0;
    }
}

static int applyAcceleration( Accelerating_t *accelerating ) {
    int i;

    for( i = 0; i < NUM_MOTORS; i++ ) {
        motorMovement[i].steps = accelerating->steps[i];
        motorMovement[i].acceleration = accelerating->accelerations[i];
    }
    return 1;
}

static int applyConstantSpeed( ConstantSpeed_t *constantSpeed ) {
    int i;

    for( i = 0; i < NUM_MOTORS; i++ ) {
        if( constantSpeed->steps[i] ) {
            motorMovement[i].steps = constantSpeed->steps[i];
            motorMovement[i].acceleration = 0;
            motorMovement[i].speed = constantSpeed->speeds[i];
        }
    }
    return 1;
}

static int homeMachine( Home_t *home ){
	isHoming = isHoming ^ 0x01;
}
