#include <stdint.h>
#include <stdlib.h>
#include "comm.h"
#include "scheduler.h"

#define sign(x) ( x >= 0 )

static MotorMovement motorMovement[NUM_MOTORS];

// I feel like this is bad, I'll need to talk linking.
#define X_STEP ( 1 << 0 )
#define Y_STEP ( 1 << 2 )
#define Z_STEP ( 1 << 7 )
#define A_STEP ( 1 << 5 )

#pragma CODE_SECTION( updateMotors, "ramfuncs" );
#pragma CODE_SECTION( applyCommand, "ramfuncs" );
#pragma CODE_SECTION( applyAcceleration, "ramfuncs" );
#pragma CODE_SECTION( applyConstantSpeed, "ramfuncs" );

static void applyAcceleration( Accelerating_t *accelerating );
static void applyConstantSpeed( ConstantSpeed_t *constantSpeed );
void applyDirection( int, int );
void applyStep( int );

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

// I've trimmed this down a bit, for loop adds overhead.
// GPATOGGLE is the fastest way to toggle a pin -BUT-
// because of the opto's, we'll need another interrupt
// that keeps it high for 2uS.
inline void set( MotorMovement *motor, int i ){
	int32_t oldFractionalStep;
	if( motor->steps ) {
		motor->speed += motor->acceleration;
		oldFractionalStep = motor->fractionalStep;
		motor->fractionalStep += motor->speed;
		if( ( motor->fractionalStep ^ motor->speed ) & ( motor->fractionalStep ^ oldFractionalStep ) & 0x80000000 ) {
			applyStep( i );
			motor->steps--;
		}
	}
}
int updateMotors( void ) {
	set( &motorMovement[0], X_STEP );
    set( &motorMovement[1], Y_STEP );
    set( &motorMovement[2], Z_STEP );
    set( &motorMovement[3], A_STEP );
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

    	// before applying a new direction, make sure the motor is not moving.
        if( motorMovement[i].speed == 0 )
        	applyDirection( i, sign(motorMovement[i].speed ) );
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
