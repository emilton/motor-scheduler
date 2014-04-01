#include <stdint.h>
#include <stdlib.h>
#include "comm.h"
#include "scheduler.h"

#define sign(x) ( x >= 0 )

static MotorMovement motorMovement[NUM_MOTORS];
static Command_t	 commandBuffer[MAX_COMMANDS];
static Command_t*    thisCommand;
static Command_t*    emptyCommand;


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
static void applyConfiguration( Configuration_t *configuration );

void applyDirection( int, int );
void applyStep( int );

int schedulerInit( void ) {
    int i;

    // Create command buffer ring.
    for( i = 0; i < MAX_COMMANDS-1; i++ ){
    	commandBuffer[i].nextCommand = &commandBuffer[i+1];
    }
    commandBuffer[i].nextCommand = &commandBuffer[0];

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
inline int calculate( MotorMovement *motor, int i ){
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
	return motor->steps;
}

void updateMotors( void ) {
	if(!(calculate( &motorMovement[0], X_STEP ) ||
		 calculate( &motorMovement[1], Y_STEP ) ||
		 calculate( &motorMovement[2], Z_STEP ) ||
		 calculate( &motorMovement[3], A_STEP ) )){
		 applyCommand( thisCommand );
	}

}

int insertCommand( Command_t* command ){
	if( command->commandType == EmergencyStop );
		//applyEmergencyStop( command );
	if( emptyCommand->commandType != NoOp )
		return 0;
	else{
		Command_t* t = emptyCommand->nextCommand;
		*emptyCommand = *command;
		emptyCommand = t;
		return 1;
	}
}

int applyCommand( Command_t* command ) {
    switch( command->commandType ) {
        case Accelerating:
            applyAcceleration( &command->command.accelerating );
            break;
        case ConstantSpeed:
            applyConstantSpeed( &command->command.constantSpeed );
            break;
        case Configuration:
            applyConfiguration( &command->command.configuration );
            break;
        default:
            return 0;
    }

    command->commandType = NoOp;
    thisCommand = thisCommand->nextCommand;
    return 1;
}

int applyEmergencyStop(){
	// stop shit
}

static void applyAcceleration( Accelerating_t *accelerating ) {
    int i;

    for( i = 0; i < NUM_MOTORS; i++ ) {
        motorMovement[i].steps = accelerating->steps[i];
        motorMovement[i].acceleration = accelerating->accelerations[i];

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

static void applyConfiguration( Configuration_t *configuration ) {
	int i;

	for( i = 0; i < NUM_MOTORS; i++ ) {

	}
}
