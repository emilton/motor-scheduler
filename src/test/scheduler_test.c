#include <stdbool.h>
#include "scheduler.c"
#include "CuTest.h"

#define updateMotors() CuAssert( tc, "Should not have error updating motors.", updateMotors() )
#undef NUM_MOTORS
#define NUM_MOTORS 1

int expectedMotorNumber;
int expectedMotorDirection;

static void schedulerInitTest( CuTest *tc );
static void moveMotorTest( CuTest *tc );
static void setupMotors( void );
static void updateMotorsTest_Accelerating( CuTest *tc );
static void updateMotorsTest_ConstantSpeed( CuTest *tc );
static void updateMotorsTest_MoveMotor( CuTest *tc );
static void updateMotorsTest_CompleteMove( CuTest *tc );

static void schedulerInitTest( CuTest *tc ) {
    int i;

    CuAssert( tc, "Did not successfully initialize.", schedulerInit() );

    for( i = 0; i < NUM_MOTORS; ++i ) {
        CuAssert( tc, "Did not set the steps.", motorMovement[i].steps == 0 );
        CuAssert( tc, "Did not set the fractionStep.", motorMovement[i].fractionalStep == 0 );
        CuAssert( tc, "Did not set the speed.", motorMovement[i].speed == 0 );
        CuAssert( tc, "Did not set the acceleration.", motorMovement[i].acceleration == 0 );
    }
}

static void moveMotorTest( CuTest *tc ) {
    expectedMotorNumber = -1;
    CuAssert( tc, "Haven't told expected motor number, should fail.", !moveMotor( 0, 1 ) );

    expectedMotorNumber = 0;
    expectedMotorDirection = 1;
    CuAssert( tc, "Gave the expected motor number and direction, should work.", moveMotor( 0, 1 ) );

    expectedMotorDirection = 0;
    CuAssert( tc, "Gave the expected motor number and direction, should work.", moveMotor( 0, 0 ) );
}

static void setupMotors( void ) {
    motorMovement[0].steps = 100;
    motorMovement[0].fractionalStep = 0;
    motorMovement[0].speed = 0;
    motorMovement[0].acceleration = 100;

    expectedMotorNumber = 0;
    expectedMotorDirection = 1;
}

static void updateMotorsTest_Accelerating( CuTest *tc ) {
    int i;
    setupMotors();

    for( i = 0; i < 100; ++i ) {
        updateMotors();
    }
    CuAssert( tc, "Speed should be max.", motorMovement[0].speed == 10000 );
    CuAssert( tc, "Fractional step should be 100*(100*101)/2.", motorMovement[0].fractionalStep == 505000 );

    motorMovement[0].acceleration = -100;
    updateMotors();
    CuAssert( tc, "Speed should be decremented.", motorMovement[0].speed == 9900 );
}

static void updateMotorsTest_ConstantSpeed( CuTest *tc ) {
    setupMotors();
    motorMovement[0].speed = 100;
    motorMovement[0].acceleration = 0;

    updateMotors();
    updateMotors();
    CuAssert( tc, "Should have moved at a constant speed.", motorMovement[0].fractionalStep == 200 );
}

static void updateMotorsTest_MoveMotor( CuTest *tc ) {
    setupMotors();
    motorMovement[0].fractionalStep = INT32_MAX - 100;

    expectedMotorNumber = -1;
    updateMotors();
    expectedMotorNumber = 0;
    expectedMotorDirection = 1;
    updateMotors();
    CuAssert( tc, "Should have taken a step.", motorMovement[0].steps == 99 );

    motorMovement[0].speed = 0;
    motorMovement[0].acceleration = -100;
    motorMovement[0].fractionalStep = INT32_MIN + 100;

    expectedMotorNumber = -1;
    updateMotors();
    expectedMotorNumber = 0;
    expectedMotorDirection = 0;
    updateMotors();
    CuAssert( tc, "Should have taken a step.", motorMovement[0].steps == 98 );
}

static void updateMotorsTest_CompleteMove( CuTest *tc ) {
    setupMotors();
    motorMovement[0].fractionalStep = INT32_MAX - 99;
    motorMovement[0].steps = 1;

    updateMotors();
    CuAssert( tc, "Should be done moving.", motorMovement[0].fractionalStep == INT32_MIN );
    updateMotors();
    CuAssert( tc, "Should be done moving.", motorMovement[0].fractionalStep == INT32_MIN );

}

CuSuite* CuGetSuite( void ) {
    CuSuite* suite = CuSuiteNew();

    SUITE_ADD_TEST( suite, schedulerInitTest );
    SUITE_ADD_TEST( suite, moveMotorTest );
    SUITE_ADD_TEST( suite, updateMotorsTest_Accelerating );
    SUITE_ADD_TEST( suite, updateMotorsTest_ConstantSpeed );
    SUITE_ADD_TEST( suite, updateMotorsTest_MoveMotor );
    SUITE_ADD_TEST( suite, updateMotorsTest_CompleteMove );

    return suite;
}

static int moveMotor( int motorNumber, int forwardDirection ) {
    if( expectedMotorNumber < 0 || expectedMotorDirection < 0 ) {
        return 0;
    } else {
        return motorNumber == expectedMotorNumber && forwardDirection == expectedMotorDirection;
    }
}
