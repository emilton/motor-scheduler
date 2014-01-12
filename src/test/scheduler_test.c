#include <stdbool.h>
#include "scheduler.c"
#include "CuTest.h"

#define updateMotors() CuAssert( tc, "Should not have error updating motors.", updateMotors() )

int expectedMotorNumber;

static void schedulerInitTest( CuTest *tc );
static void moveMotorTest( CuTest *tc );
static void setupMotors( void );
static void updateMotorsTest_Idle( CuTest *tc );
static void updateMotorsTest_Accelerating( CuTest *tc );
static void updateMotorsTest_AcceleratingTriangle( CuTest *tc );
static void updateMotorsTest_MoveMotor( CuTest *tc );

static void schedulerInitTest( CuTest *tc ) {
    int i;

    CuAssert( tc, "Did not successfully initialize.", schedulerInit() );

    for( i = 0; i < NUM_MOTORS; ++i ) {
        CuAssert( tc, "Did not set the status.", motorMovement[i].motorStatus == Idle );
        CuAssert( tc, "Did not set the steps.", motorMovement[i].steps == 0 );
        CuAssert( tc, "Did not set the fractionStep.", motorMovement[i].fractionalStep == 0 );
        CuAssert( tc, "Did not set the maxSpeed.", motorMovement[i].maxSpeed == 0 );
        CuAssert( tc, "Did not set the speed.", motorMovement[i].speed == 0 );
        CuAssert( tc, "Did not set the acceleration.", motorMovement[i].acceleration == 0 );
    }
}

static void moveMotorTest( CuTest *tc ) {
    expectedMotorNumber = -1;
    CuAssert( tc, "Haven't told expected motor number, should fail.", !moveMotor( 0 ) );

    expectedMotorNumber = 0;
    CuAssert( tc, "Gave the expected motor number, should work.", moveMotor( 0 ) );
}

static void setupMotors( void ) {
    motorMovement[0].motorStatus = Idle;
    motorMovement[0].steps = 100;
    motorMovement[0].fractionalStep = 0;
    motorMovement[0].maxSpeed = 10000;
    motorMovement[0].speed = 0;
    motorMovement[0].acceleration = 100;

    expectedMotorNumber = 0;
}

static void updateMotorsTest_Idle( CuTest *tc ) {
    setupMotors();

    updateMotors();
    CuAssert( tc, "Speed should be still be zero.", motorMovement[0].speed == 0 );
    CuAssert( tc, "Fractional step should be still be zero.", motorMovement[0].fractionalStep == 0 );
}

static void updateMotorsTest_Accelerating( CuTest *tc ) {
    int i;
    setupMotors();

    motorMovement[0].motorStatus = Accelerating;
    for( i = 0; i < 100; ++i ) {
        updateMotors();
    }
    CuAssert( tc, "Speed should be max.", motorMovement[0].speed == 10000 );
    CuAssert( tc, "Should now be in constant speed mode.", motorMovement[0].motorStatus == ConstantSpeed );
    CuAssert( tc, "Fractional step should be 100*(100*101)/2.", motorMovement[0].fractionalStep == 505000 );
}

static void updateMotorsTest_AcceleratingTriangle( CuTest *tc ) {
    setupMotors();
    updateMotors();
}

static void updateMotorsTest_MoveMotor( CuTest *tc ) {
    setupMotors();
    motorMovement[0].motorStatus = Accelerating;
    motorMovement[0].fractionalStep = INT32_MAX - 100;

    expectedMotorNumber = -1;
    updateMotors();
    expectedMotorNumber = 0;
    updateMotors();

    motorMovement[0].speed = 0;
    motorMovement[0].acceleration = -100;
    motorMovement[0].fractionalStep = INT32_MIN + 100;

    expectedMotorNumber = -1;
    updateMotors();
    expectedMotorNumber = 0;
    updateMotors();
}

CuSuite* CuGetSuite( void ) {
    CuSuite* suite = CuSuiteNew();

    SUITE_ADD_TEST( suite, schedulerInitTest );
    SUITE_ADD_TEST( suite, moveMotorTest );
    SUITE_ADD_TEST( suite, updateMotorsTest_Idle );
    SUITE_ADD_TEST( suite, updateMotorsTest_Accelerating );
    SUITE_ADD_TEST( suite, updateMotorsTest_AcceleratingTriangle );
    SUITE_ADD_TEST( suite, updateMotorsTest_MoveMotor );

    return suite;
}