#include <stdbool.h>
#include "scheduler.c"
#include "CuTest.h"

int expectedMotorNumber;

static void schedulerInitTest( CuTest *tc );
static void moveMotorTest( CuTest *tc );

static void schedulerInitTest( CuTest *tc ) {
    int i;

    CuAssert( tc, "Did not successfully initialize.", schedulerInit() );

    for( i = 0; i < NUM_MOTORS; ++i ) {
        CuAssert( tc, "Did not set the status.", motorMovement[i].motorStatus == Idle );
        CuAssert( tc, "Did not set the steps.", motorMovement[i].steps == 0 );
        CuAssert( tc, "Did not set the fractionStep.", motorMovement[i].fractionalStep == 0 );
        CuAssert( tc, "Did not set the maxSpeed.", motorMovement[i].maxSpeed == 0 );
        CuAssert( tc, "Did not set the speed.", motorMovement[i].speed == 0 );
        CuAssert( tc, "Did not set the fractionalSpeed.", motorMovement[i].fractionalSpeed == 0 );
        CuAssert( tc, "Did not set the acceleration.", motorMovement[i].acceleration == 0 );
        CuAssert( tc, "Did not set the fractionalAcceleration.", motorMovement[i].fractionalAcceleration == 0 );
    }
}

static void moveMotorTest( CuTest *tc ) {
    expectedMotorNumber = -1;
    CuAssert( tc, "Haven't told expected motor number, should fail.", !moveMotor( 0 ) );

    expectedMotorNumber = 0;
    CuAssert( tc, "Gave the expected motor number, should work.", moveMotor( 0 ) );
}

CuSuite* CuGetSuite( void ) {
    CuSuite* suite = CuSuiteNew();

    SUITE_ADD_TEST( suite, schedulerInitTest );
    SUITE_ADD_TEST( suite, moveMotorTest );

    return suite;
}