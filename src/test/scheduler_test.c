#include <stdbool.h>
#include "scheduler.c"
#include "CuTest.h"

void schedulerInitTest( CuTest* tc );

void schedulerInitTest( CuTest* tc ) {
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

void motorMovementTest( CuTest* tc ) {
    CuAssert( tc, "First call should work.", moveMotor( 0 ) == 1 );

    CuAssert( tc, "Subsequent calls should not work.", moveMotor( 0 ) == 0 );
    CuAssert( tc, "Subsequent calls should not work.", moveMotor( 0 ) == 0 );

    CuAssert( tc, "Resetting should make it work.", moveMotor( -1 ) == 1 );
    CuAssert( tc, "Cannot reset twice in a row.", moveMotor( -1 ) == 0 );

    CuAssert( tc, "Can make another normal call.", moveMotor( 0 ) == 1 );
    CuAssert( tc, "Resetting should make it work.", moveMotor( -1 ) == 1 );
}

CuSuite* CuGetSuite( void ) {
    CuSuite* suite = CuSuiteNew();

    SUITE_ADD_TEST( suite, schedulerInitTest );
    SUITE_ADD_TEST( suite, motorMovementTest );

    return suite;
}