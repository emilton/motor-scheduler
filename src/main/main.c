#include <limits.h>

#include "DSP28x_Project.h"
#include "F2802x_Device.h"

#include "f2802x_common/include/clk.h"
#include "f2802x_common/include/flash.h"
#include "f2802x_common/include/gpio.h"
#include "f2802x_common/include/pie.h"
#include "f2802x_common/include/pll.h"
#include "f2802x_common/include/spi.h"
#include "f2802x_common/include/timer.h"
#include "f2802x_common/include/wdog.h"

#include "comm.h"
#include "scheduler.h"

#define X_STEP GPIO_Number_0
#define Y_STEP GPIO_Number_2
#define Z_STEP GPIO_Number_7
#define A_STEP GPIO_Number_5
#define X_DIRECTION 4
#define Y_DIRECTION 6
#define Z_DIRECTION GPIO_Number_1
#define A_DIRECTION GPIO_Number_6
#define X_HOME GPIO_Number_4
#define Y_HOME GPIO_Number_3
#define Z_HOME GPIO_Number_32
#define A_HOME GPIO_Number_33
#define DRIVER_ENABLE 2
#define FAULT GPIO_Number_12

#pragma CODE_SECTION( commandReceive, "ramfuncs" );
#pragma CODE_SECTION( readSpi, "ramfuncs" );
#pragma CODE_SECTION( updateMotorsInterrupt, "ramfuncs" );

static void commandReceive( void );
static uint8_t readSpi( void );
static void handlesInit( void );
static void gpioInit( void );
static void spiInit( void );
static void interruptInit( void );
static interrupt void updateMotorsInterrupt( void );
static void clearMotors( void );

CLK_Handle myClk;
CPU_Handle myCpu;
#ifdef _FLASH
FLASH_Handle myFlash;
#endif
GPIO_Handle myGpio;
PIE_Handle myPie;
PLL_Handle myPll;
SPI_Handle mySpi;
TIMER_Handle myTimer;
WDOG_Handle myWDog;

static const GPIO_Number_e motors[NUM_MOTORS] = {X_STEP, Y_STEP, Z_STEP};
static const int motorDirections[NUM_MOTORS - 1] = {X_DIRECTION, Y_DIRECTION};
static Command_t commandArray[3] = {0};
static int commandCount = 2;

void main( void ) {
    handlesInit();
    gpioInit();
    spiInit();
    interruptInit();
    schedulerInit();
    commandReceive();
}

static uint8_t waitSpi( void ) {
    uint8_t readData;

    while( SPI_getRxFifoStatus( mySpi ) == SPI_FifoStatus_Empty ) {}
    readData = SPI_read( mySpi );
    SPI_write8( mySpi, 0xA5 );

    return readData;
}

static void commandReceive( void ) {
    uint16_t command[ sizeof( Command_t ) + 1];
    size_t i = sizeof( uint8_t ), j;
    uint8_t readData;

    /*
    uint8_t readData = waitSpi();
    while( !readData )
    	readData = waitSpi();
    */

    while( !waitSpi() )
	for( i = 0; i < sizeof( command )*2*3; i++ ) {
		readData = readSpi();/*
		for( j = 0; j < 2; j++ ) {
			command[i] >>= 8;
			readData = readSpi();
			command[i] |= ( readData << 8 );
		}*/
	}
}

static uint8_t readSpi( void ) {
    uint8_t readData;

    while( SPI_getRxFifoStatus( mySpi ) == SPI_FifoStatus_Empty ) {}
    readData = SPI_read( mySpi );
    SPI_write8( mySpi, readData );

    return readData;
}

void getNewCommand( void ){
	 commandCount++;
	 if( commandCount >= 3 ){
		 commandReceive();
		 commandCount  = 0;
	 }
	 //applyCommand( &commandArray[commandCount] );
}

static void handlesInit( void ) {
    myClk = CLK_init( ( void * )CLK_BASE_ADDR, sizeof( CLK_Obj ) );
    myCpu = CPU_init( ( void * )NULL, sizeof( CPU_Obj ) );
#ifdef _FLASH
    myFlash = FLASH_init( (void * )FLASH_BASE_ADDR, sizeof( FLASH_Obj ) );
#endif
    myGpio = GPIO_init( ( void * )GPIO_BASE_ADDR, sizeof( GPIO_Obj ) );
    myPie = PIE_init( ( void * )PIE_BASE_ADDR, sizeof( PIE_Obj ) );
    myPll = PLL_init( ( void * )PLL_BASE_ADDR, sizeof( PLL_Obj ) );
    mySpi = SPI_init( ( void * )SPIA_BASE_ADDR, sizeof( SPI_Obj ) );
    myTimer = TIMER_init( ( void * )TIMER0_BASE_ADDR, sizeof( TIMER_Obj ));
    myWDog = WDOG_init( ( void * )WDOG_BASE_ADDR, sizeof( WDOG_Obj ));

    // Perform basic system initialization
    WDOG_disable( myWDog );
    CLK_enableAdcClock( myClk );
    ( *Device_cal )();

    // Enable SPI-A Clock
    CLK_enableSpiaClock( myClk );

    //Select the internal oscillator 1 as the clock source
    CLK_setOscSrc( myClk, CLK_OscSrc_Internal );

    PLL_setup( myPll, PLL_Multiplier_12, PLL_DivideSelect_ClkIn_by_1 );

    // Disable the PIE and all interrupts
    PIE_disable( myPie );
    PIE_disableAllInts( myPie );
    CPU_disableGlobalInts( myCpu );
    CPU_clearIntFlags( myCpu );

    // If running from flash copy RAM only functions to RAM
#ifdef _FLASH
    memcpy( &RamfuncsRunStart, &RamfuncsLoadStart, ( size_t )&RamfuncsLoadSize );
#endif

    // Setup a debug vector table and enable the PIE
#ifdef _DEBUG
    PIE_setDebugIntVectorTable( myPie );
#endif
    PIE_enable( myPie );
}

static void gpioInit( void ) {
    GPIO_setMode( myGpio, X_STEP, GPIO_0_Mode_GeneralPurpose );
    GPIO_setMode( myGpio, Y_STEP, GPIO_0_Mode_GeneralPurpose );
    GPIO_setMode( myGpio, Z_STEP, GPIO_0_Mode_GeneralPurpose );
    GPIO_setMode( myGpio, A_STEP, GPIO_0_Mode_GeneralPurpose );
    GPIO_setMode( myGpio, X_HOME, GPIO_0_Mode_GeneralPurpose );
    GPIO_setMode( myGpio, Y_HOME, GPIO_0_Mode_GeneralPurpose );
    GPIO_setMode( myGpio, Z_HOME, GPIO_0_Mode_GeneralPurpose );
    GPIO_setMode( myGpio, A_HOME, GPIO_0_Mode_GeneralPurpose );
    GPIO_setMode( myGpio, Z_DIRECTION, GPIO_0_Mode_GeneralPurpose );
    GPIO_setMode( myGpio, A_DIRECTION, GPIO_0_Mode_GeneralPurpose );
    GPIO_setMode( myGpio, FAULT, GPIO_0_Mode_GeneralPurpose );
    GPIO_setDirection( myGpio, X_STEP, GPIO_Direction_Output );
    GPIO_setDirection( myGpio, Y_STEP, GPIO_Direction_Output );
    GPIO_setDirection( myGpio, Z_STEP, GPIO_Direction_Output );
    GPIO_setDirection( myGpio, A_STEP, GPIO_Direction_Output );
    GPIO_setDirection( myGpio, X_HOME, GPIO_Direction_Output );
    GPIO_setDirection( myGpio, Y_HOME, GPIO_Direction_Output );
    GPIO_setDirection( myGpio, Z_HOME, GPIO_Direction_Output );
    GPIO_setDirection( myGpio, A_HOME, GPIO_Direction_Output );
    GPIO_setDirection( myGpio, Z_DIRECTION, GPIO_Direction_Output );
    GPIO_setDirection( myGpio, A_DIRECTION, GPIO_Direction_Output );
    GPIO_setDirection( myGpio, FAULT, GPIO_Direction_Input );
    GPIO_setHigh( myGpio, X_STEP );
    GPIO_setHigh( myGpio, Y_STEP );
    GPIO_setHigh( myGpio, Z_STEP );
    GPIO_setHigh( myGpio, A_STEP );
    GPIO_setHigh( myGpio, Z_DIRECTION );
    GPIO_setHigh( myGpio, A_DIRECTION );

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    ( ( GPIO_Obj* )myGpio )->AIOMUX1 = 0;
    ( ( GPIO_Obj* )myGpio )->AIODIR = ( 1 << X_DIRECTION ) | ( 1 << Y_DIRECTION ) | ( 1 << DRIVER_ENABLE );
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    GPIO_setPullUp( myGpio, GPIO_Number_16, GPIO_PullUp_Enable );
    GPIO_setPullUp( myGpio, GPIO_Number_17, GPIO_PullUp_Enable );
    GPIO_setPullUp( myGpio, GPIO_Number_18, GPIO_PullUp_Enable );
    GPIO_setPullUp( myGpio, GPIO_Number_19, GPIO_PullUp_Enable );
    GPIO_setQualification( myGpio, GPIO_Number_16, GPIO_Qual_ASync );
    GPIO_setQualification( myGpio, GPIO_Number_17, GPIO_Qual_ASync );
    GPIO_setQualification( myGpio, GPIO_Number_18, GPIO_Qual_ASync );
    GPIO_setQualification( myGpio, GPIO_Number_19, GPIO_Qual_ASync );
    GPIO_setMode( myGpio, GPIO_Number_16, GPIO_16_Mode_SPISIMOA );
    GPIO_setMode( myGpio, GPIO_Number_17, GPIO_17_Mode_SPISOMIA );
    GPIO_setMode( myGpio, GPIO_Number_18, GPIO_18_Mode_SPICLKA );
    GPIO_setMode( myGpio, GPIO_Number_19, GPIO_19_Mode_SPISTEA_NOT );
}

static void spiInit( void ) {
    SPI_reset( mySpi );

    SPI_setCharLength( mySpi, SPI_CharLength_8_Bits );
    SPI_setClkPhase( mySpi, SPI_ClkPhase_Normal );
    SPI_setClkPolarity( mySpi, SPI_ClkPolarity_OutputFallingEdge_InputRisingEdge );
    SPI_setMode( mySpi, SPI_Mode_Slave );
    SPI_enableTx( mySpi );
    SPI_setTxDelay( mySpi, 0 );

    // Initialize SPI FIFO registers
    SPI_enableFifoEnh( mySpi );
    SPI_enableChannels( mySpi );
    SPI_resetTxFifo( mySpi );
    SPI_resetRxFifo( mySpi );
    SPI_enableTxFifo( mySpi );
    SPI_enableRxFifo( mySpi );

    // Set so breakpoints don't disturb transmission
    SPI_setPriority( mySpi, SPI_Priority_Immediate );

    SPI_enable( mySpi );
}

static void interruptInit( void ) {
    // Register interrupt handlers in the PIE vector table
    PIE_registerPieIntHandler( myPie, PIE_GroupNumber_1, PIE_SubGroupNumber_7, ( intVec_t )&updateMotorsInterrupt );

    TIMER_stop( myTimer );
    TIMER_setPeriod( myTimer, 2400 );
    TIMER_setPreScaler( myTimer, 0 );
    TIMER_reload( myTimer );
    TIMER_setEmulationMode( myTimer, TIMER_EmulationMode_StopAfterNextDecrement );
    TIMER_enableInt( myTimer );
    TIMER_start( myTimer );

    CPU_enableInt( myCpu, CPU_IntNumber_1 );
    PIE_enableTimer0Int( myPie );

    // Enable global Interrupts and higher priority real-time debug events
    CPU_enableGlobalInts( myCpu );
    CPU_enableDebugInt( myCpu );
}

static interrupt void updateMotorsInterrupt( void ) {
    clearMotors();
    updateMotors();

    // Acknowledge this interrupt to receive more interrupts from group 1
    PIE_clearInt( myPie, PIE_GroupNumber_1 );
}

static void clearMotors( void ) {
    int i;

    for( i = 0; i < NUM_MOTORS; i++ ) {
        GPIO_setLow( myGpio, motors[i] );
    }
}

int setDirection( int motorNumber, int forwardDirection ) {
	if( motorNumber < 2 ) {
		int motorDirection = motorDirections[motorNumber];
		if( forwardDirection ) {
			( ( GPIO_Obj* )myGpio )->AIOSET = ( 1 << motorDirection );
		} else {
			( ( GPIO_Obj* )myGpio )->AIOCLEAR = ( 1 << motorDirection );
		}
	} else {
		if( forwardDirection ) {
			GPIO_setHigh( myGpio, Z_DIRECTION );
		} else {
			GPIO_setLow( myGpio, Z_DIRECTION );
		}
	}
    return 1;
}

int moveMotor( int motorNumber ) {
    GPIO_setHigh( myGpio, motors[motorNumber] );
    return 1;
}
