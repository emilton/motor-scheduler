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
#include "f2802x_common/include/pwm.h"

#include "comm.h"
#include "scheduler.h"

#define X_STEP GPIO_Number_0
#define Y_STEP GPIO_Number_2
#define Z_STEP GPIO_Number_7
#define A_STEP GPIO_Number_5

#define X_DIRECTION GPIO_Number_4
#define Y_DIRECTION GPIO_Number_6
#define Z_DIRECTION GPIO_Number_1
#define A_DIRECTION GPIO_Number_6

#define X_HOME GPIO_Number_4
#define Y_HOME GPIO_Number_3
#define Z_HOME GPIO_Number_32
#define A_HOME GPIO_Number_33

#define PI_EMERGENCY_STOP GPIO_Number_28
#define DRIVER_ENABLE GPIO_Number_2
#define FAULT GPIO_Number_12

#define ePWM3Period 30000

static void commandReceive( void );
static uint8_t readSpi( void );
static void getNewCommand( void );
static void handleInit( void );
static void gpioInit( void );
static void spiInit( void );
static void interruptInit( void );
static interrupt void updateMotorsInterrupt( void );
static void clearMotors( void );
static void InitEPwm1( void );

#ifdef _FLASH
FLASH_Handle myFlash;
#endif

CLK_Handle myClk;
CPU_Handle myCpu;
GPIO_Handle myGpio;
PIE_Handle myPie;
PLL_Handle myPll;
SPI_Handle mySpi;
TIMER_Handle myTimer;
WDOG_Handle myWDog;
PWM_Handle myPwm3;

static const GPIO_Number_e motorSteps[NUM_MOTORS+NUM_WORKHEADS] = {X_STEP, Y_STEP, Z_STEP, A_STEP};
static const GPIO_Number_e motorDirections[NUM_MOTORS+NUM_WORKHEADS] = {X_DIRECTION, Y_DIRECTION, Z_DIRECTION, A_DIRECTION};
static const GPIO_Number_e motorHomes[NUM_MOTORS+NUM_WORKHEADS] = {X_HOME, Y_HOME, Z_HOME, A_HOME};

static Command_t commandArray[3];
static int numberCommands = 0;
static int commandCount = 0;

static int shouldGetNextCommand = 1;

void main( void ) {
    handleInit();
    InitEPwm1();
    gpioInit();
    spiInit();
    schedulerInit();
    interruptInit();

    for( ;; ) {
        listenForShutdown();
        if( shouldGetNextCommand ) {
            getNewCommand();
            shouldGetNextCommand = 0;
        }
    }
}

int listenForShutdown( void ) {
    if( !GPIO_getData( myGpio, PI_EMERGENCY_STOP ) ) {
        schedulerInit();
        return 1;
    }
    return 0;
}

static int waitSpi( void ) {
    uint8_t readData;
    for( ;; ) {
        while( SPI_getRxFifoStatus( mySpi ) == SPI_FifoStatus_Empty ) {}
        readData = ( SPI_read( mySpi ) & 0xFF );
        if( readData == 0x33 ) {
            SPI_write8( mySpi, 0 );
            return 3;
        } else if ( readData == 0x11 ) {
            SPI_write8( mySpi, 0 );
            return 1;
        }else {
            SPI_write8( mySpi, 0xA5 );
        }
    }
}

static void commandReceive( void ) {
    int i, wordsToReceive;
    uint16_t *commands = ( uint16_t* ) commandArray;

    SPI_enable( mySpi );

    numberCommands = waitSpi();
    wordsToReceive = numberCommands * sizeof( Command_t );
    for( i = 0; i < wordsToReceive; i++ ) {
        commands[i] = readSpi();
        commands[i] <<= 8;
        commands[i] |= readSpi() & 0xFF;
    }
    readSpi();
    readSpi();

    SPI_disable( mySpi );
}

static uint8_t readSpi( void ) {
    uint8_t readData;

    while( SPI_getRxFifoStatus( mySpi ) == SPI_FifoStatus_Empty ) {}
    readData = SPI_read( mySpi );
    SPI_write8( mySpi, ~readData );

    return readData;
}


static void getNewCommand( void ){
    int i;
    commandCount++;
    if( commandCount >= numberCommands ) {
        TIMER_disableInt( myTimer );
        commandReceive();
        if( commandArray[0].commandType == Accelerating ) {
            for( i = 0; i < NUM_MOTORS; i++ ) {
                setDirection( i, sign( commandArray[i].command.accelerating.accelerations[i] ) );
            }
        } else if( commandArray[0].commandType == Home ) {
            for( i = 0; i < NUM_MOTORS; i++ ) {
                setDirection( i, sign( commandArray[i].command.home.accelerations[i] ) );
            }
        }
        commandCount  = 0;
        TIMER_enableInt( myTimer );
    }

    applyCommand( &commandArray[commandCount] );
}

static void handleInit( void ) {
    myClk  = CLK_init( ( void * )CLK_BASE_ADDR, sizeof( CLK_Obj ) );
    myCpu  = CPU_init( ( void * )NULL, sizeof( CPU_Obj ) );
#ifdef _FLASH
    myFlash = FLASH_init( (void * )FLASH_BASE_ADDR, sizeof( FLASH_Obj ) );
#endif
    myGpio = GPIO_init( ( void * )GPIO_BASE_ADDR, sizeof( GPIO_Obj ) );
    myPie  = PIE_init( ( void * )PIE_BASE_ADDR, sizeof( PIE_Obj ) );
    myPll  = PLL_init( ( void * )PLL_BASE_ADDR, sizeof( PLL_Obj ) );
    mySpi  = SPI_init( ( void * )SPIA_BASE_ADDR, sizeof( SPI_Obj ) );
    myTimer= TIMER_init( ( void * )TIMER0_BASE_ADDR, sizeof( TIMER_Obj ));
    myWDog = WDOG_init( ( void * )WDOG_BASE_ADDR, sizeof( WDOG_Obj ));
    myPwm3 = PWM_init((void *)PWM_ePWM3_BASE_ADDR, sizeof(PWM_Obj));

    // Perform basic system initialization
    WDOG_disable( myWDog );
    CLK_enableAdcClock( myClk );
    ( *Device_cal )();
    CLK_enableSpiaClock( myClk );
    CLK_setOscSrc( myClk, CLK_OscSrc_Internal );
    PLL_setup( myPll, PLL_Multiplier_12, PLL_DivideSelect_ClkIn_by_1 );
    PIE_disable( myPie );
    PIE_disableAllInts( myPie );
    CPU_disableGlobalInts( myCpu );
    CPU_clearIntFlags( myCpu );

    #ifdef _DEBUG
        PIE_setDebugIntVectorTable( myPie );
    #endif

    PIE_enable( myPie );
}

static void gpioInit( void ) {
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    (( GPIO_Obj* )myGpio )->GPAMUX1 = 0;
    (( GPIO_Obj* )myGpio )->GPBMUX1 = 0;
    (( GPIO_Obj* )myGpio )->AIOMUX1 = 0;
    (( GPIO_Obj* )myGpio )->GPADIR |=  (( 1 << X_STEP ) | ( 1 << Y_STEP ) | ( 1 << Z_STEP ) | ( 1 << A_STEP ));
    (( GPIO_Obj* )myGpio )->GPADIR &= ~(( 1 << X_HOME ) | ( 1 << Y_HOME )  | ( 1 << FAULT ));
    (( GPIO_Obj* )myGpio )->GPBDIR &= ~(( 1 << (Z_HOME - GPIO_Number_32) ) | ( 1 << (A_HOME - GPIO_Number_32) ));
    (( GPIO_Obj* )myGpio )->GPADIR |=  (( 1 << Z_DIRECTION ) | ( 1 << A_DIRECTION ));
    (( GPIO_Obj* )myGpio )->AIODIR |=  (( 1 << X_DIRECTION ) | ( 1 << Y_DIRECTION ) | ( 1 << DRIVER_ENABLE ));
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    (( GPIO_Obj* )myGpio )->GPACLEAR = ( 1 << X_STEP ) | ( 1 << Y_STEP ) | ( 1 << Z_STEP ) | ( 1 << A_STEP );
    (( GPIO_Obj* )myGpio )->GPASET = ( 1 << Z_DIRECTION ) | ( 1 << A_DIRECTION );
    (( GPIO_Obj* )myGpio )->AIOSET = ( 1 << X_DIRECTION ) | ( 1 << Y_DIRECTION ) | ( 1 << DRIVER_ENABLE );
/*
    ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    (( GPIO_Obj* )myGpio )->GPAPUD   &= ~(( 1 << GPIO_Number_16 ) | ( 1 << GPIO_Number_17 ) | ( 1 << GPIO_Number_18 ) | ( 1 << GPIO_Number_19 ));
    (( GPIO_Obj* )myGpio )->GPAQSEL2 &= ~(( GPIO_Qual_ASync << (2*(GPIO_Number_16-GPIO_Rsvd_15))) | ( GPIO_Qual_ASync << (2*(GPIO_Number_17-GPIO_Rsvd_15))) | ( GPIO_Qual_ASync << (2*(GPIO_Number_18-GPIO_Rsvd_15))) | ( GPIO_Qual_ASync << (2*(GPIO_Number_19-GPIO_Rsvd_15))));
    DISABLE_PROTECTED_REGISTER_WRITE_MODE;
*/
    // Left as-is for time being, setMode is a little more complex.
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

    GPIO_setPullUp( myGpio, PI_EMERGENCY_STOP, GPIO_PullUp_Enable );
    GPIO_setMode( myGpio, PI_EMERGENCY_STOP, GPIO_28_Mode_GeneralPurpose );
    GPIO_setDirection( myGpio, PI_EMERGENCY_STOP, GPIO_Direction_Input );

	GPIO_setPullUp(myGpio, GPIO_Number_5, GPIO_PullUp_Disable);
	GPIO_setMode(myGpio, GPIO_Number_5, GPIO_5_Mode_EPWM3B);
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
    CPU_enableGlobalInts( myCpu );
    CPU_enableDebugInt( myCpu );
}

// Switched from PWM1A (worked) to PWM3B (needs verification)
static void InitEPwm1( void ) {
	CLK_disableTbClockSync(myClk);
	CLK_enablePwmClock(myClk, PWM_Number_3);

	// Setup TBCLK
	PWM_setCounterMode(myPwm3, PWM_CounterMode_Up);     // Count up
	PWM_setPeriod(myPwm3, ePWM3Period);                 // Set timer period
	PWM_disableCounterLoad(myPwm3);                     // Disable phase loading
	PWM_setPhase(myPwm3, 0x0000);                       // Phase is 0
	PWM_setCount(myPwm3, 0x0000);                       // Clear counter
	PWM_setHighSpeedClkDiv(myPwm3, PWM_HspClkDiv_by_2); // Clock ratio to SYSCLKOUT
	PWM_setClkDiv(myPwm3, PWM_ClkDiv_by_2);

	// Setup shadow register load on ZERO
	PWM_setShadowMode_CmpA(myPwm3, PWM_ShadowMode_Shadow);
	PWM_setShadowMode_CmpB(myPwm3, PWM_ShadowMode_Shadow);
	PWM_setLoadMode_CmpA(myPwm3, PWM_LoadMode_Zero);
	PWM_setLoadMode_CmpB(myPwm3, PWM_LoadMode_Zero);

	// Set Compare values
	PWM_setCmpA(myPwm3, 5000);    // Set compare A value
	PWM_setCmpB(myPwm3, 7500);    // Set Compare B value

	// Set Actions
	PWM_setActionQual_Zero_PwmA(myPwm3, PWM_ActionQual_Set);
	PWM_setActionQual_CntUp_CmpA_PwmA(myPwm3, PWM_ActionQual_Clear);

	PWM_setActionQual_Zero_PwmB(myPwm3, PWM_ActionQual_Set);
	PWM_setActionQual_CntUp_CmpB_PwmB(myPwm3, PWM_ActionQual_Clear);

	CLK_enableTbClockSync(myClk); // remove for release?
}
static interrupt void updateMotorsInterrupt( void ) {
    clearMotors();
    if( updateMotors() == -1 ) {
        shouldGetNextCommand = 1;
    }
    PIE_clearInt( myPie, PIE_GroupNumber_1 );
}

static void clearMotors( void ) {
    int i;
    for( i = 0; i < NUM_MOTORS; i++ ) {
        (( GPIO_Obj* )myGpio )->GPACLEAR = ( 1 << motorSteps[i] );
    }
}

int setWorkHead( int dutyCycle ) {
	if( dutyCycle <= 0 || dutyCycle >= 100 ) {
	    CLK_disableTbClockSync(myClk);
		GPIO_setPullUp(myGpio, A_STEP, GPIO_PullUp_Enable);
		GPIO_setMode(myGpio, A_STEP, GPIO_5_Mode_GeneralPurpose);
	}
	else {
		GPIO_setPullUp(myGpio, A_STEP, GPIO_PullUp_Disable);
		GPIO_setMode(myGpio, A_STEP, GPIO_5_Mode_EPWM3B);
		PWM_setCmpB(myPwm3, (dutyCycle*ePWM3Period)/100);
		CLK_enableTbClockSync(myClk);
	}
	return 1;
}

int moveMotor( int i ) {
    GPIO_setHigh( myGpio, motorSteps[i] );
    return 1;
}

int isHomed( int motorNumber ) {
    return !GPIO_getData( myGpio, motorHomes[motorNumber] );
}

int setDirection( int motorNumber, int forwardDirection ) {
    if( motorNumber < 2 ){
        if( forwardDirection ) {
            ( ( GPIO_Obj* )myGpio )->AIOSET   = ( 1 << motorDirections[motorNumber] );
        } else {
            ( ( GPIO_Obj* )myGpio )->AIOCLEAR = ( 1 << motorDirections[motorNumber] );
        }
    } else {
        if( forwardDirection ) {
            ( ( GPIO_Obj* )myGpio )->GPASET   = ( 1 << motorDirections[motorNumber] );
        } else {
            ( ( GPIO_Obj* )myGpio )->GPACLEAR = ( 1 << motorDirections[motorNumber] );
        }
    }
    return 1;
}
